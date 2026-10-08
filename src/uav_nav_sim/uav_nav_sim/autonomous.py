"""Autonomous frontier visits through the existing checked goal supervisor."""
import math
import time

import numpy as np
from scipy.ndimage import generate_binary_structure, label, maximum_filter, uniform_filter
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from std_srvs.srv import SetBool
from rclpy.qos import QoSProfile, DurabilityPolicy

from .exploration import segment_free


def frontier_viewpoint(grid, start, radius, visited=(), rejected=(), preferred_height=1.2):
    """Select a known, reachable viewpoint; unknown volume is only a gain proxy.

    No world geometry or simulated truth is used. Exhausting these viewpoints
    does not prove that unknown or inaccessible map regions have been covered.
    """
    start = np.asarray(start, dtype=float)
    r = grid.resolution
    radius += r/2
    if grid.collision(start, radius):
        return None
    unknown = ~grid.observed | ~np.isfinite(grid.distance)
    padding = int(np.ceil((radius+r/2)/r))
    near_blocked = maximum_filter(unknown | (grid.distance <= 0),
                                  size=2*padding+1, mode='constant', cval=1)
    safe = ~near_blocked & (grid.distance > radius+(np.sqrt(3)+1)*r/2)
    components, _ = label(safe, generate_binary_structure(3, 1))
    index = grid.index(start)
    component = components[tuple(index)]
    if component == 0:
        lo, hi = np.maximum(0, index-2), np.minimum(safe.shape, index+3)
        nearby = np.argwhere(safe[tuple(slice(a,b) for a,b in zip(lo,hi))])+lo
        for i in sorted(nearby, key=lambda i: np.linalg.norm(grid.origin+(i+.5)*r-start)):
            if segment_free(grid, start, grid.origin+(i+.5)*r, radius):
                component = components[tuple(i)]
                break
    if component == 0:
        return None
    fringe = maximum_filter(unknown, size=2*(padding+2)+1, mode='constant', cval=0)
    indices = np.argwhere((components == component) & fringe)
    if not len(indices):
        return None
    points = grid.origin+(indices+.5)*r
    distance = np.linalg.norm(points-start, axis=1)
    keep = distance >= .8
    for point in visited:
        keep &= np.linalg.norm(points-point, axis=1) >= 1.0
    for point in rejected:
        keep &= np.linalg.norm(points-point, axis=1) >= 1.0
    points, indices, distance = points[keep], indices[keep], distance[keep]
    if not len(points):
        return None
    # Bounded neighbourhood density, not a claim that every voxel is visible.
    gain = uniform_filter(unknown.astype(np.float32), size=2*int(np.ceil(2/r))+1,
                          mode='constant', cval=0)[tuple(indices.T)]
    clearance = grid.distance[tuple(indices.T)]-radius
    score = (gain/(1+.25*distance) - .06*np.abs(points[:,2]-preferred_height)
             - .03/np.maximum(clearance, 2*r))
    return points[int(np.argmax(score))].copy()


class AutonomousExplorer:
    """Runs in the executor thread; never publishes velocity or bypasses checks."""
    def __init__(self, manager):
        self.manager = manager
        self.node = manager.node
        self.enabled = self.dispatching = False
        self.phase = 'IDLE'
        self.state = 'DISABLED'
        self.status = self.node.create_publisher(String, '/uav/exploration/state',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.create_service(SetBool, '/uav/exploration/enabled', self.enable)
        self.publish()

    def watchdog(self):
        if not self.enabled:
            return
        now = time.monotonic()
        if self.node.grid is not None:
            observed = int(np.count_nonzero(self.node.grid.observed))
            if (observed-self.observed)*self.node.grid.resolution**3 >= .5:
                self.observed, self.progress_at = observed, now
        if now-self.started > 1800 or self.completed >= 120 or now-self.progress_at > 180:
            self.disable('LIMIT_REACHED')
            self.node.stop('CANCELLED')

    def publish(self):
        self.status.publish(String(data=self.state))

    def report(self, state):
        if state != self.state:
            self.node.get_logger().info('Autonomous exploration: '+state)
        self.state = state
        self.publish()

    def disable(self, reason):
        self.enabled = False
        self.phase = 'IDLE'
        self.report(reason)

    def enable(self, request, response):
        action = getattr(self.node, 'navigation_action', None)
        if action is not None and action.busy:
            response.success, response.message = False, 'Navigation Action owns control'
            return response
        if not request.data:
            self.disable('DISABLED')
            self.node.stop('CANCELLED')
            response.success, response.message = True, 'Autonomous exploration disabled; holding'
            return response
        if self.enabled:
            response.success, response.message = True, 'Autonomous exploration already active'
            return response
        if (not self.manager.explore or not self.node.ready() or self.node.map.static_map or
                self.node.grid.collision(self.manager.position(), self.node.radius+self.node.grid.resolution/2)):
            response.success, response.message = False, 'Require fresh online map and observed launch volume; run init'
            return response
        self.node.stop('CANCELLED')
        self.enabled = True
        self.visited, self.rejected = [], []
        self.target = None
        self.completed = self.failures = 0
        self.started = self.progress_at = time.monotonic()
        self.observed = int(np.count_nonzero(self.node.grid.observed))
        self.preferred_height = float(self.manager.position()[2])
        self.last_full_position = None
        self.quick_scans = 0
        self.dispatched_observed = self.observed
        self.begin_scan()
        response.success, response.message = True, 'Autonomous frontier exploration started'
        return response

    def begin_scan(self, force_full=False):
        position = self.manager.position()
        observed = int(np.count_nonzero(self.node.grid.observed))
        gain = (observed-self.dispatched_observed)*self.node.grid.resolution**3
        # In-motion mapping can make a repeated panoramic scan unnecessary.
        # Still scan fully on startup, after failures, every third arrival, or
        # after moving four metres from the last panoramic observation.
        brief = (not force_full and self.completed > 0 and self.failures == 0 and
                 self.last_full_position is not None and gain >= .5 and
                 self.quick_scans < 2 and
                 np.linalg.norm(position-self.last_full_position) < 4.)
        self.scan_views = 1 if brief else 4
        self.quick_scans = self.quick_scans+1 if brief else 0
        self.phase = 'SCAN'
        self.manager.scan_index = 0
        self.manager.observe_since = None
        self.scan_started = time.monotonic()
        # Start from the arrival heading; do not turn back to world east first.
        q = self.node.odom.pose.pose.orientation
        quaternion = np.array([q.x,q.y,q.z,q.w])
        yaw = 0.
        if np.all(np.isfinite(quaternion)) and np.linalg.norm(quaternion) > 1e-6:
            x,y,z,w = quaternion/np.linalg.norm(quaternion)
            yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        self.scan_direction = np.array([math.cos(yaw),math.sin(yaw),0.])
        self.report('SCANNING:BRIEF' if brief else 'SCANNING')
        self.manager.report('OBSERVING', 'AUTONOMOUS_BRIEF_SCAN' if brief else 'AUTONOMOUS_SCAN')

    def finished(self, state, reason):
        if not self.enabled or self.dispatching:
            return
        if state == 'REACHED':
            self.completed += 1
            self.failures = 0
            self.begin_scan()
        elif state == 'BLOCKED':
            if self.target is not None:
                self.rejected.append(self.target.copy())
            self.failures += 1
            if self.failures >= 8:
                self.disable('BLOCKED:REPEATED_PLANNING_FAILURE')
            else:
                self.begin_scan()
        else:
            self.disable(state+(':'+reason if reason else ''))

    def scan_tick(self):
        command = Twist()
        if not self.enabled:
            return command
        now = time.monotonic()
        if not self.node.ready():
            self.manager.finish('STOPPED', 'STALE_MAP_OR_ODOMETRY')
            return command
        position = self.manager.position()
        if not np.all(np.isfinite(position)) or self.node.grid.collision(position, self.node.radius):
            self.manager.finish('STOPPED', 'CURRENT_VOLUME_BLOCKED')
            return command
        self.watchdog()
        if not self.enabled:
            return command
        if now-self.scan_started > 60:
            self.disable('BLOCKED:SCAN_TIMEOUT')
            return command
        velocity = self.node.odom.twist.twist.linear
        if not np.all(np.isfinite([velocity.x, velocity.y, velocity.z])):
            self.manager.finish('STOPPED', 'INVALID_ODOMETRY')
            return command
        if np.linalg.norm([velocity.x, velocity.y, velocity.z]) > .05:
            return command
        command, observed = self.manager.observation_command(position, now, direction=self.scan_direction, reuse=self.scan_views == 1)
        if not self.enabled or not observed:
            return command
        self.manager.scan_index += 1
        self.manager.observe_since = None
        if self.manager.scan_index < self.scan_views:
            return Twist()
        target = frontier_viewpoint(self.node.grid, position, self.node.radius,
                                    [*self.visited, position], self.rejected, self.preferred_height)
        if target is None and self.scan_views == 1:
            self.begin_scan(force_full=True)
            return Twist()
        self.visited.append(position.copy())
        if self.scan_views == 4:
            self.last_full_position = position.copy()
        if target is None:
            self.disable('FRONTIERS_EXHAUSTED')
            return Twist()
        self.target = target
        self.dispatched_observed = int(np.count_nonzero(self.node.grid.observed))
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, target)
        msg.pose.orientation.w = 1.
        self.dispatching = True
        try:
            self.manager.goal_cb(msg, autonomous=True)
        finally:
            self.dispatching = False
        self.phase = 'GOAL'
        self.report('EXPLORING')
        self.node.get_logger().info(f'Autonomous viewpoint: {target.tolist()}')
        return Twist()
