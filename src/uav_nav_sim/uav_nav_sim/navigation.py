"""Goal supervisor owned by the single-threaded Gazebo executor.

Sharing ownership makes cancel, replacement and trajectory acceptance atomic.
Only the executor publishes velocity; observation turns cannot fight flight commands.
"""
import math
import time

import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
from uav_nav_interfaces.msg import PlannerStatus

from .exploration import ExplorationSettings, choose_subgoal, segment_free
from .core import CellState
from .autonomous import AutonomousExplorer


def stamp_key(stamp):
    return stamp.sec*1_000_000_000+stamp.nanosec


class GoalManager:
    def __init__(self, executor):
        self.node = executor
        executor.declare_parameter('explore_unknown', True)
        self.explore = executor.get_parameter('explore_unknown').value
        values = {}
        for name, value in vars(ExplorationSettings()).items():
            executor.declare_parameter('exploration.'+name, value)
            values[name] = executor.get_parameter('exploration.'+name).value
        self.settings = ExplorationSettings(**values)
        self.goal = self.local = None
        self.token = self.last_token = 0
        self.state = 'IDLE'
        self.reason = ''
        self.phase = 'IDLE'
        self.local_final = False
        self.local_pub = executor.create_publisher(PoseStamped, '/uav/local_goal', 10)
        self.status = executor.create_publisher(String, '/uav/navigation/state',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        executor.create_subscription(PoseStamped, '/uav/goal', self.goal_cb, 10)
        executor.create_subscription(PlannerStatus, '/uav/planner/result', self.planner_cb, 10)
        self.autonomous = AutonomousExplorer(self)
        executor.create_timer(0.5, self.publish_status)
        self.publish_status()

    def publish_status(self):
        self.autonomous.watchdog()
        self.autonomous.publish()
        self.status.publish(String(data=self.state+(':'+self.reason if self.reason else '')))

    def report(self, state, reason=''):
        if (state, reason) != (self.state, self.reason):
            self.node.get_logger().info(f'Navigation {state}: {reason}')
        self.state, self.reason = state, reason
        self.publish_status()

    def finish(self, state, reason=''):
        self.goal = self.local = None
        self.token = 0
        self.phase = 'IDLE'
        self.report(state, reason)
        self.autonomous.finished(state, reason)

    def goal_cb(self, msg, autonomous=False):
        p = msg.pose.position
        goal = np.array([p.x, p.y, p.z])
        if msg.header.frame_id != 'map' or not np.all(np.isfinite(goal)):
            self.node.get_logger().warning('Rejected goal: requires finite map-frame XYZ')
            return
        if not autonomous:
            self.autonomous.disable('DISABLED:MANUAL_GOAL')
        self.node.stop('GOAL_REPLACED')
        self.goal = goal
        self.visited, self.rejected = [], []
        self.failures = self.segments = 0
        self.started = self.progress_at = time.monotonic()
        self.best_distance = float('inf')
        self.no_candidate_at = None
        self.last_selection = 0.0
        self.last_known_probe = float('-inf')
        self.observe_since = None
        self.scan_index = 0
        self.scan_remaining = self.stagnant_segments = 0
        self.observed_at_progress = int(np.count_nonzero(self.node.grid.observed)) if self.node.grid is not None else 0
        self.phase = 'OBSERVE'
        self.report('OBSERVING')

    def accepts(self, msg):
        return self.goal is not None and self.phase == 'PLANNING' and self.token != 0 and stamp_key(msg.goal_stamp) == self.token

    def accepted(self):
        self.phase = 'EXECUTING'
        self.report('NAVIGATING' if self.local_final else 'EXPLORING')

    def failed_segment(self, reason):
        if self.goal is None:
            return
        if self.local is not None:
            self.rejected.append(self.local.copy())
        self.local = None
        self.token = 0
        self.failures += 1
        if self.failures >= self.settings.max_failures:
            self.finish('BLOCKED', reason)
        else:
            self.phase = 'OBSERVE'
            self.observe_since = None
            self.report('OBSERVING', reason)

    def planner_cb(self, msg):
        if self.goal is None or self.phase != 'PLANNING' or stamp_key(msg.goal_stamp) != self.token:
            return
        if msg.state in ('WAIT_MAP_OR_ODOMETRY', 'WAIT_STOPPED', 'SAFE_SEED_FALLBACK', 'TRAJECTORY_PUBLISHED'):
            return
        if msg.state == 'GOAL_REACHED':
            # Planner uses a wider tolerance than the executor: still verify the
            # actual position, and never count this as final arrival unconditionally.
            if np.linalg.norm(self.position()-self.local) < 0.15:
                self.on_stop('LOCAL_GOAL_REACHED')
            else:
                self.failed_segment('INVALID_PLANNER_ARRIVAL')
        else:
            self.failed_segment(msg.state)

    def on_stop(self, reason):
        if reason in ('GOAL_REPLACED', 'CANCELLED', 'MAP_SESSION_CHANGED'):
            self.finish('CANCELLED', reason)
        elif self.goal is None:
            if self.autonomous.enabled and reason not in ('GOAL_REACHED', 'LOCAL_GOAL_REACHED'):
                self.finish('STOPPED', reason)
            return
        elif reason in ('GOAL_REACHED', 'LOCAL_GOAL_REACHED'):
            self.token = 0
            if np.linalg.norm(self.position()-self.goal) < 0.15:
                self.finish('REACHED')
                return
            if self.local is not None:
                self.visited.append(self.local.copy())
            self.local = None
            self.segments += 1
            self.failures = 0
            if self.explore and not self.node.map.static_map:
                # A useful detour can increase goal distance while revealing a
                # way around an obstacle. Count new map volume only on arrival;
                # idle scans alone cannot indefinitely renew the task budget.
                observed = int(np.count_nonzero(self.node.grid.observed))
                gain = (observed-self.observed_at_progress)*self.node.grid.resolution**3
                if gain >= self.settings.minimum_map_gain:
                    self.progress_at = time.monotonic()
                    self.observed_at_progress = observed
                if np.linalg.norm(self.position()-self.goal) >= self.best_distance-0.2:
                    self.stagnant_segments += 1
                else:
                    self.stagnant_segments = 0
                if self.stagnant_segments >= 2:
                    self.scan_remaining = 3
                    self.stagnant_segments = 0
            self.phase = 'OBSERVE'
            self.observe_since = None
            self.scan_index = 0
            self.report('OBSERVING')
        elif reason.startswith('MAP_RECHECK_FAILED') or reason == 'CURRENT_VOLUME_BLOCKED':
            self.failed_segment(reason)
        else:
            # Health/clock/tracking faults latch until a new user goal. A fresh
            # map alone must not restart a task that has stopped on an error.
            self.finish('STOPPED', reason)

    def position(self):
        p = self.node.odom.pose.pose.position
        return np.array([p.x, p.y, p.z])

    def observation_command(self, position, now, require_observation=True, direction=None):
        command = Twist()
        if not self.explore or (self.node.map is not None and self.node.map.static_map):
            return command, True
        q = self.node.odom.pose.pose.orientation
        quaternion = np.array([q.x, q.y, q.z, q.w])
        if not np.all(np.isfinite(quaternion)) or np.linalg.norm(quaternion) < 1e-6:
            self.finish('STOPPED', 'INVALID_ORIENTATION')
            return command, False
        x, y, z, w = quaternion/np.linalg.norm(quaternion)
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        pitch = math.asin(float(np.clip(2*(w*y-z*x), -1., 1.)))
        roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        if not require_observation:
            # A fully observed direct corridor needs no camera scan, but body
            # leveling and invalid-orientation checks still precede planning.
            return command, abs(pitch) <= .06 and abs(roll) <= .06
        if direction is None:
            direction = self.goal-position
        desired = math.atan2(direction[1], direction[0])
        # With no candidate, sweep both sides and behind before declaring blocked.
        # Sweep adjacent quadrants in order instead of repeatedly reversing
        # through 180 degrees and spending the observation budget retracing.
        desired += (0.0, math.pi/2, math.pi, -math.pi/2)[self.scan_index % 4]
        error = math.atan2(math.sin(desired-yaw), math.cos(desired-yaw))
        # Observe by yawing only. Do not tilt the aircraft to compensate for
        # the fixed camera mount or the target's elevation. The executor levels
        # the body in every phase, including idle and terminal HOLD states.
        if abs(error) > 0.12 or abs(pitch) > .06 or abs(roll) > .06:
            command.angular.z = float(np.clip(2*error, -0.6, 0.6))
            self.observe_since = None
            return command, False
        if self.observe_since is None:
            self.observe_since = now
        return command, now-self.observe_since >= self.settings.observation_time

    def tick(self):
        """Called only while the executor is holding, returns a turn or zero twist."""
        command = Twist()
        if self.autonomous.enabled and self.autonomous.phase == 'SCAN':
            return self.autonomous.scan_tick()
        if self.goal is None:
            return command
        now = time.monotonic()
        node = self.node
        if node.odom is None or now-node.odom_wall > 0.5:
            if now-self.started > node.timeout:
                self.finish('STOPPED', 'STALE_ODOMETRY')
            return command
        odom_age = (node.get_clock().now().nanoseconds-stamp_key(node.odom.header.stamp))/1e9
        if not 0 <= odom_age <= 0.5:
            self.finish('STOPPED', 'STALE_ODOMETRY')
            return command
        position = self.position()
        if not np.all(np.isfinite(position)):
            self.finish('STOPPED', 'INVALID_ODOMETRY')
            return command
        if not node.ready():
            if now-self.started > node.timeout:
                self.finish('STOPPED', 'STALE_MAP_OR_ODOMETRY')
            return command
        if node.grid.state(self.goal) == CellState.OCCUPIED:
            self.finish('BLOCKED', 'KNOWN_GOAL_OCCUPIED')
            return command
        distance = np.linalg.norm(position-self.goal)
        if distance < 0.15:
            self.finish('REACHED')
            return command
        if distance < self.best_distance-0.2:
            self.best_distance = distance
            self.progress_at = now
        if now-self.progress_at > self.settings.progress_timeout or self.segments >= self.settings.max_segments:
            self.finish('BLOCKED', 'EXPLORATION_BUDGET')
            return command
        v = node.odom.twist.twist.linear
        if not np.all(np.isfinite([v.x, v.y, v.z])):
            self.finish('STOPPED', 'INVALID_ODOMETRY')
            return command
        if np.linalg.norm([v.x, v.y, v.z]) > 0.05:
            return command
        if self.phase == 'PLANNING':
            if now-self.sent_at > self.settings.planning_timeout:
                self.failed_segment('PLANNING_TIMEOUT')
            return command
        if self.phase != 'OBSERVE':
            return command
        result = None
        # Bound probing cost. Only skip observation for a checked direct route;
        # frontier exploration, failed candidates and active scans keep the
        # existing observation/retry policy. The planner and executor still
        # independently validate the actual timed curve before any movement.
        if (self.explore and not node.map.static_map and self.scan_index == 0 and
                self.scan_remaining == 0 and
                now-self.last_known_probe >= 0.5 and
                self.no_candidate_at is None):
            self.last_known_probe = now
            if (not any(np.linalg.norm(self.goal-p) < self.settings.revisit_radius
                        for p in self.rejected) and
                    not node.grid.collision(self.goal, node.radius) and
                    segment_free(node.grid, position, self.goal, node.radius)):
                result = (self.goal.copy(), True)
        command, observed = self.observation_command(position, now, require_observation=result is None)
        if self.goal is None:
            return Twist()
        # Deadline is checked even during a scan; inability to orient or find a
        # frontier must not keep the task alive indefinitely.
        if self.no_candidate_at is not None and now-self.no_candidate_at > self.settings.blocked_timeout:
            self.finish('BLOCKED', 'NO_REACHABLE_FRONTIER')
            return Twist()
        if not observed or now-self.last_selection < 0.5:
            return command
        if self.scan_remaining:
            self.scan_remaining -= 1
            self.scan_index += 1
            self.observe_since = None
            return Twist()
        self.last_selection = now
        if result is None:
            result = choose_subgoal(node.grid, position, self.goal, node.radius, self.settings,
                self.visited, self.rejected, self.explore and not node.map.static_map, self.best_distance)
        if result is None:
            if not self.explore or node.map.static_map:
                self.finish('BLOCKED', 'NO_KNOWN_PATH')
            else:
                if self.no_candidate_at is None:
                    self.no_candidate_at = now
                self.scan_index += 1
                self.observe_since = None
                self.report('OBSERVING', 'NO_REACHABLE_FRONTIER')
            return Twist()
        self.no_candidate_at = None
        self.local, self.local_final = result
        self.token = max(node.get_clock().now().nanoseconds, self.last_token+1)
        self.last_token = self.token
        message = PoseStamped()
        message.header.frame_id = 'map'
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(self.token, 1_000_000_000)
        message.pose.position.x, message.pose.position.y, message.pose.position.z = map(float, self.local)
        message.pose.orientation.w = 1.0
        self.sent_at = time.monotonic()
        self.phase = 'PLANNING'
        self.report('PLANNING')
        self.local_pub.publish(message)
        return Twist()
