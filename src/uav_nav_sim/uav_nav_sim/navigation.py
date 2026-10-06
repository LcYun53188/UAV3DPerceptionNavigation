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
from uav_nav_interfaces.msg import PlannerStatus, TrajectoryRequest

from .exploration import ExplorationSettings, choose_subgoal, segment_free
from .core import CellState
from .autonomous import AutonomousExplorer
from .background import BackgroundSelector
from .viewpoints import focused_scan_yaw


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
        executor.declare_parameter('continuous_navigation', True)
        executor.declare_parameter('moving_handover', True)
        self.continuous = executor.get_parameter('continuous_navigation').value
        self.moving_handover = executor.get_parameter('moving_handover').value
        executor.declare_parameter('reuse_inflight_observation', True)
        self.reuse_observation = executor.get_parameter('reuse_inflight_observation').value
        executor.declare_parameter('focused_observation', True)
        self.focused_observation = executor.get_parameter('focused_observation').value
        self.focused_yaw = None
        self.focused_checked = False
        self.reset_observation_credit()
        executor.declare_parameter('background_replan', True)
        self.background_replan = executor.get_parameter('background_replan').value
        self.background = BackgroundSelector()
        self.background_target = None
        self.replan = None
        self.last_replan_probe = float('-inf')
        self.replan_pub = executor.create_publisher(TrajectoryRequest, '/uav/replan_request', 10)
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
        self.background.invalidate()
        self.background_target = None
        retain = state == 'REACHED' and self.autonomous.enabled
        arrival_view, last = (self.arrival_view, self.flight_view_last) if retain else (None, None)
        self.reset_observation_credit()
        self.arrival_view, self.flight_view_last = arrival_view, last
        self.replan = None
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
        self.segment_start_distance = (float(np.linalg.norm(self.position()-goal))
                                       if self.node.odom is not None else float('inf'))
        self.no_candidate_at = None
        self.no_candidate_reason = 'NO_REACHABLE_FRONTIER'
        self.last_selection = 0.0
        self.last_known_probe = float('-inf')
        self.observe_since = None
        self.scan_index = 0
        self.focused_yaw = None
        self.focused_checked = False
        self.scan_remaining = self.stagnant_segments = 0
        self.observed_at_progress = int(np.count_nonzero(self.node.grid.observed)) if self.node.grid is not None else 0
        self.phase = 'OBSERVE'
        self.report('OBSERVING')

    def accepts(self, msg):
        if msg.parent_trajectory_id:
            return (self.goal is not None and self.phase == 'EXECUTING' and self.replan is not None and
                    stamp_key(msg.goal_stamp) == self.replan['token'] and
                    msg.parent_trajectory_id == self.replan['parent'] and
                    stamp_key(msg.start_time) == self.replan['start_ns'])
        return self.goal is not None and self.phase == 'PLANNING' and self.token != 0 and stamp_key(msg.goal_stamp) == self.token

    def replan_failed(self, reason):
        # A speculative successor must never invalidate the executing stop path.
        self.replan = None
        self.node.get_logger().info('Moving replan deferred: '+reason)

    def handover(self):
        # Record where we actually passed, rather than marking an unreached
        # observation target as visited. This discourages speculative loops.
        self.visited.append(self.position().copy())
        self.local, self.local_final = self.replan['target'], self.replan['final']
        self.token = self.replan['token']
        self.replan = None
        self.segments += 1
        self.accepted()

    def inflight_tick(self, t, duration):
        """Cheap, bounded straight-corridor probes while the old curve executes.

        Full connected-component/frontier searches remain in HOLD: they must not
        stall the 50 Hz control loop. No unseen space is used for a continuation.
        """
        node = self.node
        now = time.monotonic()
        self.track_flight_observation(now)
        try:
            result = self.background.poll()
            if result is not None:
                context, selection = result
                if (context == (node.session, node.trajectory.trajectory_id, tuple(self.goal)) and
                        selection is not None):
                    self.background_target = selection
                    node.event.publish(String(data='BACKGROUND_SEARCH_READY'))
        except Exception as exc:
            self.background_target = None
            node.get_logger().warning('Background replan deferred: '+str(exc))
        distance = np.linalg.norm(self.position()-self.goal) if self.goal is not None else 0.
        if self.goal is not None and distance < self.best_distance-.2:
            self.best_distance, self.progress_at = distance, now
        if self.replan is not None:
            if node.get_clock().now().nanoseconds >= self.replan['start_ns'] and node.pending is None:
                self.replan_failed('HANDOVER_TIMEOUT')
            return
        lead = 1.2
        if (not self.moving_handover or self.goal is None or self.local_final or self.local is None or
                self.phase != 'EXECUTING' or not lead+.5 < duration-t <= 6.0 or
                now-self.last_replan_probe < 1.0 or
                self.segments >= self.settings.max_segments or
                now-self.progress_at > self.settings.progress_timeout):
            return
        self.last_replan_probe = now
        start_t = t+lead
        start, velocity, acceleration = (node.curve(start_t, d) for d in (0,1,2))
        if np.linalg.norm(velocity) < .08:
            return
        direction = self.goal-self.local
        remaining = np.linalg.norm(direction)
        if remaining < .6:
            return
        # A nearby target with unseen/blocked body volume needs an actual
        # observation at this endpoint; chaining away would skip that view.
        if remaining < 2.5 and node.grid.collision(self.goal, node.radius+node.grid.resolution/2):
            return
        radius = node.radius+node.grid.resolution/2
        target, final = None, False
        from_background = False
        # The final corridor may have become observed during flight.
        candidates = [(self.goal, True)] if np.dot(direction, velocity) > 0 else []
        candidates += [(self.local+direction/remaining*min(step,remaining), False)
                       for step in (3., 2., 1.) if step < remaining and np.dot(direction, velocity) > 0]
        for candidate, is_final in candidates:
            if (any(np.linalg.norm(candidate-p) < self.settings.revisit_radius for p in self.rejected) or
                    np.linalg.norm(candidate-self.local) < .6):
                continue
            # Bound controller work even for a distant final goal.
            if np.linalg.norm(candidate-start) > 6.:
                continue
            if segment_free(node.grid, start, candidate, radius, 256):
                target, final = candidate.copy(), is_final
                break
        if target is None and self.background_target is not None:
            candidate, is_final = self.background_target
            self.background_target = None
            if (np.linalg.norm(candidate-self.local) >= .6 and
                    np.linalg.norm(candidate-start) <= 6. and
                    not node.grid.collision(start, radius) and
                    not node.grid.collision(candidate, radius) and
                    not any(np.linalg.norm(candidate-p) < self.settings.revisit_radius
                            for p in self.rejected)):
                target, final = candidate.copy(), is_final
                from_background = True
        if target is None:
            if (self.background_replan and self.explore and not node.map.static_map and
                    duration-t > lead+1.):
                try:
                    submitted = self.background.submit(
                        (node.session, node.trajectory.trajectory_id, tuple(self.goal)),
                        node.grid, start.copy(), self.goal.copy(), node.radius, self.settings,
                        tuple([*self.visited, self.local.copy()]), tuple(self.rejected), self.best_distance)
                    if submitted:
                        node.event.publish(String(data='BACKGROUND_SEARCH_STARTED'))
                except Exception as exc:
                    node.get_logger().warning('Background submit deferred: '+str(exc))
            return
        self.background.invalidate()
        clock_ns = node.get_clock().now().nanoseconds
        start_ns = stamp_key(node.trajectory.start_time)+int(start_t*1e9)
        if start_ns-clock_ns < 800_000_000:
            return
        token = max(clock_ns, self.last_token+1)
        self.last_token = token
        request = TrajectoryRequest()
        request.header.frame_id = 'map'
        request.header.stamp.sec, request.header.stamp.nanosec = divmod(token,1_000_000_000)
        request.start_time.sec, request.start_time.nanosec = divmod(start_ns,1_000_000_000)
        request.map_id, request.epoch = node.session
        request.parent_trajectory_id = node.trajectory.trajectory_id
        for field, value in [('start_position',start),('start_velocity',velocity),
                             ('start_acceleration',acceleration),('goal',target)]:
            point = getattr(request,field)
            point.x, point.y, point.z = map(float,value)
        self.replan = dict(token=token, parent=request.parent_trajectory_id, start_ns=start_ns,
                           target=target, final=final)
        self.replan_pub.publish(request)
        node.event.publish(String(data='REPLAN_REQUESTED'))
        if from_background:
            node.event.publish(String(data='BACKGROUND_REPLAN_REQUESTED'))

    def accepted(self):
        self.background.invalidate()
        self.background_target = None
        self.reset_observation_credit()
        self.flight_observed_start = int(np.count_nonzero(self.node.grid.observed))
        self.segment_start_distance = float(np.linalg.norm(self.position()-self.goal))
        self.phase = 'EXECUTING'
        self.report('NAVIGATING' if self.local_final else 'EXPLORING')

    def failed_segment(self, reason):
        self.background.invalidate()
        self.background_target = None
        self.reset_observation_credit()
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
            self.focused_yaw = None
            self.focused_checked = False
            self.report('OBSERVING', reason)

    def planner_cb(self, msg):
        if self.replan is not None and stamp_key(msg.goal_stamp) == self.replan['token']:
            if msg.state not in ('WAIT_MAP_OR_ODOMETRY', 'SAFE_SEED_FALLBACK', 'TRAJECTORY_PUBLISHED'):
                self.replan_failed(msg.state)
            return
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
        self.replan = None
        if reason in ('GOAL_REPLACED', 'CANCELLED', 'MAP_SESSION_CHANGED'):
            self.finish('CANCELLED', reason)
        elif self.goal is None:
            if self.autonomous.enabled and reason not in ('GOAL_REACHED', 'LOCAL_GOAL_REACHED'):
                self.finish('STOPPED', reason)
            return
        elif reason in ('GOAL_REACHED', 'LOCAL_GOAL_REACHED'):
            self.arrival_view = self.flight_observation_credit(time.monotonic())
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
                if np.linalg.norm(self.position()-self.goal) >= self.segment_start_distance-0.2:
                    self.stagnant_segments += 1
                else:
                    self.stagnant_segments = 0
                if self.stagnant_segments >= 2:
                    self.node.event.publish(String(data='STAGNATION_SCAN_REQUESTED'))
                    self.scan_remaining = 3
                    self.stagnant_segments = 0
            self.phase = 'OBSERVE'
            self.observe_since = None
            self.scan_index = 0
            self.focused_yaw = None
            self.focused_checked = False
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

    def reset_observation_credit(self):
        self.flight_view_since = self.flight_view_yaw = self.flight_view_last = None
        self.flight_view_version = None
        self.flight_observed_start = None
        self.arrival_view = None

    def track_flight_observation(self, now):
        if not self.reuse_observation or not self.explore or self.node.map.static_map:
            return
        q = self.node.odom.pose.pose.orientation
        quaternion = np.array([q.x, q.y, q.z, q.w])
        if not np.all(np.isfinite(quaternion)) or np.linalg.norm(quaternion) < 1e-6:
            self.flight_view_since = None
            return
        x,y,z,w = quaternion/np.linalg.norm(quaternion)
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        pitch = math.asin(float(np.clip(2*(w*y-z*x), -1., 1.)))
        roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        if abs(pitch) > .06 or abs(roll) > .06:
            self.flight_view_since = None
            return
        if (self.flight_view_since is None or self.flight_view_last is None or
                now-self.flight_view_last > .2 or
                abs(math.atan2(math.sin(yaw-self.flight_view_yaw),
                               math.cos(yaw-self.flight_view_yaw))) > .1):
            self.flight_view_since = now
            self.flight_view_yaw = yaw
            self.flight_view_version = self.node.map.version
        self.flight_view_last = now

    def flight_observation_credit(self, now):
        if (not self.reuse_observation or self.flight_view_since is None or
                self.flight_view_last is None or now-self.flight_view_last > .2 or
                now-self.flight_view_since < self.settings.observation_time or
                self.flight_observed_start is None or
                self.node.map.version <= self.flight_view_version):
            return None
        gain = (int(np.count_nonzero(self.node.grid.observed))-
                self.flight_observed_start)*self.node.grid.resolution**3
        return self.flight_view_yaw if gain >= self.settings.minimum_map_gain else None

    def observation_command(self, position, now, require_observation=True, direction=None, reuse=False):
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
        manual_direction = direction is not None
        if direction is None:
            direction = self.goal-position
        desired = math.atan2(direction[1], direction[0])
        if (self.focused_observation and not manual_direction and
                self.scan_index == 0 and self.scan_remaining == 0):
            if not self.focused_checked:
                self.focused_checked = True
                self.focused_yaw = focused_scan_yaw(self.node.grid, position,
                                                   self.goal, self.node.radius, yaw)
                if self.focused_yaw is not None:
                    self.node.event.publish(String(data=f'FOCUSED_OBSERVATION:{self.focused_yaw:.3f}'))
            if self.focused_yaw is not None:
                desired = self.focused_yaw
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
        if (reuse and self.arrival_view is not None and self.flight_view_last is not None and
                0 <= now-self.flight_view_last <= .2 and self.scan_index == 0 and
                self.scan_remaining == 0 and
                abs(math.atan2(math.sin(desired-self.arrival_view),
                               math.cos(desired-self.arrival_view))) <= .1):
            self.arrival_view = None
            self.observe_since = now-self.settings.observation_time
            self.node.event.publish(String(data='OBSERVATION_REUSED'))
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
                    segment_free(node.grid, position, self.goal, node.radius+node.grid.resolution/2)):
                result = (self.goal.copy(), True)
        command, observed = self.observation_command(position, now, require_observation=result is None, reuse=True)
        if self.goal is None:
            return Twist()
        # Deadline is checked even during a scan; inability to orient or find a
        # frontier must not keep the task alive indefinitely.
        if self.no_candidate_at is not None and now-self.no_candidate_at > self.settings.blocked_timeout:
            self.finish('BLOCKED', self.no_candidate_reason)
            return Twist()
        if not observed or now-self.last_selection < 0.5:
            return command
        if self.scan_remaining:
            self.scan_remaining -= 1
            self.scan_index += 1
            self.observe_since = None
            return Twist()
        self.last_selection = now
        diagnostics = {}
        if result is None:
            result = choose_subgoal(node.grid, position, self.goal, node.radius, self.settings,
                self.visited, self.rejected, self.explore and not node.map.static_map, self.best_distance,
                continuous=self.continuous, diagnostics=diagnostics)
        if result is None:
            if not self.explore or node.map.static_map:
                self.finish('BLOCKED', 'NO_KNOWN_PATH')
            else:
                if self.no_candidate_at is None:
                    self.no_candidate_at = now
                self.no_candidate_reason = diagnostics.get('reason', 'NO_REACHABLE_FRONTIER')
                if self.scan_index == 0 and self.focused_yaw is not None:
                    # A focused view is an extra first attempt, not a
                    # replacement for any direction in the fallback sweep.
                    self.focused_yaw = None
                    self.node.event.publish(String(data='FOCUSED_OBSERVATION_FALLBACK'))
                else:
                    self.scan_index += 1
                self.observe_since = None
                self.report('OBSERVING', self.no_candidate_reason)
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
