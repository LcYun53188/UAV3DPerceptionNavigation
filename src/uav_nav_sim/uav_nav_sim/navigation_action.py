"""Cancellable EGO navigation, owned and ticked by the velocity executor.

Callbacks share its single executor thread. The Reentrant group only permits
an execute coroutine to await its result while odometry and ticks continue.
No callback in this adapter publishes motion commands.
"""
import math
import time

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from uav_nav_interfaces.action import NavigateToPose3D

from .navigation import stamp_key
from .control_reservation import ControlReservation


def map_session(node):
    return f'{node.session[0]}:{node.session[1]}' if node.session else ''


def request_error(request):
    p = request.goal.pose.position
    if request.goal.header.frame_id != 'map' or not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
        return 'INVALID_GOAL'
    # GoalManager currently reaches the final goal inside 0.15 m.
    bounds = ((request.position_tolerance_m, .15, 1.),
              (request.stopped_speed_mps, .01, .2),
              (request.stable_duration_s, .5, 10.),
              (request.timeout_s, 1., 600.))
    if any(not math.isfinite(v) or not lo <= v <= hi for v, lo, hi in bounds):
        return 'INVALID_LIMITS'
    session = request.control_session
    if not any(session.session_id.uuid) or session.generation == 0 or not session.owner.strip():
        return 'INVALID_CONTROL_SESSION'
    if not request.map_session:
        return 'INVALID_MAP_SESSION'
    return ''


class NavigationAction:
    cleanup_timeout = 5.

    def __init__(self, node):
        self.node = node
        self.reserved = False
        self.fault_latched = False
        self.active_goal = None
        self.done = None
        self.executions = {}
        self.terminal = None
        self.stable_since = None
        self.last_stamp = None
        self.stamp_advanced = 0.
        self.sequence = 0
        self.internal_stop = False
        self.parent = ControlReservation(node, self)
        self.server = ActionServer(
            node, NavigateToPose3D, '/uav/algorithm/navigate', self.execute,
            goal_callback=self.accept, cancel_callback=self.cancel,
            handle_accepted_callback=self.accepted,
            callback_group=ReentrantCallbackGroup())

    @property
    def busy(self):
        return self.reserved or self.fault_latched or self.parent.session is not None

    def accept(self, request):
        reason = request_error(request)
        nav = self.node.navigation
        if not reason and self.fault_latched:
            reason = 'STOP_UNCONFIRMED_RESTART_REQUIRED'
        if not reason and (self.reserved or nav.goal is not None or nav.autonomous.enabled):
            reason = 'BUSY'
        if not reason and not self.parent.permits(request.control_session):
            reason = 'CONTROL_SESSION_NOT_AUTHORIZED'
        if not reason and (not self.node.ready() or request.map_session != map_session(self.node)):
            reason = 'MAP_OR_ODOMETRY_NOT_READY'
        if reason:
            self.node.get_logger().warning('Navigation Action rejected: '+reason)
            return GoalResponse.REJECT
        # Reserve in the goal callback, before acceptance is sent over DDS.
        self.reserved = True
        return GoalResponse.ACCEPT

    def accepted(self, handle):
        self.active_goal = handle
        self.dispatched = False
        self.done = Future()
        self.executions[bytes(handle.goal_id.uuid)] = self.done
        self.terminal = None
        self.stable_since = None
        self.last_stamp = None
        self.started = self.stamp_advanced = time.monotonic()
        self.last_feedback = float('-inf')
        self.sequence = 0
        # Acceptance and execute are separate callbacks; recheck the map before
        # dispatch, and permit cancel-before-dispatch without starting motion.
        handle.execute()

    async def execute(self, handle):
        key = bytes(handle.goal_id.uuid)
        completion = self.executions[key]
        try:
            # A late execute callback must not dispatch a retired goal or touch
            # the bookkeeping of its successor.
            if handle is self.active_goal:
                self.dispatched = True
                if self.terminal is None:
                    if handle.is_cancel_requested:
                        self.finish('CANCELED', 'CANCEL_REQUESTED')
                    elif (not self.node.ready() or
                          not self.parent.permits(handle.request.control_session) or
                          handle.request.map_session != map_session(self.node)):
                        self.finish('ABORTED', 'MAP_OR_ODOMETRY_NOT_READY')
                    else:
                        self.node.navigation.goal_cb(handle.request.goal, controlled=True)
            return await completion
        finally:
            self.executions.pop(key, None)

    def cancel(self, handle):
        if handle is not self.active_goal or self.terminal is not None:
            return CancelResponse.REJECT
        self.finish('CANCELED', 'CANCEL_REQUESTED')
        return CancelResponse.ACCEPT

    def on_stop(self, reason):
        if (self.active_goal is not None and not self.internal_stop and
                reason not in ('GOAL_REPLACED', 'GOAL_REACHED', 'LOCAL_GOAL_REACHED')):
            self.finish('ABORTED', reason)

    def finish(self, code, reason):
        if self.active_goal is None:
            return
        if self.terminal is not None:
            # Health faults during braking invalidate provisional success or
            # cancellation without restarting the fixed cleanup deadline.
            if code == 'ABORTED' and self.terminal[0] != 'ABORTED':
                self.terminal = (code, reason)
            return
        self.terminal = (code, reason)
        self.cleanup_started = time.monotonic()
        self.stable_since = None
        self.internal_stop = True
        try:
            # Invalidates all local goal/replan tokens before cancel is ACKed.
            self.node.stop('CANCELLED')
            self.node.navigation.report('STOPPING', reason)
        finally:
            self.internal_stop = False

    def stationary(self, now):
        odom = self.node.odom
        if odom is None or not 0 <= now-self.node.odom_wall <= .5:
            self.stable_since = None
            return False
        stamp = stamp_key(odom.header.stamp)
        ros_age = (self.node.get_clock().now().nanoseconds-stamp)/1e9
        v, w = odom.twist.twist.linear, odom.twist.twist.angular
        p = odom.pose.pose.position
        values = (v.x, v.y, v.z, w.x, w.y, w.z, p.x, p.y, p.z)
        valid = (all(math.isfinite(x) for x in values) and 0 <= ros_age <= .5 and
                 now-self.stamp_advanced < .5 and
                 math.hypot(v.x, v.y, v.z) <= self.active_goal.request.stopped_speed_mps and
                 math.hypot(w.x, w.y, w.z) <= .1 and
                 self.node.curve is None and self.node.navigation.goal is None)
        if not valid:
            self.stable_since = None
            return False
        if self.stable_since is None:
            self.stable_since = now
        return now-self.stable_since >= self.active_goal.request.stable_duration_s

    def distance(self):
        if self.node.odom is None:
            return float('inf')
        p = self.node.odom.pose.pose.position
        g = self.active_goal.request.goal.pose.position
        return math.hypot(p.x-g.x, p.y-g.y, p.z-g.z)

    def tick(self):
        self.parent.tick()
        if self.active_goal is None or (not self.dispatched and self.terminal is None):
            return
        now = time.monotonic()
        nav = self.node.navigation
        odom = self.node.odom
        if odom is not None:
            stamp = stamp_key(odom.header.stamp)
            if self.last_stamp is None or stamp > self.last_stamp:
                self.stamp_advanced = now
            elif stamp < self.last_stamp:
                self.stable_since = None
                self.finish('ABORTED', 'ODOMETRY_CLOCK_RESET')
            self.last_stamp = stamp
        if self.active_goal.request.map_session != map_session(self.node):
            self.finish('ABORTED', 'MAP_SESSION_CHANGED')
        elif now-self.stamp_advanced >= .5:
            self.finish('ABORTED', 'ODOMETRY_CLOCK_STALLED')
        elif not self.node.ready():
            self.finish('ABORTED', 'STALE_MAP_OR_ODOMETRY')
        if self.terminal is None:
            if now-self.started >= self.active_goal.request.timeout_s:
                self.finish('ABORTED', 'NAVIGATION_TIMEOUT')
            elif nav.goal is None:
                if nav.state == 'REACHED' and self.distance() <= self.active_goal.request.position_tolerance_m:
                    self.finish('SUCCEEDED', 'GOAL_REACHED')
                else:
                    self.finish('ABORTED', nav.reason or 'NAVIGATION_'+nav.state)
        if self.terminal is not None:
            stopped = self.stationary(now)
            if stopped or now-self.cleanup_started >= self.cleanup_timeout+self.active_goal.request.stable_duration_s:
                code, reason = self.terminal
                # Drift out of the acceptance sphere during braking is failure.
                if code == 'SUCCEEDED' and self.distance() > self.active_goal.request.position_tolerance_m:
                    code, reason = 'ABORTED', 'GOAL_DRIFT_DURING_STOP'
                if not stopped:
                    self.fault_latched = True
                    code, reason = 'ABORTED', reason+':STOP_UNCONFIRMED'
                result = NavigateToPose3D.Result(
                    result_code=code, reason=reason, cleanup_confirmed=stopped, mock=False)
                handle = self.active_goal
                if code == 'SUCCEEDED':
                    handle.succeed()
                elif code == 'CANCELED':
                    handle.canceled()
                else:
                    handle.abort()
                nav.report({'SUCCEEDED': 'REACHED', 'CANCELED': 'CANCELLED'}.get(
                    code, 'STOPPED'), reason)
                self.active_goal = None
                self.reserved = False
                self.done.set_result(result)
                return
        if now-self.last_feedback >= .1:
            self.last_feedback = now
            self.sequence += 1
            feedback = NavigateToPose3D.Feedback()
            status = feedback.status
            status.header.stamp = self.node.get_clock().now().to_msg()
            status.header.frame_id = 'map'
            status.mission_uuid = self.active_goal.goal_id
            status.control_session = self.active_goal.request.control_session
            status.coordinator_instance = status.control_session.owner
            status.event_sequence = self.sequence
            status.map_session = self.active_goal.request.map_session
            status.phase = 'STOPPING' if self.terminal else nav.phase
            status.reason = self.terminal[1] if self.terminal else ''
            status.total_remaining_s = max(0., self.active_goal.request.timeout_s-(now-self.started))
            feedback.current_pose.header = status.header
            if self.node.odom is not None:
                feedback.current_pose.pose = self.node.odom.pose.pose
            feedback.remaining_distance_m = self.distance()
            feedback.local_stage = nav.phase
            feedback.segment_count = getattr(nav, 'segments', 0)
            self.active_goal.publish_feedback(feedback)
