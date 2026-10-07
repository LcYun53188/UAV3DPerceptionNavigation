"""S1 ROS protocol fixture; publishes only /uav/mock/*, never vehicle controls."""
import json
import math
import threading
import time
import uuid

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from unique_identifier_msgs.msg import UUID
from uav_nav_interfaces.action import ExecuteMission, NavigateToPose3D
from uav_nav_interfaces.msg import ControlSession, ControlStatus, TaskStatus
from uav_nav_interfaces.srv import PauseMission, ResumeMission

from .protocol import Protocol


def uuid_text(msg):
    return str(uuid.UUID(bytes=bytes(msg.uuid)))


def uuid_msg(value):
    return UUID(uuid=list(uuid.UUID(value).bytes)) if value else UUID()


class MockServer(Node):
    def __init__(self):
        super().__init__('mission_protocol_mock')
        self.lock = threading.RLock()
        self.group = ReentrantCallbackGroup()
        self.protocol = Protocol()
        self.active_goal = None
        self.reserved = False
        self.options = {}
        self.elapsed = 0.
        self.last_tick = time.monotonic()
        self.stop_started = None
        self.status_pub = self.create_publisher(TaskStatus, '/uav/mock/task_status', 10)
        self.control_pub = self.create_publisher(ControlStatus, '/uav/mock/control_status', 10)
        self.servers = []
        for kind, name in ((ExecuteMission, 'execute_mission'), (NavigateToPose3D, 'navigate')):
            self.servers.append(ActionServer(
                self, kind, f'/uav/mock/{name}', execute_callback=self.execute,
                goal_callback=self.goal, handle_accepted_callback=self.accept,
                cancel_callback=self.cancel, callback_group=self.group))
        self.create_service(PauseMission, '/uav/mock/pause', self.pause, callback_group=self.group)
        self.create_service(ResumeMission, '/uav/mock/resume', self.resume, callback_group=self.group)
        self.create_timer(.02, self.drive, callback_group=self.group,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('MOCK ONLY: no PX4, TF, planner or vehicle control I/O')

    def validated_options(self, request):
        if not math.isfinite(request.timeout_s) or request.timeout_s <= self.protocol.cleanup_s:
            raise ValueError('Invalid timeout')
        if isinstance(request, ExecuteMission.Goal):
            if request.backend != 'MOCK' or request.mission_type != 'NAVIGATE':
                raise ValueError('Only MOCK NAVIGATE is supported')
            options = json.loads(request.parameters_json or '{}')
            if not isinstance(options, dict) or set(options) - {'work_s', 'handoff_failure', 'progress_stall'}:
                raise ValueError('Invalid mock options')
        else:
            pose = request.goal.pose
            numbers = [pose.position.x, pose.position.y, pose.position.z,
                       pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            if request.goal.header.frame_id != 'map' or not all(math.isfinite(x) for x in numbers):
                raise ValueError('Invalid pose')
            norm = sum(x*x for x in numbers[3:])
            if abs(norm - 1.) > .001:
                raise ValueError('Invalid quaternion')
            for value in (request.position_tolerance_m, request.stopped_speed_mps, request.stable_duration_s):
                if not math.isfinite(value) or value <= 0:
                    raise ValueError('Invalid navigation thresholds')
            if any(request.control_session.session_id.uuid) or request.control_session.generation or request.control_session.owner:
                raise ValueError('Standalone mock does not accept external control authorization')
            options = {'work_s': .3}
        work = options.get('work_s', 2.)
        if isinstance(work, bool) or not isinstance(work, (int, float)) or not math.isfinite(work) or work <= 0:
            raise ValueError('Invalid mock duration')
        for key in ('handoff_failure', 'progress_stall'):
            if key in options and not isinstance(options[key], bool):
                raise ValueError('Invalid mock flag')
        return dict(work_s=float(work), handoff_failure=options.get('handoff_failure', False),
                    progress_stall=options.get('progress_stall', False))

    def goal(self, request):
        with self.lock:
            if self.active_goal is not None or self.reserved or self.protocol.owner in ('PILOT', 'FAILSAFE'):
                return GoalResponse.REJECT
            try:
                self.validated_options(request)
            except (ValueError, TypeError):
                return GoalResponse.REJECT
            self.reserved = True
            return GoalResponse.ACCEPT

    def accept(self, handle):
        with self.lock:
            now = time.monotonic()
            self.options = self.validated_options(handle.request)
            decision = self.protocol.begin(uuid_text(handle.goal_id), now, handle.request.timeout_s,
                                           authorized=True, ready=True, map_session='mock-map',
                                           definition=str(handle.request))
            self.reserved = False
            if not decision.accepted:
                # Admission is serialized and reserves the sole task slot.
                raise RuntimeError(f'Mock admission invariant failed: {decision.reason}')
            self.active_goal = handle
            self.elapsed = 0.
            self.last_tick = now
            self.stop_started = None
            handle.execute()

    def cancel(self, handle):
        with self.lock:
            if uuid_text(handle.goal_id) != self.protocol.mission:
                return CancelResponse.REJECT
            decision = self.protocol.cancel(self.protocol.context, time.monotonic())
            return CancelResponse.ACCEPT if decision.accepted else CancelResponse.REJECT

    def service(self, verb, request, response):
        with self.lock:
            decision = self.protocol.command(
                verb, uuid_text(request.mission_uuid), request.coordinator_instance,
                uuid_text(request.request_id), time.monotonic(), ready=True, map_session='mock-map')
            response.accepted, response.reason, response.phase = (
                decision.accepted, decision.reason, decision.phase)
            return response

    def pause(self, request, response):
        return self.service('pause', request, response)

    def resume(self, request, response):
        return self.service('resume', request, response)

    def status(self, now):
        p = self.protocol
        session = ControlSession(session_id=uuid_msg(p.session), generation=p.generation, owner=p.owner)
        status = TaskStatus(mission_uuid=uuid_msg(p.mission), coordinator_instance=p.instance,
                            event_sequence=p.sequence, phase=p.phase, result_code=p.result_code,
                            reason='MOCK_ONLY:' + p.reason, control_session=session,
                            map_session=p.map_session, child_uuid=uuid_msg(p.child),
                            has_child=bool(p.child), total_remaining_s=max(0., p.deadline-now), mock=True)
        status.header.stamp = self.get_clock().now().to_msg()
        return status

    def drive(self):
        with self.lock:
            now = time.monotonic()
            dt = now - self.last_tick
            self.last_tick = now
            p = self.protocol
            p.tick(now)
            if p.phase in ('RUNNING', 'RESUMING'):
                if not self.options.get('progress_stall'):
                    p.progress(p.context, now, healthy=True)
                if p.phase == 'RESUMING':
                    p.child_started(p.context, now)
                if p.phase == 'RUNNING':
                    self.elapsed += dt
                    if self.elapsed >= self.options.get('work_s', math.inf):
                        p.child_finished(p.context, now, final_reached=True, stable=True)
            if p.phase in ('PAUSING', 'STOPPING'):
                if self.stop_started is None:
                    self.stop_started = now
                if not self.options.get('handoff_failure'):
                    if not p.hold_ack_pending and now - self.stop_started >= .1:
                        p.stopped(p.context, now, sample_time=now, stable=True)
                    if p.hold_ack_pending and now - self.stop_started >= .2:
                        p.hold_ack(p.context, now, reference_time=now, healthy=True)
            else:
                self.stop_started = None
            if p.owner == 'HOLD_CONTROLLER' and not p.hold_ack_pending:
                p.renew_hold(p.context, now, reference_time=now, healthy=True)
            status = self.status(now)
            self.status_pub.publish(status)
            control = ControlStatus(coordinator_instance=p.instance, session=status.control_session,
                                    lease_remaining_s=max(0., p.lease_deadline-now),
                                    final_hold_remaining_s=max(0., p.hold_deadline-now),
                                    actual_flight_mode='UNKNOWN', reason='MOCK_ONLY:' + p.reason, mock=True)
            control.header = status.header
            control.allowed_operations = (['MOCK_NAVIGATE'] if p.owner == 'TASK' else
                                          ['MOCK_HOLD'] if p.owner == 'HOLD_CONTROLLER' else [])
            self.control_pub.publish(control)
            if self.active_goal is not None and p.phase not in p.TERMINAL:
                if isinstance(self.active_goal.request, ExecuteMission.Goal):
                    feedback = ExecuteMission.Feedback(status=status, tree_node='MOCK_PROTOCOL', fault=p.reason)
                else:
                    feedback = NavigateToPose3D.Feedback(status=status, remaining_distance_m=-1.,
                                                         local_stage='MOCK_PROTOCOL', segment_count=0)
                    feedback.current_pose.header.frame_id = 'mock_unknown'
                self.active_goal.publish_feedback(feedback)

    def execute(self, handle):
        while rclpy.ok():
            with self.lock:
                p = self.protocol
                if p.phase in p.TERMINAL:
                    kind = ExecuteMission if isinstance(handle.request, ExecuteMission.Goal) else NavigateToPose3D
                    result = kind.Result(result_code=p.result_code, reason='MOCK_ONLY:' + p.reason,
                                         cleanup_confirmed=p.cleanup_confirmed, mock=True)
                    if p.phase == 'CANCELED':
                        handle.canceled()
                    elif p.phase == 'SUCCEEDED':
                        handle.succeed()
                    else:
                        handle.abort()
                    self.active_goal = None
                    return result
            time.sleep(.01)
        raise RuntimeError('Mock shut down while goal active')


def main(args=None):
    rclpy.init(args=args)
    node = MockServer()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
