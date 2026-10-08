"""Actual C++ BT/MissionServer + Python executor adapter over DDS.

Navigation and odometry are synthetic here. Gazebo evidence is collected by the
separate mission regression script; these tests prove transport/lifecycle races.
"""
import itertools
import json
import os
from pathlib import Path
import signal
import sys
import subprocess
import time
from types import SimpleNamespace as NS
import uuid

import pytest
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from uav_nav_interfaces.action import ExecuteMission, NavigateToPose3D
from uav_nav_interfaces.msg import MissionProgress
from uav_nav_interfaces.srv import PauseMission, ResumeMission, ManageControlSession
sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'uav_nav_sim'))
from uav_nav_sim.navigation_action import NavigationAction

_domains = itertools.count(110)


class ContractAction(NavigationAction):
    async def execute(self, handle):
        result = await super().execute(handle)
        if result.result_code == 'CANCELED' and self.node.corrupt_cancel:
            result.result_code = 'SUCCEEDED'
        if self.node.mock_result:
            result.mock = True
        return result


class Backend(Node):
    def __init__(self, context):
        super().__init__('synthetic_navigation_backend', context=context)
        self.session = ('contract-map', 1)
        self.curve = None
        self.odom = Odometry()
        self.odom.pose.pose.orientation.w = 1.
        self.odom_wall = time.monotonic()
        self.delay = .4
        self.health_ready = True
        self.odom_mode = 'live'
        self.corrupt_cancel = self.mock_result = False
        self.dispatches = []
        self.navigation = NS(goal=None, autonomous=NS(enabled=False), state='IDLE',
                             phase='IDLE', reason='', segments=0, token=0)
        self.navigation.goal_cb = self.dispatch
        self.navigation.report = lambda state, reason: vars(self.navigation).update(state=state, reason=reason)
        self.navigation_action = ContractAction(self)
        self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def ready(self):
        return self.health_ready

    def dispatch(self, pose, controlled=False):
        assert controlled
        self.dispatches.append(pose)
        self.navigation.goal = pose
        self.navigation.phase = 'EXECUTING'
        self.navigation.state = 'NAVIGATING'
        self.navigation.token = len(self.dispatches)
        self.dispatched_at = time.monotonic()

    def stop(self, reason):
        self.navigation.goal = None
        self.navigation.token = 0
        self.navigation.phase = 'IDLE'
        self.odom.twist.twist.linear.x = 0.
        self.navigation_action.on_stop(reason)

    def tick(self):
        if self.odom_mode != 'frozen':
            self.odom.header.stamp = self.get_clock().now().to_msg()
        if self.odom_mode != 'missing':
            self.odom_wall = time.monotonic()
        if self.navigation.goal is not None:
            self.odom.twist.twist.linear.x = .4
            if time.monotonic()-self.dispatched_at >= self.delay:
                self.odom.pose.pose = self.navigation.goal.pose
                self.odom.twist.twist.linear.x = 0.
                self.navigation.goal = None
                self.navigation.state = 'REACHED'
        self.navigation_action.tick()


@pytest.fixture
def system(tmp_path):
    domain = next(_domains)
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = Backend(context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    binary = Path(get_package_prefix('uav_bt'))/'lib/uav_bt/algorithm_mission_server'
    assert binary.is_file(), 'Build uav_bt first'
    checkpoint = tmp_path/'checkpoint.json'
    log = (tmp_path/'server.log').open('w')
    process = subprocess.Popen([str(binary), '--ros-args', '-p', f'checkpoint_file:={checkpoint}',
                                '-p', 'pause_timeout_s:=1.5'],
                               env={**os.environ, 'ROS_DOMAIN_ID': str(domain)}, stdout=log, stderr=log)
    client = ActionClient(node, ExecuteMission, '/uav/algorithm/execute_mission')
    pause = node.create_client(PauseMission, '/uav/algorithm/pause_mission')
    resume = node.create_client(ResumeMission, '/uav/algorithm/resume_mission')

    def until(predicate, timeout=5.):
        end = time.monotonic()+timeout
        while not predicate() and time.monotonic() < end:
            executor.spin_once(timeout_sec=.01)
        assert predicate(), (tmp_path/'server.log').read_text()

    until(client.server_is_ready)
    until(lambda: pause.service_is_ready() and resume.service_is_ready())
    identity = node.create_client(GetParameters, '/algorithm_mission_server/get_parameters')
    until(identity.service_is_ready)
    identity_future = identity.call_async(GetParameters.Request(names=['coordinator_instance']))
    until(identity_future.done)
    instance = identity_future.result().values[0].string_value
    feedback = []

    def send(**changes):
        goal = ExecuteMission.Goal(mission_type='WAYPOINTS', backend='ALGORITHM', timeout_s=30.,
            parameters_json=json.dumps({'coordinator_instance': instance, 'map_session': 'contract-map:1',
                                        'waypoints': [[1., 0., 0.], [2., 0., 0.]],
                                        'navigation_timeout_s': 10.}))
        for key, value in changes.items():
            setattr(goal, key, value)
        future = client.send_goal_async(goal, feedback_callback=lambda f: feedback.append(f.feedback))
        until(future.done)
        return future.result()

    def command(handle, pause_request=True, request_id=None, **changes):
        service = pause if pause_request else resume
        request = (PauseMission if pause_request else ResumeMission).Request()
        request.mission_uuid = handle.goal_id
        request.coordinator_instance = feedback[-1].status.coordinator_instance
        request.request_id.uuid = list(request_id or uuid.uuid4().bytes)
        for key, value in changes.items():
            setattr(request, key, value)
        future = service.call_async(request)
        until(future.done)
        return future.result()

    yield node, until, send, command, feedback, checkpoint, process
    process.send_signal(signal.SIGTERM)
    end = time.monotonic()+7.
    while process.poll() is None and time.monotonic() < end:
        executor.spin_once(timeout_sec=.01)
    if process.poll() is None:
        process.kill()
    process.wait(timeout=2.)
    log.close()
    client.destroy()
    node.navigation_action.server.destroy()
    executor.remove_node(node)
    node.destroy_node()
    executor.shutdown()
    context.shutdown()


def test_two_waypoints_complete_once_and_release_parent(system):
    node, until, send, _, feedback, checkpoint, _ = system
    root = send()
    assert root.accepted
    result = root.get_result_async()
    until(result.done, 7.)
    value = result.result()
    assert value.status == GoalStatus.STATUS_SUCCEEDED
    assert value.result.result_code == 'SUCCEEDED' and value.result.cleanup_confirmed
    assert not value.result.mock
    assert len(node.dispatches) == 2
    assert node.navigation_action.parent.session is None
    saved = json.loads(checkpoint.read_text())
    assert saved['waypoint_index'] == 2 and saved['phase'] == 'SUCCEEDED'
    assert len(saved['definition_sha256']) == 64 and not saved['restart_resume_supported']
    assert all(f.status.mission_uuid == root.goal_id for f in feedback)


def test_pause_retains_root_and_owner_resume_gets_new_child(system):
    node, until, send, command, feedback, checkpoint, _ = system
    node.delay = 3.
    root = send()
    result = root.get_result_async()
    until(lambda: feedback and feedback[-1].status.has_child)
    first_child = bytes(feedback[-1].status.child_uuid.uuid)
    request_id = uuid.uuid4().bytes
    assert not command(root, coordinator_instance='old-instance').accepted
    decision = command(root, request_id=request_id)
    assert decision.accepted and decision.phase == 'PAUSING'
    until(lambda: feedback[-1].status.phase == 'PAUSED')
    assert not result.done()
    assert node.navigation_action.busy and node.navigation.goal is None
    paused = json.loads(checkpoint.read_text())
    assert paused['waypoint_index'] == 0 and paused['phase'] == 'PAUSED'
    # A separate client cannot replace a paused parent's child.
    rival = NavigateToPose3D.Goal()
    rival.goal.header.frame_id = 'map'
    rival.position_tolerance_m = .2
    rival.stopped_speed_mps = .05
    rival.stable_duration_s = .6
    rival.timeout_s = 10.
    rival.map_session = 'contract-map:1'
    rival.control_session.session_id.uuid = list(uuid.uuid4().bytes)
    rival.control_session.owner = 'rival'
    rival.control_session.generation = 1
    client = ActionClient(node, NavigateToPose3D, '/uav/algorithm/navigate')
    assert client.wait_for_server(timeout_sec=2.)
    other = client.send_goal_async(rival)
    until(other.done)
    assert not other.result().accepted
    client.destroy()
    repeat = command(root, request_id=request_id)
    assert (repeat.accepted, repeat.phase) == (decision.accepted, decision.phase)
    assert not command(root, pause_request=False, request_id=request_id).accepted
    node.delay = .4
    assert command(root, pause_request=False).accepted
    until(lambda: feedback[-1].status.has_child and
          bytes(feedback[-1].status.child_uuid.uuid) != first_child)
    until(result.done, 7.)
    assert result.result().status == GoalStatus.STATUS_SUCCEEDED
    assert len(node.dispatches) == 3
    assert node.navigation_action.parent.session is None
    final = json.loads(checkpoint.read_text())
    assert final['definition_sha256'] == paused['definition_sha256']
    assert final['remaining_total_s'] < paused['remaining_total_s']
    assert not command(root, pause_request=False).accepted


def test_parent_cancel_wins_during_pausing(system):
    node, until, send, command, feedback, _, _ = system
    node.delay = 3.
    root = send()
    until(lambda: feedback and feedback[-1].status.has_child)
    assert command(root).accepted
    cancel = root.cancel_goal_async()
    until(cancel.done)
    assert cancel.result().goals_canceling
    result = root.get_result_async()
    until(result.done)
    assert result.result().status == GoalStatus.STATUS_CANCELED
    assert result.result().result.cleanup_confirmed
    assert node.navigation_action.parent.session is None
    assert len(node.dispatches) == 1


def test_pause_deadline_aborts_and_releases(system):
    node, until, send, command, feedback, _, _ = system
    node.delay = 3.
    root = send()
    until(lambda: feedback and feedback[-1].status.has_child)
    assert command(root).accepted
    result = root.get_result_async()
    until(result.done, 6.)
    assert result.result().status == GoalStatus.STATUS_ABORTED
    assert result.result().result.reason == 'PAUSE_TIMEOUT'
    assert result.result().result.cleanup_confirmed
    assert node.navigation_action.parent.session is None


def test_runner_stall_expires_backend_lease_and_cannot_replay(system):
    node, until, send, _, feedback, _, process = system
    node.delay = 8.
    root = send()
    until(lambda: feedback and feedback[-1].status.has_child and node.navigation.goal is not None)
    process.send_signal(signal.SIGSTOP)
    until(lambda: node.navigation_action.parent.fault == 'BT_PROGRESS_TIMEOUT', 2.)
    until(lambda: node.navigation_action.active_goal is None, 2.)
    assert node.navigation.goal is None and node.navigation_action.busy
    process.send_signal(signal.SIGCONT)
    result = root.get_result_async()
    until(result.done)
    assert result.result().status == GoalStatus.STATUS_ABORTED
    assert result.result().result.reason == 'BT_PROGRESS_TIMEOUT'
    assert result.result().result.cleanup_confirmed
    assert len(node.dispatches) == 1
    assert node.navigation_action.parent.session is None


@pytest.mark.parametrize('kind', ['backend', 'instance'])
def test_invalid_mission_is_rejected_before_ownership(system, kind):
    node, _, send, _, _, _, _ = system
    root = send(backend='PX4_KNOWN_REGION') if kind == 'backend' else send(parameters_json=json.dumps({
        'map_session': 'contract-map:1', 'coordinator_instance': 'retired-instance',
        'waypoints': [[1., 0., 0.], [2., 0., 0.]]}))
    assert not root.accepted
    assert node.navigation_action.parent.session is None


def test_successor_root_gets_fresh_generation_and_no_old_cancel(system):
    node, until, send, _, _, _, _ = system
    first = send()
    result = first.get_result_async()
    until(result.done, 7.)
    assert result.result().status == GoalStatus.STATUS_SUCCEEDED
    generation = node.navigation_action.parent.seen_owners.copy()
    second = send()
    assert second.accepted
    until(lambda: node.navigation_action.parent.session is not None)
    owner = node.navigation_action.parent.session.owner
    assert node.navigation_action.parent.session.generation > generation[owner]
    stale = first.cancel_goal_async()
    until(stale.done)
    assert not stale.result().goals_canceling
    result = second.get_result_async()
    until(result.done, 7.)
    assert result.result().status == GoalStatus.STATUS_SUCCEEDED
    assert len(node.dispatches) == 4


def test_reservation_rejects_replay_and_retired_generation(system):
    node, until, _, _, _, _, _ = system
    service = node.create_client(ManageControlSession, '/uav/algorithm/control_session')
    until(service.service_is_ready)
    request = ManageControlSession.Request(operation='ACQUIRE', map_session='contract-map:1')
    request.control_session.owner = 'manual-contract'
    request.control_session.session_id.uuid = list(uuid.uuid4().bytes)
    request.control_session.generation = 1
    call = service.call_async(request)
    until(call.done)
    assert call.result().accepted
    reservation = node.navigation_action.parent
    assert not reservation.permits(request.control_session)  # Requires first actual BT tick.
    publisher = node.create_publisher(MissionProgress, '/uav/algorithm/mission_progress', 10)
    until(lambda: publisher.get_subscription_count() > 0)
    progress = MissionProgress(mission_uuid=request.control_session.session_id,
                              coordinator_instance='manual-contract', tick_sequence=1)
    publisher.publish(progress)
    until(lambda: reservation.sequence == 1)
    node.session = ('another-map', 2)
    assert not reservation.permits(request.control_session)
    node.session = ('contract-map', 1)
    deadline = reservation.deadline
    publisher.publish(progress)  # Duplicate cannot extend the lease.
    bad = MissionProgress(mission_uuid=request.control_session.session_id,
                          coordinator_instance='retired-instance', tick_sequence=100)
    publisher.publish(bad)
    until(lambda: reservation.fault == 'BT_PROGRESS_TIMEOUT', 2.)
    assert reservation.sequence == 1 and reservation.deadline == deadline
    progress.tick_sequence = 2
    publisher.publish(progress)
    until(lambda: reservation.stable_since is not None and
          time.monotonic()-reservation.stable_since >= .6)
    request.operation = 'RELEASE'
    call = service.call_async(request)
    until(call.done)
    assert call.result().accepted and call.result().reason == 'BT_PROGRESS_TIMEOUT'
    assert reservation.session is None and not reservation.permits(request.control_session)
    request.operation = 'ACQUIRE'
    call = service.call_async(request)
    until(call.done)
    assert not call.result().accepted and call.result().reason == 'INVALID_OR_RETIRED_SESSION'


def test_pause_before_dispatch_resumes_without_canceling_new_child(system):
    node, until, send, command, feedback, _, _ = system
    root = send()
    until(lambda: feedback and feedback[-1].status.phase == 'RUNNING' and
          not feedback[-1].status.has_child)
    assert command(root).accepted
    until(lambda: feedback[-1].status.phase == 'PAUSED')
    assert command(root, pause_request=False).accepted
    result = root.get_result_async()
    until(result.done, 7.)
    assert result.result().status == GoalStatus.STATUS_SUCCEEDED
    assert len(node.dispatches) == 2


@pytest.mark.parametrize('corruption', ['contradictory', 'mock'])
def test_root_cancel_rejects_untrustworthy_child_result(system, corruption):
    node, until, send, _, feedback, _, _ = system
    node.delay = 3.
    node.corrupt_cancel = corruption == 'contradictory'
    node.mock_result = corruption == 'mock'
    root = send()
    until(lambda: feedback and feedback[-1].status.has_child)
    cancel = root.cancel_goal_async()
    until(cancel.done)
    result = root.get_result_async()
    until(result.done)
    assert result.result().status == GoalStatus.STATUS_ABORTED
    assert result.result().result.result_code == 'ABORTED'
    assert result.result().result.reason == (
        'INVALID_CHILD_RESULT' if corruption == 'contradictory' else 'CHILD_STOP_UNCONFIRMED')
    assert result.result().result.cleanup_confirmed == (corruption == 'contradictory')



def test_server_exit_waits_for_confirmed_stop_and_aborts_root(system):
    node, until, send, _, feedback, _, process = system
    node.delay = 3.
    root = send()
    result = root.get_result_async()
    until(lambda: feedback and feedback[-1].status.has_child)
    process.terminate()
    until(result.done)
    assert result.result().status == GoalStatus.STATUS_ABORTED
    assert result.result().result.reason == 'SERVER_EXIT_REQUESTED'
    assert result.result().result.cleanup_confirmed
    assert node.navigation_action.parent.session is None
    until(lambda: process.poll() is not None)
    assert process.returncode == 0


@pytest.mark.parametrize('phase', ['RUNNING', 'PAUSED'])
@pytest.mark.parametrize('fault,reason', [
    ('map', 'MAP_SESSION_CHANGED'),
    ('health', 'MAP_OR_ODOMETRY_NOT_READY'),
    ('frozen', 'ODOMETRY_OR_CLOCK_FAULT'),
    ('missing', 'ODOMETRY_OR_CLOCK_FAULT'),
    ('rewind', 'CLOCK_RESET'),
])
def test_fault_revokes_parent_and_recovery_requires_new_root(system, phase, fault, reason):
    node, until, send, command, feedback, checkpoint, _ = system
    node.delay = 8.
    root = send()
    result = root.get_result_async()
    until(lambda: feedback and feedback[-1].status.has_child and node.navigation.goal is not None)
    old_session = node.navigation_action.parent.session
    old_child = bytes(feedback[-1].status.child_uuid.uuid)
    if phase == 'PAUSED':
        assert command(root).accepted
        until(lambda: feedback[-1].status.phase == 'PAUSED')
    if fault == 'map':
        node.session = ('contract-map', 2)
    elif fault == 'health':
        node.health_ready = False
    elif fault in ('frozen', 'missing'):
        node.odom_mode = fault
    else:
        node.odom_mode = 'frozen'
        node.odom.header.stamp.sec -= 1
    reservation = node.navigation_action.parent
    until(lambda: reservation.fault == reason, 2.)
    assert not reservation.permits(old_session)
    assert node.navigation.goal is None
    if phase == 'PAUSED':
        # Before the fault status crosses DDS a resume ACK can use the previous
        # fresh status. The backend must still reject authorization atomically.
        decision = command(root, pause_request=False)
        assert not decision.accepted or decision.phase == 'RESUMING'
    # A fault cannot clear itself or dispatch the old child after fresh input returns.
    node.odom_mode, node.health_ready = 'live', True
    until(result.done, 5.)
    value = result.result()
    assert value.status == GoalStatus.STATUS_ABORTED
    assert value.result.reason == reason and value.result.cleanup_confirmed and not value.result.mock
    assert reservation.session is None and not reservation.permits(old_session)
    assert len(node.dispatches) == 1
    # Rebind explicitly to the recovered map; only a fresh root/generation may move.
    node.delay = .4
    definition = json.loads(checkpoint.read_text())['definition']
    definition['map_session'] = f'{node.session[0]}:{node.session[1]}'
    successor = send(parameters_json=json.dumps(definition))
    assert successor.accepted and successor.goal_id != root.goal_id
    next_result = successor.get_result_async()
    until(next_result.done, 7.)
    assert next_result.result().status == GoalStatus.STATUS_SUCCEEDED
    assert len(node.dispatches) == 3
    assert all(bytes(f.status.child_uuid.uuid) != old_child for f in feedback
               if f.status.mission_uuid == successor.goal_id and f.status.has_child)


def test_persistent_odometry_loss_cannot_claim_cleanup_or_accept_successor(system):
    node, until, send, _, feedback, _, _ = system
    node.delay = 20.
    root = send()
    until(lambda: feedback and feedback[-1].status.has_child and node.navigation.goal is not None)
    session = node.navigation_action.parent.session
    node.odom_mode = 'missing'
    result = root.get_result_async()
    until(result.done, 9.)
    value = result.result()
    assert value.status == GoalStatus.STATUS_ABORTED
    assert value.result.reason == 'CHILD_STOP_UNCONFIRMED'
    assert not value.result.cleanup_confirmed and not value.result.mock
    assert node.navigation_action.fault_latched
    assert node.navigation_action.parent.matches(session)
    assert node.navigation.goal is None
    # Returning data must not clear either the backend or MissionServer latch.
    node.odom_mode = 'live'
    assert not send().accepted
    assert not node.navigation_action.parent.permits(session)
    assert len(node.dispatches) == 1
