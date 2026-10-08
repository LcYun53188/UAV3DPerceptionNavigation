"""Real DDS Action contract with synthetic navigation/odometry, not flight proof."""
import itertools
import time
from types import SimpleNamespace as NS

import pytest
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from uav_nav_interfaces.action import NavigateToPose3D
from uav_nav_sim.navigation_action import NavigationAction


_domains = itertools.count(90)


class Harness(Node):
    def __init__(self, context):
        super().__init__('navigation_contract', context=context)
        self.session = ('test-map', 1)
        self.curve = None
        self.odom = Odometry()
        self.odom.pose.pose.orientation.w = 1.
        self.odom_wall = time.monotonic()
        self.healthy = True
        self.refresh = True
        self.speed = 0.
        self.stamp_refresh = True
        self.navigation = NS(goal=None, autonomous=NS(enabled=False), phase='IDLE',
                             state='IDLE', reason='', segments=0, token=0)
        self.navigation.goal_cb = self.dispatch
        self.navigation.report = lambda state, reason: vars(self.navigation).update(
            state=state, reason=reason)
        self.navigation_action = NavigationAction(self)
        self.create_timer(.02, self.tick)

    def ready(self):
        return self.healthy

    def dispatch(self, pose, controlled=False):
        assert controlled
        self.navigation.goal = pose
        self.navigation.state = 'EXECUTING'
        self.navigation.phase = 'PLANNING'
        self.navigation.token += 1

    def stop(self, reason):
        self.curve = None
        self.navigation.goal = None
        self.navigation.token = 0
        self.navigation.phase = 'IDLE'
        self.navigation_action.on_stop(reason)

    def tick(self):
        if self.refresh:
            self.odom_wall = time.monotonic()
            if self.stamp_refresh:
                self.odom.header.stamp = self.get_clock().now().to_msg()
            self.odom.twist.twist.linear.x = self.speed
        self.navigation_action.tick()


@pytest.fixture
def runtime():
    context = Context()
    rclpy.init(context=context, domain_id=next(_domains))
    node = Harness(context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    client = ActionClient(node, NavigateToPose3D, '/uav/algorithm/navigate')
    assert client.wait_for_server(timeout_sec=3.)

    def until(predicate, timeout=3.):
        deadline = time.monotonic()+timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
        assert predicate(), 'Timed out waiting for Action state'

    def send(**changes):
        goal = NavigateToPose3D.Goal()
        goal.goal = PoseStamped()
        goal.goal.header.frame_id = 'map'
        goal.goal.pose.orientation.w = 1.
        goal.position_tolerance_m = .2
        goal.stopped_speed_mps = .05
        goal.stable_duration_s = .5
        goal.timeout_s = 5.
        goal.map_session = 'test-map:1'
        goal.control_session.owner = 'contract'
        goal.control_session.generation = 1
        goal.control_session.session_id.uuid = [1]*16
        for key, value in changes.items():
            setattr(goal, key, value)
        feedback = []
        future = client.send_goal_async(goal, feedback_callback=feedback.append)
        until(future.done)
        handle = future.result()
        if handle.accepted:
            until(lambda: node.navigation_action.dispatched)
        return handle, feedback

    yield node, executor, until, send
    client.destroy()
    node.navigation_action.server.destroy()
    executor.remove_node(node)
    node.destroy_node()
    executor.shutdown()
    context.shutdown()


def test_success_waits_for_real_stop_window_and_feedback(runtime):
    node, _, until, send = runtime
    node.speed = .4
    handle, feedback = send()
    result = handle.get_result_async()
    node.navigation.goal = None
    node.navigation.state = 'REACHED'
    until(lambda: node.navigation_action.terminal is not None)
    assert not result.done()
    node.speed = 0.
    stopped = time.monotonic()
    until(result.done)
    assert time.monotonic()-stopped >= .48
    value = result.result()
    assert value.status == GoalStatus.STATUS_SUCCEEDED
    assert value.result.result_code == 'SUCCEEDED'
    assert node.navigation.state == 'REACHED'
    assert value.result.cleanup_confirmed and not value.result.mock
    assert feedback and feedback[-1].feedback.status.phase == 'STOPPING'
    assert feedback[-1].feedback.status.control_session.owner == 'contract'
    assert [f.feedback.status.event_sequence for f in feedback] == sorted(
        {f.feedback.status.event_sequence for f in feedback})


def test_cancel_ack_invalidates_token_but_waits_for_stop(runtime):
    node, _, until, send = runtime
    node.speed = .4
    handle, _ = send()
    result = handle.get_result_async()
    cancel = handle.cancel_goal_async()
    until(cancel.done)
    assert cancel.result().goals_canceling
    assert node.navigation.goal is None and node.navigation.token == 0
    assert not result.done()
    rejected, _ = send()
    assert not rejected.accepted
    node.speed = 0.
    until(result.done)
    assert result.result().status == GoalStatus.STATUS_CANCELED
    assert result.result().result.cleanup_confirmed
    # A retired UUID cannot cancel or corrupt the next goal.
    newer, _ = send()
    assert newer.accepted
    old_cancel = handle.cancel_goal_async()
    until(old_cancel.done)
    assert not old_cancel.result().goals_canceling
    assert bytes(node.navigation_action.active_goal.goal_id.uuid) == bytes(newer.goal_id.uuid)
    cancel = newer.cancel_goal_async()
    until(cancel.done)
    newer_result = newer.get_result_async()
    until(newer_result.done)


@pytest.mark.parametrize('fault,reason', [
    ('session', 'MAP_SESSION_CHANGED'),
    ('health', 'STALE_MAP_OR_ODOMETRY'),
    ('planner', 'NO_PATH'),
    ('timeout', 'NAVIGATION_TIMEOUT'),
])
def test_faults_abort_only_after_confirmed_cleanup(runtime, fault, reason):
    node, _, until, send = runtime
    handle, _ = send(timeout_s=1.)
    result = handle.get_result_async()
    if fault == 'session':
        node.session = ('another-map', 2)
    elif fault == 'health':
        node.healthy = False
    elif fault == 'planner':
        node.navigation.goal = None
        node.navigation.state = 'BLOCKED'
        node.navigation.reason = 'NO_PATH'
    until(result.done)
    value = result.result()
    assert value.status == GoalStatus.STATUS_ABORTED
    assert value.result.reason == reason
    assert value.result.cleanup_confirmed


def test_frozen_stamp_never_counts_as_stopped(runtime):
    node, _, until, send = runtime
    node.navigation_action.cleanup_timeout = .8
    handle, _ = send()
    result = handle.get_result_async()
    node.stamp_refresh = False
    until(result.done)
    value = result.result()
    assert value.status == GoalStatus.STATUS_ABORTED
    assert value.result.reason == 'ODOMETRY_CLOCK_STALLED:STOP_UNCONFIRMED'
    assert not value.result.cleanup_confirmed
    rejected, _ = send()
    assert not rejected.accepted and node.navigation_action.busy


def test_local_arrival_is_not_final_arrival(runtime):
    node, _, until, send = runtime
    handle, _ = send()
    node.navigation.segments = 1
    node.navigation.phase = 'OBSERVE'
    result = handle.get_result_async()
    until(lambda: node.navigation_action.sequence >= 3)
    assert not result.done()
    cancel = handle.cancel_goal_async()
    until(cancel.done)
    until(result.done)


@pytest.mark.parametrize('change', [
    {'timeout_s': float('nan')}, {'position_tolerance_m': .01},
    {'map_session': 'old-map:1'}, {'stable_duration_s': 0.},
])
def test_bad_contract_rejected_without_taking_control(runtime, change):
    node, _, _, send = runtime
    handle, _ = send(**change)
    assert not handle.accepted
    assert not node.navigation_action.busy
    assert node.navigation.goal is None


def test_arrival_drift_during_braking_is_not_success(runtime):
    node, _, until, send = runtime
    handle, _ = send()
    result = handle.get_result_async()
    node.navigation.goal = None
    node.navigation.state = 'REACHED'
    until(lambda: node.navigation_action.terminal is not None)
    node.odom.pose.pose.position.x = .3
    until(result.done)
    assert result.result().status == GoalStatus.STATUS_ABORTED
    assert result.result().result.reason == 'GOAL_DRIFT_DURING_STOP'
    assert result.result().result.cleanup_confirmed


def test_observation_rotation_does_not_count_as_stopped(runtime):
    node, _, until, send = runtime
    node.odom.twist.twist.angular.z = .4
    handle, feedback = send()
    result = handle.get_result_async()
    canceled = handle.cancel_goal_async()
    until(canceled.done)
    until(lambda: len(feedback) >= 4)
    assert not result.done()
    node.odom.twist.twist.angular.z = 0.
    until(result.done)
    assert result.result().status == GoalStatus.STATUS_CANCELED
    assert result.result().result.cleanup_confirmed


def test_map_change_during_arrival_cleanup_invalidates_success(runtime):
    node, _, until, send = runtime
    handle, _ = send()
    result = handle.get_result_async()
    node.navigation.goal = None
    node.navigation.state = 'REACHED'
    until(lambda: node.navigation_action.terminal is not None)
    node.session = ('replacement-map', 2)
    until(result.done)
    assert result.result().status == GoalStatus.STATUS_ABORTED
    assert result.result().result.reason == 'MAP_SESSION_CHANGED'
    assert result.result().result.cleanup_confirmed
