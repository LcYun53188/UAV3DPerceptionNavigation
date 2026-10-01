from types import SimpleNamespace as NS
from unittest.mock import Mock

import numpy as np
import pytest
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.time import Time
from rclpy.clock import ClockType
from uav_nav_interfaces.msg import PlannerStatus, TimedTrajectory
from uav_nav_sim.navigation import GoalManager
from uav_nav_sim.executor import Executor
from uav_nav_sim.core import Grid


@pytest.fixture
def task(monkeypatch):
    now = [100.]
    monkeypatch.setattr('uav_nav_sim.navigation.time.monotonic', lambda: now[0])
    params = {'explore_unknown': False}
    node = NS(radius=.3, timeout=2., map=NS(static_map=True), odom=Odometry(),
              odom_wall=100., curve=None, ready=lambda: True,
              create_publisher=lambda *a, **k: Mock(), create_subscription=lambda *a, **k: Mock(),
              create_timer=lambda *a, **k: Mock(), create_service=lambda *a, **k: Mock(), get_logger=lambda: Mock(),
              get_clock=lambda: NS(nanoseconds=int(now[0]*1e9), now=lambda: Time(nanoseconds=int(now[0]*1e9), clock_type=ClockType.ROS_TIME)))
    node.declare_parameter=lambda k,v: params.setdefault(k,v)
    node.get_parameter=lambda k: NS(value=params[k])
    node.odom.header.stamp.sec=100
    node.odom.pose.pose.orientation.w=1.
    node.grid=Grid(np.array([-2.,-2.,-2.]),.2,np.full((40,20,20),3.),np.ones((40,20,20),bool))
    manager=GoalManager(node)
    node.navigation=manager
    node.command, node.event = Mock(), Mock()
    node.stop=lambda reason: Executor.stop(node,reason)
    return manager, node, now


def begin(manager, x=3.):
    msg=PoseStamped()
    msg.header.frame_id='map'
    msg.pose.position.x=x
    manager.goal_cb(msg)
    return msg


def advance(node, now, seconds):
    now[0]+=seconds
    node.odom_wall=now[0]
    node.odom.header.stamp.sec=int(now[0])
    node.odom.header.stamp.nanosec=int((now[0]%1)*1e9)


def test_cancel_rejects_delayed_trajectory_and_stays_latched(task):
    manager,node,now=task
    begin(manager)
    manager.tick()
    trajectory=TimedTrajectory(goal_stamp=manager.local_pub.publish.call_args[0][0].header.stamp)
    assert manager.accepts(trajectory)
    Executor.cancel(node,None,NS(success=False))
    assert manager.state=='CANCELLED'
    assert not manager.accepts(trajectory)
    Executor.trajectory_cb(node,trajectory)
    assert node.curve is None
    for _ in range(4):
        advance(node,now,1.)
        manager.tick()
    assert manager.state=='CANCELLED' and manager.goal is None


def test_replacement_rejects_old_results_without_failing_new_task(task):
    manager,node,now=task
    begin(manager)
    manager.tick()
    old=manager.local_pub.publish.call_args[0][0].header.stamp
    begin(manager,2.)
    advance(node,now,.5)
    manager.tick()
    assert not manager.accepts(TimedTrajectory(goal_stamp=old))
    manager.planner_cb(PlannerStatus(goal_stamp=old,state='NO_PATH'))
    assert manager.phase=='PLANNING' and manager.failures==0


def test_local_arrival_keeps_final_goal(task):
    manager,node,now=task
    begin(manager)
    manager.local=np.array([1.,0.,0.])
    manager.local_final=False
    node.odom.pose.pose.position.x=1.
    node.stop('GOAL_REACHED')
    assert manager.phase=='OBSERVE' and np.allclose(manager.goal,[3.,0.,0.])
    assert node.event.publish.call_args[0][0].data=='LOCAL_GOAL_REACHED'
    node.odom.pose.pose.position.x=3.
    manager.tick()
    assert manager.state=='REACHED' and manager.goal is None


def test_static_failure_terminates_without_infinite_retry(task):
    manager,node,now=task
    begin(manager)
    manager.tick()
    stamp=manager.local_pub.publish.call_args[0][0].header.stamp
    manager.planner_cb(PlannerStatus(goal_stamp=stamp,state='NO_PATH'))
    advance(node,now,.5)
    manager.tick()
    assert manager.state=='BLOCKED'
    count=manager.local_pub.publish.call_count
    advance(node,now,1.)
    manager.tick()
    assert manager.local_pub.publish.call_count==count


def test_plan_timeout_invalidates_delayed_success(task):
    manager,node,now=task
    begin(manager)
    manager.tick()
    stamp=manager.local_pub.publish.call_args[0][0].header.stamp
    advance(node,now,9.)
    manager.tick()
    assert not manager.accepts(TimedTrajectory(goal_stamp=stamp))
    assert manager.failures==1


@pytest.mark.parametrize('reason',['STALE_MAP_OR_ODOMETRY','CLOCK_RESET','TRACKING_ERROR','MAP_SESSION_CHANGED'])
def test_health_and_session_stops_never_auto_resume(task,reason):
    manager,node,now=task
    begin(manager)
    manager.tick()
    node.stop(reason)
    count=manager.local_pub.publish.call_count
    advance(node,now,.5)
    manager.tick()
    assert manager.goal is None and manager.local_pub.publish.call_count==count


def test_invalid_goal_does_not_replace_active_task(task):
    manager,node,now=task
    begin(manager)
    msg=PoseStamped()
    msg.header.frame_id='odom'
    manager.goal_cb(msg)
    assert np.allclose(manager.goal,[3.,0.,0.])


def test_no_frontier_timeout_is_checked_during_observation_turn(task):
    manager,node,now=task
    begin(manager)
    manager.explore=True
    node.map.static_map=False
    manager.no_candidate_at=now[0]-16.
    manager.scan_index=1
    command=manager.tick()
    assert manager.state=='BLOCKED' and command.angular.z==0.


def test_max_failures_latch_blocked(task):
    manager,node,now=task
    begin(manager)
    for _ in range(manager.settings.max_failures):
        manager.failed_segment('NO_PATH')
    assert manager.state=='BLOCKED' and manager.goal is None


def test_invalid_map_cancels_task_even_before_a_trajectory(task):
    from uav_nav_interfaces.msg import MapSnapshot
    manager,node,now=task
    node.session=('map',1)
    node.last_id=0
    begin(manager)
    node.curve=None
    Executor.map_cb(node,MapSnapshot(map_id='map',epoch=1,valid=False))
    assert manager.state=='STOPPED' and manager.goal is None


def test_cancel_during_observation_publishes_zero_command(task):
    manager,node,now=task
    begin(manager)
    manager.explore=True
    node.map.static_map=False
    manager.scan_index=1
    assert manager.tick().angular.z != 0.
    node.stop('CANCELLED')
    assert node.command.publish.call_args[0][0].angular.z==0.
    assert manager.tick().angular.z==0.



@pytest.mark.parametrize('height', [-10., 0., 10.])
def test_observation_does_not_tilt_body_to_aim_camera(task, height):
    manager,node,now=task
    begin(manager)
    manager.explore=True
    node.map.static_map=False
    manager.goal[2] = height
    command, _ = manager.observation_command(manager.position(), now[0])
    assert command.angular.x == command.angular.y == 0.
    assert command.linear.x == command.linear.y == command.linear.z == 0.


def test_unsafe_remaining_curve_keeps_valid_new_map_for_replan(task):
    from uav_nav_interfaces.msg import MapSnapshot
    from uav_nav_sim.core import spline
    manager,node,now=task
    begin(manager)
    node.session=('map',1)
    node.map=None
    node.limits=[5.,5.,5.]
    node.curve=spline([[0.,0.,0.]]*3+[[1.,0.,0.]]+[[3.,0.,0.]]*3,3.)
    node.trajectory=TimedTrajectory()
    node.trajectory.start_time.sec=100
    msg=MapSnapshot(map_id='map',epoch=1,version=2,valid=True,resolution=.2)
    msg.origin.x=msg.origin.y=msg.origin.z=-2.
    msg.shape=[40,20,20]
    distance=np.full((40,20,20),3.,dtype=np.float32)
    distance[15,10,10]=-.1
    msg.distance=distance.ravel().tolist()
    msg.observed=np.ones(distance.size,dtype=np.uint8).tolist()
    Executor.map_cb(node,msg)
    assert node.curve is None
    assert node.map is msg and node.grid is not None
    assert manager.phase=='OBSERVE' and manager.failures==1



def test_known_occupied_goal_blocks_without_moving_or_scanning(task):
    manager,node,now=task
    distance=node.grid.distance.copy()
    distance[25,10,10]=-.1
    node.grid=Grid(node.grid.origin.copy(),.2,distance,node.grid.observed.copy())
    begin(manager)
    command=manager.tick()
    assert manager.state=='BLOCKED' and manager.reason=='KNOWN_GOAL_OCCUPIED'
    assert command.angular.x==command.angular.y==command.angular.z==0.
    manager.local_pub.publish.assert_not_called()


def test_stale_map_while_waiting_for_planner_latches_stop(task):
    manager,node,now=task
    begin(manager)
    manager.tick()
    node.ready=lambda:False
    advance(node,now,3.)
    manager.tick()
    assert manager.state=='STOPPED' and manager.goal is None


def test_known_direct_corridor_skips_online_scan_even_when_facing_away(task):
    manager,node,now=task
    manager.explore=True
    node.map.static_map=False
    node.odom.pose.pose.orientation.w=0.
    node.odom.pose.pose.orientation.z=1.
    begin(manager)
    command=manager.tick()
    assert manager.phase=='PLANNING' and manager.local_final
    assert command.angular.z==0.
    assert np.allclose(manager.local,[3.,0.,0.])


@pytest.mark.parametrize('obstacle', ['unknown', 'occupied'])
def test_corridor_gap_requires_observation_even_with_known_free_goal(task, obstacle):
    manager,node,now=task
    manager.explore=True
    node.map.static_map=False
    distance=node.grid.distance.copy()
    observed=node.grid.observed.copy()
    if obstacle == 'unknown':
        observed[17:19,:,:]=False
    else:
        distance[17:19,:,:]=-.1
    node.grid=Grid(node.grid.origin.copy(),.2,distance,observed)
    begin(manager)
    manager.tick()
    assert manager.phase=='OBSERVE'
    assert manager.observe_since==now[0]
    manager.local_pub.publish.assert_not_called()


@pytest.mark.parametrize('orientation', ['invalid', 'tilted'])
def test_known_corridor_does_not_bypass_orientation_checks(task, orientation):
    manager,node,now=task
    manager.explore=True
    node.map.static_map=False
    if orientation == 'invalid':
        node.odom.pose.pose.orientation.w=float('nan')
    else:
        node.odom.pose.pose.orientation.x=np.sin(.2)
        node.odom.pose.pose.orientation.w=np.cos(.2)
    begin(manager)
    manager.tick()
    manager.local_pub.publish.assert_not_called()
    assert manager.state==('STOPPED' if orientation=='invalid' else 'OBSERVING')


def test_rejected_known_goal_does_not_bypass_observation(task):
    manager,node,now=task
    manager.explore=True
    node.map.static_map=False
    begin(manager)
    manager.rejected.append(manager.goal.copy())
    manager.tick()
    assert manager.phase=='OBSERVE'
    manager.local_pub.publish.assert_not_called()


@pytest.mark.parametrize('map_gain', [False, True])
def test_detour_renews_progress_only_after_arrival_with_new_map_volume(task, map_gain):
    manager,node,now=task
    manager.explore=True
    node.map.static_map=False
    old=node.grid
    observed=old.observed.copy()
    observed[30:,:,:]=False
    node.grid=Grid(old.origin.copy(),old.resolution,old.distance.copy(),observed)
    begin(manager)
    manager.best_distance=1.
    manager.local=np.array([1.,0.,0.])
    node.odom.pose.pose.position.x=1.
    advance(node,now,61.)
    if map_gain:
        node.grid=old
    manager.on_stop('LOCAL_GOAL_REACHED')
    manager.tick()
    assert (manager.state=='BLOCKED') == (not map_gain)
    if map_gain:
        assert manager.progress_at==now[0]
        manager.segments=manager.settings.max_segments
        manager.tick()
        assert manager.state=='BLOCKED' and manager.reason=='EXPLORATION_BUDGET'


def test_two_stagnant_segments_scan_sides_before_reusing_forward_candidates(task):
    manager,node,now=task
    manager.explore=True
    node.map.static_map=False
    begin(manager)
    manager.best_distance=1.
    for x in [1.,.9]:
        manager.local=np.array([x,0.,0.])
        node.odom.pose.pose.position.x=x
        manager.on_stop('LOCAL_GOAL_REACHED')
    assert manager.scan_remaining==3
    manager.observation_command=Mock(return_value=(Twist(),True))
    for _ in range(3):
        manager.tick()
        manager.local_pub.publish.assert_not_called()
    manager.tick()
    assert manager.phase=='PLANNING'


def enable_auto(manager, node):
    manager.explore = True
    node.map.static_map = False
    response = manager.autonomous.enable(NS(data=True), NS(success=False, message=''))
    assert response.success
    return manager.autonomous


def test_autonomous_enable_requires_online_ready_map(task):
    manager, node, now = task
    response = manager.autonomous.enable(NS(data=True), NS(success=False, message=''))
    assert not response.success and not manager.autonomous.enabled
    manager.explore = True
    node.map.static_map = False
    node.ready = lambda: False
    response = manager.autonomous.enable(NS(data=True), NS(success=False, message=''))
    assert not response.success and not manager.autonomous.enabled


def test_autonomous_cancel_and_manual_goal_take_over(task):
    manager, node, now = task
    auto = enable_auto(manager, node)
    node.stop('CANCELLED')
    assert not auto.enabled
    manager.tick()
    assert manager.goal is None
    enable_auto(manager, node)
    begin(manager)
    assert not auto.enabled and manager.goal is not None


@pytest.mark.parametrize('reason', ['MAP_INVALID', 'CLOCK_RESET', 'TRACKING_ERROR', 'MAP_SESSION_CHANGED'])
def test_autonomous_health_fault_cannot_restart(task, reason):
    manager, node, now = task
    auto = enable_auto(manager, node)
    node.stop(reason)
    for _ in range(3):
        advance(node, now, .5)
        manager.tick()
    assert not auto.enabled and manager.goal is None


def test_autonomous_scans_dispatches_and_continues_after_arrival(task, monkeypatch):
    manager, node, now = task
    auto = enable_auto(manager, node)
    monkeypatch.setattr(manager, 'observation_command', lambda *a, **k: (Twist(), True))
    monkeypatch.setattr('uav_nav_sim.autonomous.frontier_viewpoint', lambda *a: np.array([2.,0.,0.]))
    for _ in range(4):
        manager.tick()
    assert auto.enabled and auto.phase == 'GOAL'
    assert np.allclose(manager.goal, [2.,0.,0.])
    node.odom.pose.pose.position.x = 2.
    manager.tick()
    assert auto.enabled and auto.phase == 'SCAN' and auto.completed == 1
    assert manager.goal is None
    monkeypatch.setattr('uav_nav_sim.autonomous.frontier_viewpoint', lambda *a: None)
    for _ in range(4):
        manager.tick()
    assert not auto.enabled and auto.state == 'FRONTIERS_EXHAUSTED'


def test_autonomous_budget_cancels_active_goal(task):
    manager, node, now = task
    auto = enable_auto(manager, node)
    auto.progress_at = now[0]-181
    manager.publish_status()
    assert not auto.enabled and auto.state == 'LIMIT_REACHED'
    assert manager.goal is None and node.state == 'HOLD'


def test_autonomous_stale_map_during_scan_latches(task):
    manager, node, now = task
    auto = enable_auto(manager, node)
    node.ready = lambda: False
    manager.tick()
    node.ready = lambda: True
    manager.tick()
    assert not auto.enabled and manager.state == 'STOPPED'


def test_autonomous_repeated_enable_preserves_mission_and_failed_targets(task):
    manager, node, now = task
    auto = enable_auto(manager, node)
    auto.visited.append(np.array([1.,0.,0.]))
    original_started = auto.started
    response = auto.enable(NS(data=True), NS(success=False, message=''))
    assert response.success and len(auto.visited) == 1 and auto.started == original_started
    for _ in range(8):
        auto.target = np.array([2.,0.,0.])
        manager.finish('BLOCKED', 'NO_KNOWN_PATH')
    assert len(auto.rejected) == 8
    assert not auto.enabled and auto.state == 'BLOCKED:REPEATED_PLANNING_FAILURE'


def test_autonomous_disabled_service_stops_goal(task):
    manager, node, now = task
    auto = enable_auto(manager, node)
    manager.goal = np.array([2.,0.,0.])
    response = auto.enable(NS(data=False), NS(success=False, message=''))
    assert response.success and not auto.enabled
    assert manager.goal is None and node.state == 'HOLD'


def prepare_brief_scan(manager, node):
    auto = enable_auto(manager, node)
    auto.completed = 1
    auto.last_full_position = manager.position().copy()
    auto.dispatched_observed = int(np.count_nonzero(node.grid.observed))-100
    return auto


def test_autonomous_scan_starts_at_arrival_heading(task):
    manager, node, now = task
    node.odom.pose.pose.orientation.z = np.sin(1.1/2)
    node.odom.pose.pose.orientation.w = np.cos(1.1/2)
    auto = enable_auto(manager, node)
    command = manager.tick()
    assert abs(command.angular.z) < 1e-9
    assert auto.scan_views == 4
    assert np.allclose(auto.scan_direction, [np.cos(1.1), np.sin(1.1), 0.])


def test_autonomous_brief_scans_require_gain_and_periodic_panorama(task):
    manager, node, now = task
    auto = prepare_brief_scan(manager, node)
    auto.begin_scan()
    assert auto.scan_views == 1
    auto.begin_scan()
    assert auto.scan_views == 1
    auto.begin_scan()
    assert auto.scan_views == 4
    auto.dispatched_observed = int(np.count_nonzero(node.grid.observed))
    auto.begin_scan()
    assert auto.scan_views == 4


def test_autonomous_far_arrival_and_failure_force_panorama(task):
    manager, node, now = task
    auto = prepare_brief_scan(manager, node)
    auto.last_full_position = manager.position()+[4.,0.,0.]
    auto.begin_scan()
    assert auto.scan_views == 4
    auto.last_full_position = manager.position().copy()
    auto.failures = 1
    auto.begin_scan()
    assert auto.scan_views == 4


def test_brief_scan_without_candidate_falls_back_before_exhaustion(task, monkeypatch):
    manager, node, now = task
    auto = prepare_brief_scan(manager, node)
    auto.begin_scan()
    monkeypatch.setattr(manager, 'observation_command', lambda *a, **k: (Twist(), True))
    monkeypatch.setattr('uav_nav_sim.autonomous.frontier_viewpoint', lambda *a: None)
    manager.tick()
    assert auto.enabled and auto.scan_views == 4 and not auto.visited
    for _ in range(4):
        manager.tick()
    assert not auto.enabled and auto.state == 'FRONTIERS_EXHAUSTED'


def test_brief_scan_dispatches_without_three_extra_turns(task, monkeypatch):
    manager, node, now = task
    auto = prepare_brief_scan(manager, node)
    auto.begin_scan()
    monkeypatch.setattr(manager, 'observation_command', lambda *a, **k: (Twist(), True))
    monkeypatch.setattr('uav_nav_sim.autonomous.frontier_viewpoint', lambda *a: np.array([2.,0.,0.]))
    manager.tick()
    assert auto.enabled and auto.phase == 'GOAL'
    assert np.allclose(manager.goal, [2.,0.,0.])
