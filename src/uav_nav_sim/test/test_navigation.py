from types import SimpleNamespace as NS
from unittest.mock import Mock

import numpy as np
import pytest
from geometry_msgs.msg import PoseStamped
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
              create_timer=lambda *a, **k: Mock(), get_logger=lambda: Mock(),
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
