from types import SimpleNamespace as NS, MethodType
from unittest.mock import Mock

import numpy as np
import pytest
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.clock import ClockType
from rclpy.time import Time
from uav_nav_interfaces.msg import TimedTrajectory, MapSnapshot
from uav_nav_sim.core import Grid, spline, validate_handover, pack_grid_data
from uav_nav_sim.executor import Executor


def stamp(seconds):
    return Time(nanoseconds=round(seconds*1e9),clock_type=ClockType.ROS_TIME).to_msg()


def trajectory(curve, ident, start, parent=0):
    msg=TimedTrajectory(trajectory_id=ident,parent_trajectory_id=parent,map_id='map',epoch=1,map_version=1)
    msg.header.frame_id='odom'
    msg.start_time=stamp(start)
    msg.knot_interval=float(curve.t[4]-curve.t[3])
    msg.max_velocity,msg.max_acceleration,msg.max_jerk=1.,2.,4.
    msg.control_points=[Point(x=float(p[0]),y=float(p[1]),z=float(p[2])) for p in curve.c]
    return msg


@pytest.fixture
def executing():
    now=[100.]
    old=spline([[x,0.,0.] for x in [0,0,0,.4,.8,1.2,1.6,2,2,2]],1.)
    offset=3.
    p,v,a=(old(offset,d) for d in (0,1,2))
    dt=1.
    controls=[p-v*dt+a*dt*dt/3,p-a*dt*dt/6,p+v*dt+a*dt*dt/3,
              p+[.8,0,0],p+[1.2,0,0],p+[1.6,0,0],p+[2,0,0],p+[2,0,0],p+[2,0,0]]
    successor=spline(controls,dt)
    n=NS(curve=old,trajectory=trajectory(old,1,98),pending=None,session=('map',1),
         map=NS(version=1,map_id='map',epoch=1),last_id=1,state='EXECUTING',last_ros=None,
         odom=Odometry(),radius=.3,limits=[1.,2.,4.],navigation=Mock(),
         event=Mock(),command=Mock(),status=Mock(),ready=lambda:True,
         get_logger=lambda:Mock(),get_parameter=lambda k:NS(value=.15),
         get_clock=lambda:NS(now=lambda:Time(nanoseconds=round(now[0]*1e9),clock_type=ClockType.ROS_TIME)))
    n.grid=Grid(np.array([-3.,-3.,-3.]),.2,np.full((50,30,30),5.),np.ones((50,30,30),bool))
    n.odom.pose.pose.orientation.w=1.
    n.odom.pose.pose.position.x=float(old(2)[0])
    n.odom.twist.twist.linear.x=float(old(2,1)[0])
    n.navigation.accepts.return_value=True
    n.stop=MethodType(Executor.stop,n)
    n.publish_hold=Mock()
    return n,now,successor,trajectory(successor,2,101,1)


def test_splice_matches_future_state_not_current_position(executing):
    n,now,curve,msg=executing
    validate_handover(n.curve,curve,3.)
    with pytest.raises(ValueError,match='Discontinuous'):
        validate_handover(n.curve,curve,2.)
    with pytest.raises(ValueError,match='outside'):
        validate_handover(n.curve,curve,100.)


@pytest.mark.parametrize('derivative',[0,1,2])
def test_each_boundary_derivative_is_checked(executing,derivative):
    n,now,curve,msg=executing
    controls=curve.c.copy()
    # Independent changes of position, velocity, acceleration at a cubic endpoint.
    delta=([1,1,1],[-1,0,1],[1/3,-1/6,1/3])[derivative]
    controls[:3,1]+=np.array(delta)*.02
    with pytest.raises(ValueError,match='Discontinuous'):
        validate_handover(n.curve,spline(controls,1.),3.)


def test_queued_successor_does_not_interrupt_old_and_switches_while_moving(executing):
    n,now,curve,msg=executing
    old=n.curve
    Executor.trajectory_cb(n,msg)
    assert n.curve is old and n.pending[1] is msg
    n.command.publish.assert_not_called()
    now[0]=100.8
    n.odom.pose.pose.position.x=float(old(2.8)[0])
    Executor.tick(n)
    assert n.curve is old and n.command.publish.call_args[0][0].linear.x>.3
    now[0]=101.
    n.odom.pose.pose.position.x=float(old(3.)[0])
    Executor.tick(n)
    assert n.trajectory is msg and n.pending is None
    assert n.command.publish.call_args[0][0].linear.x>.3
    n.navigation.handover.assert_called_once()
    assert 'HANDOVER:1->2' in [c.args[0].data for c in n.event.publish.call_args_list]


@pytest.mark.parametrize('fault',['parent','late','discontinuous','dynamics','session','future_map'])
def test_bad_successor_preserves_current_stop_path(executing,fault):
    n,now,curve,msg=executing
    old=n.curve
    if fault=='parent': msg.parent_trajectory_id=7
    elif fault=='late': msg.start_time=stamp(99.)
    elif fault=='discontinuous': msg.control_points[0].y+=.1
    elif fault=='dynamics': msg.max_velocity=.01
    elif fault=='session': msg.epoch=2
    elif fault=='future_map': msg.map_version=7
    Executor.trajectory_cb(n,msg)
    assert n.curve is old and n.pending is None and n.state=='EXECUTING'
    n.command.publish.assert_not_called()
    n.navigation.failed_segment.assert_not_called()
    n.navigation.replan_failed.assert_called_once()


@pytest.mark.parametrize('reason',['CANCELLED','MAP_SESSION_CHANGED','CLOCK_RESET','STALE_MAP_OR_ODOMETRY'])
def test_stop_discards_queued_successor(executing,reason):
    n,now,curve,msg=executing
    Executor.trajectory_cb(n,msg)
    n.stop(reason)
    assert n.curve is None and n.pending is None
    Executor.trajectory_cb(n,msg)
    assert n.curve is None and n.pending is None and n.state=='HOLD'


def test_pending_collision_discards_only_successor(executing):
    n,now,curve,msg=executing
    Executor.trajectory_cb(n,msg)
    old=n.curve
    distance=n.grid.distance.copy()
    distance[tuple(n.grid.index([2.7,0,0]))]=-.1
    grid=Grid(n.grid.origin.copy(),.2,distance,n.grid.observed.copy())
    update=MapSnapshot(map_id='map',epoch=1,version=2,valid=True,resolution=.2)
    update.origin.x=update.origin.y=update.origin.z=-3.
    update.shape=list(distance.shape);update.distance,update.observed=pack_grid_data(grid)
    Executor.map_cb(n,update)
    assert n.curve is old and n.pending is None
    n.navigation.replan_failed.assert_called_once()


def test_late_control_tick_discards_successor_and_keeps_old(executing):
    n,now,curve,msg=executing
    Executor.trajectory_cb(n,msg)
    old=n.curve
    now[0]=101.2
    n.odom.pose.pose.position.x=float(old(3.2)[0])
    Executor.tick(n)
    assert n.curve is old and n.pending is None
    assert n.command.publish.call_args[0][0].linear.x>.3
    n.navigation.handover.assert_not_called()
