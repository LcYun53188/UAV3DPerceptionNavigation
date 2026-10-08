from copy import deepcopy
import math

import numpy as np
import pytest
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time
from uav_nav_interfaces.msg import MapSnapshot, LocalizationAlignment, TimedTrajectory, LocalizedOdometry
from uav_nav_sim.planning_context import PlanningGate, transform_odometry


def stamp(seconds):return Time(sec=int(seconds),nanosec=round((seconds-int(seconds))*1e9))


def inputs():
    m=MapSnapshot(map_id='scene',epoch=1,version=1,valid=True,resolution=.2,shape=[40,40,25])
    m.header.frame_id='map';m.header.stamp=stamp(10.)
    m.origin=Point(x=0.,y=-6.,z=0.)
    m.distance=np.full(40000,2.,dtype=np.float32);m.observed=[1]*40000
    o=Odometry();o.header.frame_id='odom';o.child_frame_id='base_link';o.header.stamp=stamp(10.)
    o.pose.pose.position=Point(x=1.,y=0.,z=1.);o.pose.pose.orientation.w=1.
    a=LocalizationAlignment(map_id='scene',map_epoch=1,localization_session='ekf-one',
                            alignment_id='survey-one',generation=1,valid=True)
    a.header.frame_id='map';a.header.stamp=stamp(10.)
    a.map_to_odom.translation.x=4.;a.map_to_odom.translation.y=-3.;a.map_to_odom.translation.z=1.
    a.map_to_odom.rotation.z=math.sin(math.pi/4);a.map_to_odom.rotation.w=math.cos(math.pi/4)
    return m,LocalizedOdometry(header=deepcopy(o.header),localization_session=a.localization_session,odometry=o),a


def gate():
    g=PlanningGate()
    for name,message in zip(('map','odom','alignment'),inputs()):g.update(name,message,1.)
    assert g.ready(10.,1.)
    return g


def goal():
    p=PoseStamped();p.header.frame_id='map';p.header.stamp=stamp(10.)
    p.pose.position=Point(x=5.,y=-2.,z=2.);p.pose.orientation.w=1.
    return p


def trajectory():
    t=TimedTrajectory(map_id='scene',epoch=1,map_version=1,trajectory_id=1,knot_interval=2.)
    t.header.frame_id='map';t.header.stamp=stamp(10.);t.goal_stamp=stamp(10.);t.start_time=stamp(10.5)
    t.control_points=[Point(x=4.+x,y=-2.,z=2.) for x in (0.,0.,0.,.25,.75,1.,1.,1.)]
    return t


def test_pose_orientation_covariance_and_body_twist_transform():
    _,wrapped,a=inputs();o=wrapped.odometry;o.twist.twist.linear.x=2.
    o.pose.covariance=np.diag([1.,4.,9.,16.,25.,36.]).ravel().tolist()
    o.pose.covariance[4]=o.pose.covariance[24]=.2
    result=transform_odometry(o,a.map_to_odom)
    assert [result.pose.pose.position.x,result.pose.pose.position.y,result.pose.pose.position.z]==pytest.approx([4.,-2.,2.])
    assert result.pose.pose.orientation.z==pytest.approx(math.sin(math.pi/4))
    assert np.diag(np.array(result.pose.covariance).reshape(6,6))==pytest.approx([4.,1.,9.,25.,16.,36.])
    assert np.array(result.pose.covariance).reshape(6,6)[1,3]==pytest.approx(-.2)
    assert result.twist==o.twist and result.header.stamp==o.header.stamp
    assert o.header.frame_id=='odom' and result.header.frame_id=='map'


def test_bound_result_is_exactly_once_and_has_checked_endpoints():
    g=gate();g.dispatch(goal(),10.,1.)
    assert g.bind(trajectory(),10.1,1.1).header.frame_id=='map'
    with pytest.raises(ValueError,match='NO_CURRENT_GOAL'):g.bind(trajectory(),10.1,1.1)
    with pytest.raises(ValueError,match='REPLAYED_GOAL'):g.dispatch(goal(),10.1,1.1)


@pytest.mark.parametrize('change', ['frame','session','epoch','version','parent','goal_stamp','expired','start','endpoint','limits','unknown'])
def test_unsafe_or_stale_raw_result_is_rejected(change):
    g=gate();g.dispatch(goal(),10.,1.);t=trajectory()
    if change=='frame':t.header.frame_id='odom'
    elif change=='session':t.map_id='old'
    elif change=='epoch':t.epoch=0
    elif change=='version':t.map_version=100
    elif change=='parent':t.parent_trajectory_id=99
    elif change=='goal_stamp':t.goal_stamp=stamp(9.)
    elif change=='expired':t.header.stamp=stamp(9.)
    elif change=='start':t.start_time=stamp(9.)
    elif change=='endpoint':
        for point in t.control_points[-3:]:point.x+=.5
    elif change=='limits':t.knot_interval=.01
    elif change=='unknown':
        m=deepcopy(g.inputs['map'][0]);m.header.stamp=stamp(10.05);m.version=2;m.observed=[0]*40000
        g.update('map',m,1.05)
    with pytest.raises(ValueError):g.bind(t,10.1,1.1)


@pytest.mark.parametrize('field', ['reset','translation','localization','alignment','map'])
def test_same_generation_alignment_change_latches_and_retires_goal(field):
    g=gate();g.dispatch(goal(),10.,1.)
    a=deepcopy(g.inputs['alignment'][0]);a.header.stamp=stamp(10.1)
    if field=='reset':a.reset_counters[2]=1
    elif field=='translation':a.map_to_odom.translation.x+=1.
    elif field=='localization':a.localization_session='new'
    elif field=='alignment':a.alignment_id='new'
    elif field=='map':a.map_id='new'
    g.update('alignment',a,1.1)
    assert not g.ready(10.1,1.1) and g.goal is None
    a.header.stamp=stamp(10.2);g.update('alignment',a,1.2)
    assert not g.ready(10.2,1.2)


def test_new_generation_requires_new_goal_and_cannot_reuse_goal_stamp():
    g=gate();g.dispatch(goal(),10.,1.);old=g.context_id
    a=deepcopy(g.inputs['alignment'][0]);a.generation+=1;a.header.stamp=stamp(10.1)
    g.update('alignment',a,1.1)
    assert g.ready(10.1,1.1) and g.context_id!=old and g.goal is None
    with pytest.raises(ValueError,match='REPLAYED_GOAL'):g.dispatch(goal(),10.1,1.1)
    with pytest.raises(ValueError):g.bind(trajectory(),10.1,1.1)


@pytest.mark.parametrize('source', ['map','odom','alignment'])
def test_stale_input_cancels_goal_and_replay_cannot_renew(source):
    g=gate();g.dispatch(goal(),10.,1.)
    message=deepcopy(g.inputs[source][0])
    assert not g.update(source,message,1.6)
    delay=2.1 if source=='map' else .6
    for name,message in zip(('map','odom','alignment'),inputs()):
        if name==source:continue
        message.header.stamp=stamp(10.+delay)
        if name=='map':message.version=2
        if name=='odom':message.odometry.header=deepcopy(message.header)
        g.update(name,message,1.+delay)
    assert not g.ready(10.+delay,1.+delay) and g.goal is None
    assert g.reason=='STALE_'+source.upper()


def test_map_epoch_change_needs_explicit_rebinding_and_no_rollback():
    g=gate();g.dispatch(goal(),10.,1.)
    previous=deepcopy(g.inputs['map'][0]);m=deepcopy(previous)
    m.epoch=2;m.version=1;m.header.stamp=stamp(10.1);g.update('map',m,1.1)
    assert not g.ready(10.1,1.1)
    previous.header.stamp=stamp(10.2)
    assert not g.update('map',previous,1.2)
    assert g.inputs['map'][0].epoch==2


def test_clock_reset_latches_even_after_fresh_inputs_return():
    g=gate();g.dispatch(goal(),10.,1.)
    assert not g.ready(9.,1.1) and g.clock_fault and g.goal is None
    for name,m in zip(('map','odom','alignment'),inputs()):
        m.header.stamp=stamp(11.);g.update(name,m,1.2)
    assert not g.ready(11.,1.2)


@pytest.mark.parametrize('bad', ['nan','quaternion','covariance','child','world'])
def test_invalid_pose_or_covariance_rejected(bad):
    _,wrapped,a=inputs();o=wrapped.odometry
    if bad=='nan':o.pose.pose.position.x=math.nan
    elif bad=='quaternion':o.pose.pose.orientation.w=0.
    elif bad=='covariance':o.pose.covariance[0]=-1.
    elif bad=='child':o.child_frame_id='frd'
    elif bad=='world':o.header.frame_id='map'
    with pytest.raises(ValueError):transform_odometry(o,a.map_to_odom)


@pytest.mark.parametrize('mismatch', ['session','reset','stamp'])
def test_localization_identity_must_travel_with_pose(mismatch):
    g=gate();g.dispatch(goal(),10.,1.)
    o=deepcopy(g.inputs['odom'][0]);o.header.stamp=stamp(10.1);o.odometry.header=deepcopy(o.header)
    if mismatch=='session':o.localization_session='restarted-ekf'
    elif mismatch=='reset':o.reset_counters[5]=1
    elif mismatch=='stamp':o.odometry.header.stamp=stamp(9.)
    g.update('odom',o,1.1)
    assert not g.ready(10.1,1.1) and g.reason=='LOCALIZATION_SESSION_MISMATCH'
    assert g.goal is None


def test_trajectory_cannot_use_map_version_before_goal_admission():
    g=gate();m=deepcopy(g.inputs['map'][0]);m.version=5;m.header.stamp=stamp(10.05)
    g.update('map',m,1.05);g.dispatch(goal(),10.05,1.05)
    with pytest.raises(ValueError,match='STALE_OR_UNBOUND'):g.bind(trajectory(),10.1,1.1)


def test_full_rigid_alignment_includes_roll_and_translation():
    _,wrapped,a=inputs();o=wrapped.odometry
    a.map_to_odom.rotation.z=0.;a.map_to_odom.rotation.x=math.sin(math.pi/4)
    result=transform_odometry(o,a.map_to_odom)
    p=result.pose.pose.position
    assert [p.x,p.y,p.z]==pytest.approx([5.,-4.,1.])
    assert result.pose.pose.orientation.x==pytest.approx(math.sin(math.pi/4))


def test_observed_clock_stall_retires_goal_and_latches():
    g=gate();g.dispatch(goal(),10.,1.)
    assert not g.ready(10.,1.6) and g.reason=='CLOCK_FAULT' and g.goal is None
    assert not g.ready(10.1,1.7)


def test_reset_identity_mismatch_requires_new_alignment_generation_to_recover():
    g=gate();g.dispatch(goal(),10.,1.)
    o=deepcopy(g.inputs['odom'][0]);o.header.stamp=stamp(10.1);o.odometry.header=deepcopy(o.header)
    o.reset_counters[5]=1;g.update('odom',o,1.1)
    assert not g.ready(10.1,1.1)
    o.reset_counters[5]=0;o.header.stamp=stamp(10.2);o.odometry.header=deepcopy(o.header)
    g.update('odom',o,1.2)
    assert not g.ready(10.2,1.2) and g.goal is None
    a=deepcopy(g.inputs['alignment'][0]);a.generation=2;a.header.stamp=stamp(10.3)
    g.update('alignment',a,1.3)
    assert g.ready(10.3,1.3) and g.goal is None
