from copy import deepcopy
import math
from pathlib import Path
import sys
import numpy as np
import pytest
from uav_nav_interfaces.msg import PlanningContext,ContextTrajectory
from uav_mission.ego_execution import EgoExecution
from uav_mission.flight_geometry import Alignment
# Shared actual ROS messages used by the coordinate/session contract tests.
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'uav_nav_sim/test'))
from test_planning_context import inputs,goal,trajectory,stamp


def authorization(generation=1):return ('ekf-one',bytes([1]*16),bytes([2]*16),generation,'TASK',bytes([3]*16),1)


def setup():
    e=EgoExecution(Alignment((4.,-3.,1.),math.pi/2),lambda p:all(abs(x)<8 for x in p),.3,limits=(.5,1.,2.))
    for name,m in zip(('map','odom','alignment'),inputs()):
        if name=='map':m.source_stamp=deepcopy(m.header.stamp)
        e.gate.update(name,m,1.)
    e.start(goal(),authorization(),10.,1.)
    a=e.gate.inputs['alignment'][0]
    c=PlanningContext(context_id=e.gate.context_id,map_id='scene',map_epoch=1,map_version=1,
        localization_session=a.localization_session,alignment_id=a.alignment_id,alignment_generation=a.generation,reset_counters=a.reset_counters,valid=True)
    c.header.frame_id='map';c.header.stamp=stamp(10.1)
    return e,ContextTrajectory(context=c,trajectory=trajectory())


def refresh(e,ros,mono,map_version=None):
    for name,(original,_) in list(e.gate.inputs.items()):
        m=deepcopy(original);m.header.stamp=stamp(ros)
        if name=='map':m.version=map_version or m.version+1;m.source_stamp=deepcopy(m.header.stamp)
        if name=='odom':m.odometry.header=deepcopy(m.header)
        e.gate.update(name,m,mono)


def test_checked_curve_uses_original_clock_and_inverse_alignment():
    e,b=setup();e.admit(b,authorization(),10.1,1.1)
    p,done=e.sample(authorization(),10.2,1.2)
    assert p==pytest.approx([1.,0.,1.]) and not done
    refresh(e,10.8,1.8)
    p,done=e.sample(authorization(),10.8,1.8)
    assert p==pytest.approx(e.alignment.to_odom(e.curve(.3))) and p[1]<0 and not done
    refresh(e,20.6,11.6)
    p,done=e.sample(authorization(),20.6,11.6)
    assert done and p==pytest.approx([1.,-1.,1.])


@pytest.mark.parametrize('change',['owner','context','map','reset','start','unknown','limits','region'])
def test_admission_rechecks_authorization_context_geometry_and_motion(change):
    e,b=setup();auth=authorization()
    if change=='owner':auth=authorization(2)
    elif change=='context':b.context.context_id='old'
    elif change=='map':b.context.map_epoch+=1
    elif change=='reset':b.context.reset_counters[0]=1
    elif change=='start':b.trajectory.start_time=stamp(9.)
    elif change=='unknown':
        m=deepcopy(e.gate.inputs['map'][0]);m.version=2;m.header.stamp=stamp(10.05);m.observed=[0]*40000
        e.gate.update('map',m,1.05);b.context.map_version=2
    elif change=='limits':b.trajectory.knot_interval=.01
    elif change=='region':e.region=lambda p:False
    with pytest.raises(ValueError):e.admit(b,auth,10.1,1.1)
    assert e.curve is None


def test_newer_map_is_rechecked_instead_of_requiring_original_version():
    e,b=setup();refresh(e,10.05,1.05,map_version=2);b.context.map_version=2
    e.admit(b,authorization(),10.1,1.1)
    assert e.last_checked_version==2


@pytest.mark.parametrize('change',['lease','stale','map-obstacle','alignment','cancel'])
def test_active_curve_retires_and_never_resumes_after_fault(change):
    e,b=setup();e.admit(b,authorization(),10.1,1.1);auth=authorization()
    if change=='lease':auth=authorization(2)
    elif change=='map-obstacle':
        refresh(e,10.2,1.2)
        m=deepcopy(e.gate.inputs['map'][0]);m.version+=1;m.header.stamp=stamp(10.21);m.observed=[0]*40000;e.gate.update('map',m,1.21)
    elif change=='alignment':
        a=deepcopy(e.gate.inputs['alignment'][0]);a.generation+=1;a.header.stamp=stamp(10.2);e.gate.update('alignment',a,1.2)
    elif change=='cancel':e.retire()
    with pytest.raises(ValueError):e.sample(auth,10.7 if change=='stale' else 10.3,1.7 if change=='stale' else 1.3)
    assert e.curve is None and e.authorization is None
    refresh(e,10.8,1.8)
    with pytest.raises(ValueError):e.sample(authorization(),10.8,1.8)


def test_duplicate_results_do_not_replace_active_curve():
    e,b=setup();e.admit(b,authorization(),10.1,1.1)
    with pytest.raises(ValueError,match='ALREADY_BOUND'):e.admit(b,authorization(),10.11,1.11)
    assert e.curve is not None


def test_wait_has_fixed_planning_budget_and_retired_goals_are_rejected():
    e,b=setup();assert e.sample(authorization(),10.1,1.1)==(None,False)
    refresh(e,12.1,3.1)
    with pytest.raises(ValueError,match='PLANNER_TIMEOUT'):e.sample(authorization(),12.1,3.1)
    with pytest.raises(ValueError):e.admit(b,authorization(),12.2,3.2)


def test_fresh_headers_cannot_hide_stale_dynamic_map_measurement():
    e,b=setup();m=deepcopy(e.gate.inputs['map'][0]);m.version+=1;m.header.stamp=stamp(10.1);m.source_stamp=stamp(7.)
    e.gate.update('map',m,1.1)
    with pytest.raises(ValueError,match='STALE_MAP_SOURCE'):e.admit(b,authorization(),10.1,1.1)


def test_horizontal_braking_volume_does_not_inflate_vertical_body_into_ground():
    from uav_nav_sim.core import Grid
    from uav_mission.ego_execution import BrakingGrid
    observed=np.ones((40,40,30),dtype=bool);distance=np.full((40,40,30),4.,dtype=np.float32)
    distance[:,:,0]=0.  # Ground is occupied; centre height 1 m, body reserve .3 m.
    g=Grid(np.array([-4.,-4.,0.]),.2,distance,observed)
    assert not BrakingGrid(g,1.2).collision([0.,0.,1.],.3)
    assert g.collision([0.,0.,1.],1.5)


def test_horizontal_braking_rejects_obstacle_beyond_current_body():
    from uav_nav_sim.core import Grid
    from uav_mission.ego_execution import BrakingGrid
    distance=np.full((40,40,30),4.,dtype=np.float32);observed=np.ones_like(distance,dtype=bool)
    distance[25,20,5]=0.  # x=1 m: outside radius .3, inside horizontal stopping reserve.
    g=Grid(np.array([-4.,-4.,0.]),.2,distance,observed)
    assert not g.collision([0.,0.,1.],.3)
    assert BrakingGrid(g,1.2).collision([0.,0.,1.],.3)
