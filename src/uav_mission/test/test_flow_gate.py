"""Standalone flow must remain independent of VIO and reject broken native aids."""
from copy import deepcopy
import json
from types import SimpleNamespace

import pytest
from px4_msgs.msg import EstimatorAidSource1d, EstimatorAidSource2d, DistanceSensor, SensorOpticalFlow
from uav_mission.flow_gate import FlowGate
from uav_mission.px4_flight import FlightServer
from test_vio_gate import pose_update, CAL
from uav_mission.vio_gate import VioGate

WRITERS = {n:1 for n in FlowGate.required}


def update(gate, now, reset=0):
    fixture=VioGate(CAL, fusion_profile='aligned_pose_v1')
    pose_update(fixture,now)
    messages=fixture.samples
    f=messages['flags']
    f.cs_ev_pos=f.cs_ev_hgt=f.cs_ev_yaw=False
    f.cs_opt_flow=f.cs_rng_hgt=f.cs_mag_hdg=True
    f.cs_yaw_align=f.cs_tilt_align=f.cs_mag_heading_consistent=True
    p=messages['local'];p.xy_reset_counter=reset;p.dist_bottom_valid=True;p.dist_bottom=1.2
    messages['optical_flow']=EstimatorAidSource2d(timestamp=int(now*1e6),timestamp_sample=int(now*1e6),time_last_fuse=int(now*1e6),fused=True)
    messages['rng_hgt']=EstimatorAidSource1d(timestamp=int(now*1e6),timestamp_sample=int(now*1e6),time_last_fuse=int(now*1e6),fused=True)
    messages['range_sensor']=DistanceSensor(timestamp=int(now*1e6),orientation=25,current_distance=1.2)
    messages['flow_sensor']=SensorOpticalFlow(timestamp=int(now*1e6),quality=100,integration_timespan_us=10000)
    for name in gate.required:gate.receive(name,messages[name],now)


def ready():
    g=FlowGate()
    for t in (10.,11.,12.):
        update(g,t);g.ready(t,t,WRITERS)
    assert g.reason=='READY'
    return g


def test_vio_missing_stopped_or_reset_does_not_affect_native_flow():
    g=ready()
    g.receive('source',SimpleNamespace(valid=False,localization_session='changed'),12.)
    g.receive('ev_hgt',SimpleNamespace(fused=False),12.)
    assert g.ready(12.,12.,WRITERS)
    assert set(g.samples)==set(g.required)


def test_initial_native_magnetic_alignment_and_actual_range_admit_without_terrain_state():
    g=ready()
    g.samples['local'].heading_good_for_control=False
    g.samples['local'].dist_bottom_valid=False
    assert g.ready(12.,12.,WRITERS)
    g.samples['flags'].cs_yaw_align=False
    assert not g.ready(12.,12.,WRITERS)


@pytest.mark.parametrize('ros,now',[(11.,12.1),(12.,12.6)])
def test_clock_reset_or_stall_latches(ros,now):
    g=ready()
    assert not g.ready(now,ros,WRITERS)
    update(g,13.)
    assert not g.ready(13.,13.,WRITERS)


@pytest.mark.parametrize('fault',['range_stale','flow_stale','flow_no_fusion','range_no_fusion',
    'ev_height','ev_yaw','gnss','heading','selector','duplicate_writer','local_nan','range_invalid'])
def test_native_flow_admission_fails_closed(fault):
    g=ready();writers=dict(WRITERS)
    if fault=='range_stale':g.samples['rng_hgt'].timestamp=1
    elif fault=='flow_stale':g.samples['optical_flow'].timestamp_sample=1
    elif fault=='flow_no_fusion':g.samples['optical_flow'].time_last_fuse=1
    elif fault=='range_no_fusion':g.samples['rng_hgt'].time_last_fuse=1
    elif fault=='ev_height':g.samples['flags'].cs_ev_hgt=True
    elif fault=='ev_yaw':g.samples['flags'].cs_ev_yaw=True
    elif fault=='gnss':g.samples['flags'].cs_gnss_pos=True
    elif fault=='heading':g.samples['flags'].cs_mag_hdg=False
    elif fault=='selector':g.samples['selector'].healthy[0]=False
    elif fault=='duplicate_writer':writers['optical_flow']=2
    elif fault=='local_nan':g.samples['local'].x=float('nan')
    else:g.samples['range_sensor'].current_distance=float('nan')
    assert not g.ready(12.,12.,writers)


def test_estimator_reset_after_binding_latches_even_if_next_sample_recovers():
    g=ready();update(g,12.1,reset=1);update(g,12.2)
    assert not g.ready(12.2,12.2,WRITERS) and g.fault=='FLOW_ESTIMATOR_RESET'


def test_startup_reset_requires_new_continuous_window():
    g=FlowGate();update(g,10.);g.ready(10.,10.,WRITERS)
    update(g,11.,reset=1)
    assert not g.ready(11.,11.,WRITERS)
    for t in (12.,13.1):update(g,t,reset=1);g.ready(t,t,WRITERS)
    assert g.reason=='READY'


def test_near_ground_flow_exception_only_after_native_landing_and_still_requires_range():
    g=ready();g.samples['range_sensor'].current_distance=.1
    g.samples['flags'].cs_opt_flow=False;g.samples['optical_flow'].timestamp=1
    assert not g.ready(12.,12.,WRITERS)
    # Restore the health warmup first: a landing exception never waives stability.
    g=ready();g.samples['range_sensor'].current_distance=.1
    g.samples['flags'].cs_opt_flow=False;g.samples['optical_flow'].timestamp=1
    assert g.ready(12.,12.,WRITERS,native_landing=True)
    g.samples['rng_hgt'].time_last_fuse=1
    assert not g.ready(12.,12.,WRITERS,native_landing=True)


@pytest.mark.parametrize('kind',['NAVIGATE','RETURN','OBSERVE'])
def test_flow_recipe_cannot_claim_mapped_navigation_or_return(kind):
    f=SimpleNamespace(nonce='owned',localization_profile='FLOW_LOCAL',instance='x')
    r=SimpleNamespace(backend='PX4_KNOWN_REGION',mission_type='FLIGHT_SEQUENCE',timeout_s=60.,
        parameters_json=json.dumps(dict(authorization='owned',steps=[dict(type='TAKEOFF',height_m=1.2),
            dict(type=kind),dict(type='LAND')])))
    with pytest.raises(ValueError,match='mapped tasks require VIO'):FlightServer.parse(f,r)
