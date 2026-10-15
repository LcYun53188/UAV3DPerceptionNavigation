"""VIO-primary admission keeps EV strict while native terrain rejection is honest."""
import math
import pytest
from px4_msgs.msg import SensorOpticalFlow,DistanceSensor,EstimatorAidSource1d,EstimatorAidSource2d
from uav_mission.vio_gate import VioGate
from test_vio_gate import pose_update,CAL


def gate(profile='aligned_pose_mtf_primary_sim_v1'):
    g=VioGate(CAL,stable_s=0.,fusion_profile=profile);now=10.
    pose_update(g,now)
    g.samples['flags'].cs_opt_flow=g.samples['flags'].cs_rng_hgt=True
    for name,kind in [('optical_flow',EstimatorAidSource2d),('rng_hgt',EstimatorAidSource1d)]:
        g.receive(name,kind(timestamp=10000000,timestamp_sample=9848000,time_last_fuse=9848000,fused=True),now)
    g.receive('flow_input',SensorOpticalFlow(timestamp=10000000,timestamp_sample=10000000,
        quality=255,pixel_flow=[0.,0.],integration_timespan_us=10000,
        max_flow_rate=7.,min_ground_distance=.08,max_ground_distance=8.),now)
    g.receive('range_input',DistanceSensor(timestamp=10000000,orientation=25,
        current_distance=3.4,min_distance=.02,max_distance=8.),now)
    return g


def ready(g,now=10.):return g.ready(now,now,{k:1 for k in g.required})


def test_expected_box_top_range_rejection_is_not_a_vio_failure():
    g=gate();assert ready(g)
    aid=g.samples['rng_hgt'];aid.fused=False;aid.innovation_rejected=True;aid.test_ratio=7.747
    g.samples['range_input'].current_distance=1.2
    assert ready(g)
    assert g.auxiliary_fusion['rng_hgt']=='AUXILIARY_UNAVAILABLE'
    assert list(g.auxiliary_history)[-1]['innovation_rejected'] is True
    assert list(g.auxiliary_history)[-1]['fused'] is False
    aid.fused=True;aid.innovation_rejected=False;aid.test_ratio=.01
    assert ready(g) and g.auxiliary_fusion['rng_hgt']=='RECENT_FUSION'


def test_original_joint_ground_preflight_remains_strict():
    g=gate('aligned_pose_mtf_sim_v1');assert ready(g)
    g.samples['rng_hgt'].innovation_rejected=True;g.samples['rng_hgt'].test_ratio=7.747
    assert not ready(g) and g.reason=='VIO_MTF_NOT_FUSED:rng_hgt'
    from px4_vio_flight_session import JointSimGate
    assert not JointSimGate(CAL).primary_mtf


@pytest.mark.parametrize('fault',['ev_pos','ev_hgt','ev_yaw','stale_source','lost_source','local_reset','gnss'])
def test_primary_vio_failure_is_never_masked_by_auxiliary_policy(fault):
    g=gate();assert ready(g)
    if fault.startswith('ev_'):g.samples[fault].test_ratio=[1.1,0.] if fault=='ev_pos' else 1.1
    if fault=='stale_source':g.samples['source'].sample_stamp.sec=9
    if fault=='lost_source':g.samples['source'].valid=False
    if fault=='local_reset':
        g.samples['local'].xy_reset_counter+=1;g.receive('local',g.samples['local'],10.)
    if fault=='gnss':g.samples['flags'].cs_gnss_pos=True
    assert not ready(g)


@pytest.mark.parametrize('fault',['flow_old','range_old','flow_nan','range_nan','range_mount','flow_rate','aux_instance','aux_future'])
def test_primary_still_requires_physical_fresh_auxiliary_links_and_identity(fault):
    g=gate();assert ready(g)
    if fault=='flow_old':g.samples['flow_input'].timestamp_sample=9700000
    if fault=='range_old':g.samples['range_input'].timestamp=9700000
    if fault=='flow_nan':g.samples['flow_input'].pixel_flow=[math.nan,0.]
    if fault=='range_nan':g.samples['range_input'].current_distance=math.nan
    if fault=='range_mount':g.samples['range_input'].orientation=0
    if fault=='flow_rate':g.samples['flow_input'].integration_timespan_us=0
    if fault=='aux_instance':g.samples['rng_hgt'].estimator_instance=1
    if fault=='aux_future':g.samples['rng_hgt'].timestamp_sample=10100000
    assert not ready(g)


def test_unusable_auxiliary_measurement_gets_no_fusion_confidence_or_ev_exception():
    g=gate();assert ready(g)
    g.samples['flow_input'].quality=0
    g.samples['optical_flow'].innovation_rejected=True
    assert ready(g)
    assert g.auxiliary_fusion['optical_flow']=='AUXILIARY_UNAVAILABLE'
    g.samples['ev_pos'].innovation_rejected=True
    assert not ready(g)


def test_duplicate_auxiliary_writer_cannot_admit_primary_profile():
    g=gate();writers={n:1 for n in g.required};writers['range_input']=2
    assert not g.ready(10.,10.,writers)
