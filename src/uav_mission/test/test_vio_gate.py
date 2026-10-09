from types import SimpleNamespace
import pytest
from px4_msgs.msg import EstimatorStatusFlags, EstimatorAidSource1d, EstimatorAidSource2d, EstimatorAidSource3d, EstimatorSelectorStatus
from uav_nav_interfaces.msg import VioStatus
from uav_mission.vio_gate import VioGate
from uav_mission.px4_flight import FlightServer

CAL = 'a'*64
WRITERS = {n: 1 for n in VioGate.required}


def update(gate, now=10., session='vio-a', reset=0):
    source = VioStatus(localization_session=session, calibration_id=CAL, reset_counter=reset, valid=True)
    source.header.frame_id = 'odom'
    source.header.stamp.sec = int(now)
    source.header.stamp.nanosec = int(round((now-int(now))*1e9))
    source.sample_stamp = source.header.stamp
    flags = EstimatorStatusFlags(timestamp=int(now*1e6),cs_ev_pos=True,cs_ev_hgt=True,cs_ev_vel=True,cs_ev_yaw=True)
    gate.receive('source', source, now)
    gate.receive('flags', flags, now)
    gate.receive('selector', EstimatorSelectorStatus(timestamp=int(now*1e6),
        instances_available=1,healthy=[True]+[False]*8),now)
    for name,kind in [('ev_pos',EstimatorAidSource2d),('ev_hgt',EstimatorAidSource1d),('ev_vel',EstimatorAidSource3d),('ev_yaw',EstimatorAidSource1d)]:
        gate.receive(name,kind(timestamp=int(now*1e6),timestamp_sample=int(now*1e6),time_last_fuse=int(now*1e6),fused=True),now)


def ready_gate():
    gate = VioGate(CAL)
    for now in (10.,10.5,11.,11.5,12.):
        update(gate, now)
        ready = gate.ready(now, now, WRITERS)
    assert ready
    return gate


def test_continuous_fusion_required_for_two_seconds():
    gate = VioGate(CAL)
    update(gate)
    assert not gate.ready(10.,10.,WRITERS)
    assert gate.reason == 'VIO_STABILIZING'
    ready_gate()


def test_flags_alone_do_not_prove_fusion():
    gate = ready_gate()
    del gate.samples['ev_pos']
    assert not gate.ready(12.,12.,WRITERS)
    assert gate.reason == 'VIO_TELEMETRY_MISSING'


@pytest.mark.parametrize('name', ['ev_pos','ev_hgt','ev_vel','ev_yaw'])
@pytest.mark.parametrize('field,value', [('fused',False),('innovation_rejected',True),
                                       ('time_last_fuse',1),('timestamp_sample',1),('estimator_instance',1)])
def test_unfused_or_wrong_estimator_rejected(name,field,value):
    gate = ready_gate()
    setattr(gate.samples[name],field,value)
    assert not gate.ready(12.,12.,WRITERS)


@pytest.mark.parametrize('field', ['cs_ev_yaw_fault','reject_hor_pos','reject_ver_pos','reject_yaw',
                                  'cs_fake_pos','cs_inertial_dead_reckoning','fs_bad_hdg'])
def test_filter_fault_rejected(field):
    gate = ready_gate()
    setattr(gate.samples['flags'],field,True)
    assert not gate.ready(12.,12.,WRITERS)


@pytest.mark.parametrize('change', ['session','reset'])
def test_bound_identity_change_latches(change):
    gate = ready_gate()
    update(gate,12.05,session='vio-b' if change=='session' else 'vio-a',reset=1 if change=='reset' else 0)
    assert not gate.ready(12.05,12.05,WRITERS)
    update(gate,12.1)
    assert not gate.ready(12.1,12.1,WRITERS)
    assert gate.fault == 'VIO_SESSION_CHANGED'


def test_source_replacement_restarts_warmup():
    gate = VioGate(CAL)
    for now in (10.,10.5,11.,11.5):
        update(gate,now)
        assert not gate.ready(now,now,WRITERS)
    update(gate,12.,session='vio-b')
    assert not gate.ready(12.,12.,WRITERS)
    assert gate.stable_since == 12.


@pytest.mark.parametrize('change', ['stale_source','duplicate_writer','wrong_calibration','tracking_lost','clock_reset','clock_stall','large_innovation'])
def test_health_loss(change):
    gate = ready_gate()
    now,ros,writers = 12.05,12.05,dict(WRITERS)
    if change == 'stale_source': gate.samples['source'].sample_stamp.sec = 10
    if change == 'duplicate_writer': writers['source'] = 2
    if change == 'wrong_calibration': gate.samples['source'].calibration_id = 'b'*64
    if change == 'tracking_lost': gate.samples['source'].valid = False
    if change == 'clock_reset': ros = 11.
    if change == 'clock_stall': now,ros = 12.6,12.
    if change == 'large_innovation': gate.samples['ev_yaw'].test_ratio = 1.1
    assert not gate.ready(now,ros,writers)


def test_gateway_cannot_bypass_enabled_gate():
    f = SimpleNamespace(vio_gate=VioGate(CAL),vio_graph_checked=0.,vio_topics={'source':'/uav/vio/status'},
        get_publishers_info_by_topic=lambda topic: [],get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10000000000)))
    # VIO rejection occurs before any generic aircraft state can admit a task.
    assert not FlightServer.healthy(f,ground=True)


def test_w0_profile_does_not_require_vio():
    assert FlightServer.vio_healthy(SimpleNamespace())


def test_tracking_loss_between_control_ticks_cannot_be_erased():
    gate = ready_gate()
    source = gate.samples['source']
    source.valid = False
    gate.receive('source', source, 12.01)
    update(gate,12.02)
    assert not gate.ready(12.02,12.02,WRITERS)
    assert gate.fault == 'VIO_SOURCE_LOST'


def test_primary_estimator_switch_latches():
    gate = ready_gate()
    selector = gate.samples['selector']
    selector.primary_instance = 1
    selector.instance_changed_count = 1
    gate.receive('selector', selector,12.01)
    update(gate,12.02)
    assert not gate.ready(12.02,12.02,WRITERS)
    assert gate.fault == 'VIO_EKF_INSTANCE_CHANGED'


@pytest.mark.parametrize('stream', ['flags', 'selector'])
def test_one_hz_status_does_not_break_continuous_readiness(stream):
    gate = VioGate(CAL)
    previous_selector = None
    previous_receive = None
    for index in range(26):
        now = 10. + index*.1
        update(gate,now)
        if index % 10:
            gate.samples[stream] = previous_selector
            gate.received[stream] = previous_receive
        else:
            previous_selector = gate.samples[stream]
            previous_receive = now
        ready = gate.ready(now,now,WRITERS)
        if index >= 20:
            assert ready
    gate.received[stream] = 10.
    assert not gate.ready(12.5,12.5,WRITERS)
    assert gate.reason == 'VIO_TELEMETRY_STALE:'+stream


def pose_update(gate,now=10.):
    from px4_msgs.msg import VehicleLocalPosition
    update(gate,now)
    gate.samples['source'].header.frame_id='px4_local_enu'
    gate.samples['flags'].cs_ev_vel=False
    gate.receive('local',VehicleLocalPosition(timestamp=int(now*1e6),timestamp_sample=int(now*1e6),
        xy_valid=True,z_valid=True,v_xy_valid=True,v_z_valid=True,heading_good_for_control=True,
        heading_var=.01,eph=.1,epv=.1,evh=.1,evv=.1),now)
    gate.samples.pop('ev_vel',None)


def pose_gate():
    gate=VioGate(CAL,fusion_profile='aligned_pose_v1')
    for t in (10.,10.5,11.,11.5,12.):
        pose_update(gate,t)
        result=gate.ready(t,t,{n:1 for n in gate.required})
    assert result
    return gate


def test_explicit_pose_profile_accepts_three_fused_aids_and_local_velocity():
    gate=pose_gate()
    assert 'ev_vel' not in gate.required and 'local' in gate.required
    default=VioGate(CAL)
    pose_update(default)
    assert not default.ready(10.,10.,WRITERS)  # Default four-aid requirement remains.
    with pytest.raises(ValueError):VioGate(CAL,fusion_profile='unknown')


@pytest.mark.parametrize('fault',['ev_velocity','gnss','mag','flow','range','aux','invalid_velocity','nan_velocity',
    'large_velocity_sigma','zero_velocity_sigma','local_old','heading_invalid','dead_reckoning','position_sigma','reset'])
def test_pose_profile_rejects_hidden_aids_and_unhealthy_ekf_velocity(fault):
    gate=pose_gate();local=gate.samples['local'];flags=gate.samples['flags']
    if fault=='ev_velocity':flags.cs_ev_vel=True
    if fault=='gnss':flags.cs_gnss_pos=True
    if fault=='mag':flags.cs_mag=True
    if fault=='flow':flags.cs_opt_flow=True
    if fault=='range':flags.cs_rng_hgt=True
    if fault=='aux':flags.cs_aux_gpos=True
    if fault=='invalid_velocity':local.v_xy_valid=False
    if fault=='nan_velocity':local.vx=float('nan')
    if fault=='large_velocity_sigma':local.evv=.6
    if fault=='zero_velocity_sigma':local.evh=0.
    if fault=='local_old':local.timestamp_sample=1
    if fault=='heading_invalid':local.heading_good_for_control=False
    if fault=='dead_reckoning':local.dead_reckoning=True
    if fault=='position_sigma':local.eph=.6
    if fault=='reset':local.vxy_reset_counter+=1
    assert not gate.ready(12.,12.,{n:1 for n in gate.required})


def test_pose_local_reset_between_ticks_is_latched():
    gate=pose_gate();local=gate.samples['local'];local.heading_reset_counter=1
    gate.receive('local',local,12.01)
    pose_update(gate,12.02)
    assert not gate.ready(12.02,12.02,{n:1 for n in gate.required})
    assert gate.fault=='VIO_EKF_LOCAL_RESET'


def test_flight_gateway_forwards_pose_only_local_health_and_reset():
    from px4_msgs.msg import VehicleLocalPosition
    gate=VioGate(CAL,fusion_profile='aligned_pose_v1')
    server=SimpleNamespace(vio_gate=gate)
    local=VehicleLocalPosition(timestamp=10000000)
    FlightServer.forward_vio_local(server,'vehicle_local_position',local,10.)
    assert gate.samples['local'] is local and gate.received['local']==10.
    gate.local_identity=(0,0,0,0,0)
    local.xy_reset_counter=1
    FlightServer.forward_vio_local(server,'vehicle_local_position',local,10.02)
    assert gate.fault=='VIO_EKF_LOCAL_RESET'


def test_full_odometry_flight_profile_retains_velocity_aid_requirement():
    server=SimpleNamespace(vio_gate=VioGate(CAL))
    FlightServer.forward_vio_local(server,'vehicle_local_position',object(),10.)
    assert 'ev_vel' in server.vio_gate.required and 'local' not in server.vio_gate.samples
