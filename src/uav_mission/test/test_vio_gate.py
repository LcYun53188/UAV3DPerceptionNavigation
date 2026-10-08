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
