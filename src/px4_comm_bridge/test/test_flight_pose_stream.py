"""Flight continuity policy only; no simulator or flight authorization evidence."""
import pytest
from px4_comm_bridge.pose_stream import AlignedPoseStream, FlightAlignedPoseStream
from test_pose_stream import source
from test_pose_fusion import ID


def accept(stream,t,*,ground=False,armed=False):
    p,s=source(t)
    return stream.accept(p,s,t,t,pose_gid='pose',status_gid='status',ground=ground,armed=armed)


def bind():
    s=FlightAlignedPoseStream(ID,[0.,0.,0.],0.,ID)
    for i in range(52):accept(s,10+i*.04,ground=True)
    assert s.bound and s.count==1
    return s


def test_armed_cannot_initialize_or_reuse_ground_warmup():
    s=FlightAlignedPoseStream(ID,[0.,0.,0.],0.,ID)
    for i in range(50):accept(s,10+i*.04,ground=True)
    assert s.window and not s.bound
    for i in range(60):assert accept(s,12+i*.04,armed=True) is None
    assert not s.window and not s.bound and s.count==0


def test_ground_bound_alignment_continues_in_air_and_after_landing_once():
    s=bind();binding=s.alignment.binding
    for i in range(20):
        t=12.08+i*.04
        output=accept(s,t,armed=True)
        assert output is not None and output[1].timestamp_sample==round(t*1e6)
        assert s.alignment.binding is binding
    assert accept(s,12.88,ground=True) is not None
    assert s.alignment.binding is binding and not s.fault


def test_default_disarmed_audit_cannot_continue_armed():
    s=AlignedPoseStream(ID,[0.,0.,0.],0.,ID)
    for i in range(52):accept(s,10+i*.04,ground=True)
    assert accept(s,12.08,armed=True) is None
    assert s.fault=='VIO_ALIGNMENT_REQUIRES_GROUND'


@pytest.mark.parametrize('fault',['state','gap','stale','clock','identity','covariance'])
def test_flight_fault_never_recovers_after_landing(fault):
    s=bind();p,status=source(12.08)
    ros=mono=12.08;armed=True
    if fault=='state':armed=False
    if fault=='gap':mono+=.3
    if fault=='stale':ros+=.3
    if fault=='clock':ros=11.
    if fault=='identity':status.reset_counter=1
    if fault=='covariance':p.pose.covariance[0]=.3
    assert s.accept(p,status,ros,mono,pose_gid='pose',status_gid='status',ground=False,armed=armed) is None
    assert s.fault and s.count==1
    assert accept(s,12.12,ground=True) is None and s.count==1


def test_aligned_uncertain_sample_is_dropped_without_extending_deadline():
    s=FlightAlignedPoseStream(ID,[0.,0.,0.],0.,ID,quality_policy='bounded_gap')
    for i in range(52):accept(s,10+i*.04,ground=True)
    previous=s.alignment.last_stamp;received=s.last_receive
    p,status=source(12.08);p.pose.covariance[14]=.13
    assert s.accept(p,status,12.08,12.08,pose_gid='pose',status_gid='status',ground=False,armed=True) is None
    assert not s.fault and not s.alignment.fault and s.count==1 and s.dropped_uncertain==1
    assert s.alignment.last_stamp==previous and s.last_receive==received
    assert accept(s,12.12,armed=True) is not None and s.count==2


def test_continuous_uncertain_samples_cannot_keep_flight_alive():
    s=FlightAlignedPoseStream(ID,[0.,0.,0.],0.,ID,quality_policy='bounded_gap')
    for i in range(52):accept(s,10+i*.04,ground=True)
    for t in (12.08,12.12,12.16,12.20):
        p,status=source(t);p.pose.covariance[14]=.13
        assert s.accept(p,status,t,t,pose_gid='pose',status_gid='status',ground=False,armed=True) is None
    assert s.count==1 and s.last_receive==12.04
    assert s.check(12.25,12.25,ground=False,armed=True) is None and s.fault
    assert accept(s,12.28,ground=True) is None and s.count==1
