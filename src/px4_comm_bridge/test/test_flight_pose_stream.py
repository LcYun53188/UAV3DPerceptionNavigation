"""Flight continuity policy only; no simulator or flight authorization evidence."""
import pytest
from px4_comm_bridge.pose_stream import AlignedPoseStream, FlightAlignedPoseStream
from test_pose_stream import source
from test_pose_fusion import ID


def accept(stream,t,*,ground=False,airborne=False):
    p,s=source(t)
    return stream.accept(p,s,t,t,pose_gid='pose',status_gid='status',ground=ground,airborne=airborne)


def bind():
    s=FlightAlignedPoseStream(ID,[0.,0.,0.],0.,ID)
    for i in range(52):accept(s,10+i*.04,ground=True)
    assert s.bound and s.count==1
    return s


def test_airborne_cannot_initialize_or_reuse_ground_warmup():
    s=FlightAlignedPoseStream(ID,[0.,0.,0.],0.,ID)
    for i in range(50):accept(s,10+i*.04,ground=True)
    assert s.window and not s.bound
    for i in range(60):assert accept(s,12+i*.04,airborne=True) is None
    assert not s.window and not s.bound and s.count==0


def test_ground_bound_alignment_continues_in_air_and_after_landing_once():
    s=bind();binding=s.alignment.binding
    for i in range(20):
        t=12.08+i*.04
        output=accept(s,t,airborne=True)
        assert output is not None and output[1].timestamp_sample==round(t*1e6)
        assert s.alignment.binding is binding
    assert accept(s,12.88,ground=True) is not None
    assert s.alignment.binding is binding and not s.fault


def test_default_disarmed_audit_cannot_continue_airborne():
    s=AlignedPoseStream(ID,[0.,0.,0.],0.,ID)
    for i in range(52):accept(s,10+i*.04,ground=True)
    assert accept(s,12.08,airborne=True) is None
    assert s.fault=='VIO_ALIGNMENT_REQUIRES_GROUND'


@pytest.mark.parametrize('fault',['state','gap','stale','clock','identity','covariance'])
def test_flight_fault_never_recovers_after_landing(fault):
    s=bind();p,status=source(12.08)
    ros=mono=12.08;airborne=True
    if fault=='state':airborne=False
    if fault=='gap':mono+=.3
    if fault=='stale':ros+=.3
    if fault=='clock':ros=11.
    if fault=='identity':status.reset_counter=1
    if fault=='covariance':p.pose.covariance[0]=.3
    assert s.accept(p,status,ros,mono,pose_gid='pose',status_gid='status',ground=False,airborne=airborne) is None
    assert s.fault and s.count==1
    assert accept(s,12.12,ground=True) is None and s.count==1
