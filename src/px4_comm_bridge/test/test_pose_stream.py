import copy
import pytest
from uav_nav_interfaces.msg import VioStatus
from px4_comm_bridge.pose_stream import AlignedPoseStream
from test_pose_fusion import pose,ID


def source(t):
    p=pose(t)
    s=VioStatus(localization_session='source',calibration_id=ID,valid=True)
    s.header=p.header;s.sample_stamp=p.header.stamp
    return p,s


def feed(stream,t,**kw):
    p,s=source(t)
    return stream.accept(p,s,t,t,pose_gid='pose',status_gid='status',ground=True,**kw)


def bound():
    stream=AlignedPoseStream(ID,[0.,0.,0.],0.,ID)
    for i in range(52): feed(stream,10+i*.04)
    assert stream.bound and stream.count==1
    return stream


def test_original_sample_and_unknown_velocity_after_single_alignment():
    s=bound();result=feed(s,12.08)
    assert result is not None and result[0].header.stamp==source(12.08)[0].header.stamp
    assert result[1].timestamp_sample==12080000 and s.count==2
    assert s.alignment.binding['sample_stamp']==12.


@pytest.mark.parametrize('fault',['invalid','session','calibration','reset','pose_gid','status_gid','pair','replay','covariance','ground','stale','gap','clock'])
def test_bound_faults_latch_before_any_further_ev(fault):
    s=bound();p,status=source(12.08)
    kw=dict(pose_gid='pose',status_gid='status',ground=True);ros=mono=12.08
    if fault=='invalid':status.valid=False
    if fault=='session':status.localization_session='replacement'
    if fault=='calibration':status.calibration_id='b'*64
    if fault=='reset':status.reset_counter=1
    if fault in ('pose_gid','status_gid'):kw[fault]='new-writer'
    if fault=='pair':status.sample_stamp=copy.deepcopy(status.sample_stamp);status.sample_stamp.nanosec+=1
    if fault=='replay':p,status=source(12.04)
    if fault=='covariance':p.pose.covariance[0]=.3
    if fault=='ground':kw['ground']=False
    if fault=='stale':ros+=.3
    if fault=='gap':mono+=.3
    if fault=='clock':ros=11.
    assert s.accept(p,status,ros,mono,**kw) is None
    assert s.fault and s.count==1
    assert feed(s,12.12) is None and s.count==1


def test_warmup_invalid_resets_window_but_cannot_reuse_a_different_identity():
    s=AlignedPoseStream(ID,[0.,0.,0.],0.,ID)
    for i in range(20):feed(s,10+i*.04)
    s.reject('VIO_WRITER_COUNT')
    assert not s.window and not s.bound
    for i in range(52):feed(s,11+i*.04)
    assert s.bound


def test_moving_window_does_not_bind():
    s=AlignedPoseStream(ID,[0.,0.,0.],0.,ID)
    for i in range(80):
        p,status=source(10+i*.04);p.pose.pose.position.x+=.002*i
        s.accept(p,status,10+i*.04,10+i*.04,pose_gid='pose',status_gid='status',ground=True)
    assert not s.bound and s.count==0


def test_fresh_incoming_pair_is_checked_instead_of_old_sample_age():
    s=bound();p,status=source(12.08)
    # Previous sample is 205 ms old, but the new original sample is 165 ms old
    # and the monotonic receive gap is only 40 ms. Never re-stamp either one.
    result=s.accept(p,status,12.245,12.08,pose_gid='pose',status_gid='status',ground=True)
    assert result is not None and result[1].timestamp_sample==12080000 and not s.fault


def test_latched_watchdog_is_not_revived_by_a_later_fresh_pair():
    s=bound();assert s.check(12.245,12.08,ground=True) is None
    assert s.fault=='VIO_SAMPLE_STALE'
    assert feed(s,12.28) is None and s.count==1
