"""Estimator covariance semantics: world rotation, cross terms and invalid sources."""
import copy
import math
import numpy as np
import pytest
from geometry_msgs.msg import PoseWithCovarianceStamped
from px4_comm_bridge.cuvslam_pose import normalize_pose, fresh_stamp


def sample():
    m = PoseWithCovarianceStamped()
    m.header.frame_id = 'odom'
    m.header.stamp.sec = 10
    m.pose.pose.orientation.w = 1.
    m.pose.pose.position.x = 2.
    m.pose.covariance = np.diag([.01,.02,.03,.04,.05,.06]).ravel().tolist()
    return m


def test_identity_pose_is_not_shifted_and_source_not_mutated():
    m = sample(); original = copy.deepcopy(m)
    result = normalize_pose(m,10.1)
    assert result == m and m == original
    assert result is not m


def test_body_right_perturbation_rotates_position_orientation_and_cross_terms():
    m = sample()
    m.pose.pose.orientation.w = m.pose.pose.orientation.z = math.sqrt(.5)
    c = np.asarray(m.pose.covariance).reshape(6,6)
    c[0,3] = c[3,0] = .003
    m.pose.covariance = c.ravel().tolist()
    world = np.asarray(normalize_pose(m,10.).pose.covariance).reshape(6,6)
    assert np.diag(world) == pytest.approx([.02,.01,.03,.05,.04,.06])
    assert world[1,4] == pytest.approx(.003)
    assert world[0,3] == pytest.approx(0.,abs=1e-14)
    assert np.linalg.eigvalsh(world) == pytest.approx(np.linalg.eigvalsh(c))
    assert np.trace(world) == pytest.approx(np.trace(c))


def test_quaternion_sign_and_small_normalization_error():
    a = sample(); b = copy.deepcopy(a)
    b.pose.pose.orientation.w = -1.001
    result = normalize_pose(b,10.)
    assert result.pose.covariance == pytest.approx(a.pose.covariance)
    assert result.pose.pose.orientation.w == -1.


@pytest.mark.parametrize('fault',['frame','old','future','zero_stamp','zero_cov','negative_cov',
                                'large_cov','asymmetric','nonfinite_cov','nonfinite_pose','quaternion'])
def test_unknown_or_invalid_measurement_not_given_confidence(fault):
    m = sample()
    if fault == 'frame': m.header.frame_id = 'map'
    if fault == 'old': m.header.stamp.sec = 9
    if fault == 'future': m.header.stamp.nanosec = 100000000
    if fault == 'zero_stamp': m.header.stamp.sec = 0
    if fault == 'zero_cov': m.pose.covariance[0] = 0.
    if fault == 'negative_cov': m.pose.covariance[0] = -.01
    if fault == 'large_cov': m.pose.covariance[0] = .251
    if fault == 'asymmetric': m.pose.covariance[1] = .01
    if fault == 'nonfinite_cov': m.pose.covariance[0] = math.nan
    if fault == 'nonfinite_pose': m.pose.pose.position.x = math.inf
    if fault == 'quaternion': m.pose.pose.orientation.w = 0.
    with pytest.raises(ValueError): normalize_pose(m,10.)


@pytest.mark.parametrize('now,stamp',[(34.2,34.),(37.6,37.4)])
def test_exact_age_limit_is_not_rejected_by_float_subtraction(now,stamp):
    m=sample();m.header.stamp.sec=int(stamp);m.header.stamp.nanosec=round((stamp-int(stamp))*1e9)
    assert now-stamp>.2  # Reproduce the observed old comparison failure.
    assert normalize_pose(m,now)==m


@pytest.mark.parametrize('age_ns,expected',[(200_000_000,True),(200_000_001,False),
    (-50_000_000,True),(-50_000_001,False)])
def test_nanosecond_bounds_remain_exact_at_epoch_time(age_ns,expected):
    m=sample();m.header.stamp.sec=1_791_540_000;m.header.stamp.nanosec=1
    now_ns=m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec+age_ns
    assert fresh_stamp(now_ns,m.header.stamp)==expected
    if expected:assert normalize_pose(m,now_ns/1e9,now_ns=now_ns)==m
    else:
        with pytest.raises(ValueError,match='VIO_SAMPLE_STALE'):
            normalize_pose(m,now_ns/1e9,now_ns=now_ns)
