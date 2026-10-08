import math
import pytest
from nav_msgs.msg import Odometry
from px4_comm_bridge.vio_input import convert_vio, SourceContinuity
from px4_comm_bridge.converters import vehicle_odometry_to_ros


def odometry(stamp=10.):
    m = Odometry()
    m.header.frame_id, m.child_frame_id = 'odom', 'base_link'
    m.header.stamp.sec = int(stamp)
    m.header.stamp.nanosec = int(round((stamp-int(stamp))*1e9))
    m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = 1., 2., 3.
    m.pose.pose.orientation.w = 1.
    m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z = .1, .2, .3
    m.twist.twist.angular.x, m.twist.twist.angular.y, m.twist.twist.angular.z = .4, .5, .6
    for i in range(6):
        m.pose.covariance[7*i] = .01*(i+1)
        m.twist.covariance[7*i] = .01*(i+1)
    return m


@pytest.mark.parametrize('q', [(1.,0.,0.,0.), (.5,.5,.5,.5),
                              (math.cos(.3),0.,0.,math.sin(.3))])
def test_frame_conversion_round_trip(q):
    m = odometry()
    m.pose.pose.orientation.w, m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z = q
    result = convert_vio(m, 10.05, 7)
    assert list(result.position) == [2., 1., -3.]
    assert list(result.velocity) == pytest.approx([.1,-.2,-.3])
    assert list(result.angular_velocity) == pytest.approx([.4,-.5,-.6])
    assert result.reset_counter == 7 and result.timestamp_sample == 10000000
    restored = vehicle_odometry_to_ros(result)
    r = restored.pose.pose.orientation
    assert [r.w,r.x,r.y,r.z] == pytest.approx(q)
    assert [restored.twist.twist.linear.x,restored.twist.twist.linear.y,restored.twist.twist.linear.z] == pytest.approx([.1,.2,.3])
    assert [restored.pose.covariance[i*7] for i in range(6)] == pytest.approx([m.pose.covariance[i*7] for i in range(6)])


@pytest.mark.parametrize('change', [
    lambda m: setattr(m.header, 'frame_id', 'map'),
    lambda m: setattr(m, 'child_frame_id', 'camera_optical_frame'),
    lambda m: setattr(m.pose.pose.orientation, 'w', 0.),
    lambda m: setattr(m.pose.pose.position, 'x', math.nan),
    lambda m: setattr(m.twist.twist.linear, 'x', math.inf),
    lambda m: m.pose.covariance.__setitem__(0, 0.),
    lambda m: m.pose.covariance.__setitem__(0, -1.),
    lambda m: m.pose.covariance.__setitem__(0, .3),
    lambda m: m.pose.covariance.__setitem__(1, .1),
    lambda m: m.twist.covariance.__setitem__(0, 0.),
])
def test_invalid_source_rejected(change):
    m = odometry(); change(m)
    with pytest.raises(ValueError): convert_vio(m, 10.05)


@pytest.mark.parametrize('now', [9.9,10.21])
def test_source_age(now):
    with pytest.raises(ValueError): convert_vio(odometry(), now)


@pytest.mark.parametrize('failure', ['publisher','replay','gap','jump','attitude'])
def test_discontinuity_latches(failure):
    gate = SourceContinuity()
    gate.accept(odometry(), 'writer-a')
    m = odometry(10.05)
    publisher = 'writer-b' if failure == 'publisher' else 'writer-a'
    if failure == 'replay': m = odometry()
    if failure == 'gap': m = odometry(10.3)
    if failure == 'jump': m.pose.pose.position.x += 1.
    if failure == 'attitude':
        m.pose.pose.orientation.w = math.cos(.5)
        m.pose.pose.orientation.z = math.sin(.5)
    with pytest.raises(ValueError): gate.accept(m, publisher)
    with pytest.raises(ValueError): gate.accept(odometry(10.1), 'writer-a')


def test_quaternion_sign_flip_is_same_attitude():
    gate = SourceContinuity()
    gate.accept(odometry(), 'writer-a')
    m = odometry(10.05)
    m.pose.pose.orientation.w = -1.
    gate.accept(m, 'writer-a')
    assert not gate.fault
