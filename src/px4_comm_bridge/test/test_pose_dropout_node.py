from types import SimpleNamespace
import time
from geometry_msgs.msg import PoseWithCovarianceStamped
from px4_comm_bridge.cuvslam_pose_node import CuvslamPose


def uncertain(t=10.08):
    p=PoseWithCovarianceStamped();p.header.frame_id='odom'
    p.header.stamp.sec=int(t);p.header.stamp.nanosec=round((t-int(t))*1e9)
    p.pose.pose.orientation.w=1.
    for i in range(6):p.pose.covariance[i*7]=1.
    return p


def fixture():
    node=SimpleNamespace(fault='',bound=True,receive=time.monotonic()-.05,sample=uncertain(10.).header.stamp,
        quality_policy='bounded_gap',dropped_samples=0,emitted=[],status=0)
    node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10100000000))
    node.validate_tracking=lambda *args:None
    node.trace=lambda *args,**kwargs:None
    node.get_logger=lambda:SimpleNamespace(warning=lambda *args:None)
    node.publish_status=lambda:None
    node.reject=lambda reason:setattr(node,'fault',reason)
    node.pose_pub=SimpleNamespace(publish=node.emitted.append)
    return node


def test_rejected_sample_never_refreshes_last_good_or_publishes_ev_input():
    node=fixture();previous=(node.sample,node.receive)
    CuvslamPose.accept_pose(node,uncertain(),None)
    assert not node.fault and node.dropped_samples==1 and not node.emitted
    assert (node.sample,node.receive)==previous


def test_repeated_uncertainty_cannot_extend_existing_deadline():
    node=fixture();CuvslamPose.accept_pose(node,uncertain(),None)
    node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10201000000))
    CuvslamPose.accept_pose(node,uncertain(10.20),None)
    assert node.fault=='VIO_UNCERTAINTY_INVALID' and not node.emitted


def test_strict_policy_still_retires_on_first_uncertain_sample():
    node=fixture();node.quality_policy='strict'
    CuvslamPose.accept_pose(node,uncertain(),None)
    assert node.fault=='VIO_UNCERTAINTY_INVALID' and node.dropped_samples==0


def test_reordering_after_a_rejected_sample_still_retires_source():
    node=fixture();node.last_input_stamp=10.08
    node.continuity=SimpleNamespace(publisher='gid');node.publisher=lambda topic:'gid'
    node.pending={};node.pose_topic='/sdk/pose'
    CuvslamPose.on_pose(node,uncertain(10.04),None)
    assert node.fault=='VIO_TIME_DISCONTINUITY' and not node.emitted
