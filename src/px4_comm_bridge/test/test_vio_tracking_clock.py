"""Exercise the normalizer tracking gate at exact original ROS age bounds."""
import time
from types import SimpleNamespace
import pytest
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from px4_comm_bridge.cuvslam_pose_node import CuvslamPose


@pytest.mark.parametrize('age_ns,accepted',[(200_000_000,True),(200_000_001,False),
    (-50_000_000,True),(-50_000_001,False)])
def test_tracking_gate_checks_original_nanoseconds(age_ns,accepted):
    tracking=VisualSlamStatus();tracking.vo_state=1
    tracking.header.stamp.sec=37;tracking.header.stamp.nanosec=400_000_000
    now_ns=37_400_000_000+age_ns
    node=SimpleNamespace(last_clock=None,verified_parameters={},fault='',bound=True,
        tracking=tracking,tracking_receive=time.monotonic(),tracking_gid='gid',tracking_topic='/tracking')
    node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=now_ns))
    node.publisher=lambda topic:'gid'
    node.get_publishers_info_by_topic=lambda topic:[object()]
    node.get_logger=lambda:SimpleNamespace(warning=lambda text:None)
    if accepted:CuvslamPose.validate_tracking(node)
    else:
        with pytest.raises(ValueError,match='VIO_TRACKING_INVALID'):CuvslamPose.validate_tracking(node)
