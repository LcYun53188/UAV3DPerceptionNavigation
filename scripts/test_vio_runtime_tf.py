"""Exercise the actual ROS static broadcaster and receive its latched transforms."""
import math
from pathlib import Path
import sys
import time
import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
sys.path.insert(0,str(Path(__file__).resolve().parent))
from px4_vio_sensor_audit import SensorAudit
from assess_vio_motion_evidence import rotation


def test_actual_static_tf_contains_rendered_pitch():
    context=Context()
    rclpy.init(context=context,domain_id=89)
    node=rclpy.create_node('pitched_mount_regression',context=context)
    executor=SingleThreadedExecutor(context=context)
    executor.add_node(node)
    theta=math.radians(15)
    frames={'vio_left_optical':dict(position=[.12,.0375,.242],rpy=[-math.pi/2-theta,0,-math.pi/2]),
            'vio_right_optical':dict(position=[.12,-.0375,.242],rpy=[-math.pi/2-theta,0,-math.pi/2]),
            'vio_imu':dict(position=[.12,0.,.242],rpy=[0,0,0])}
    try:
        audit=SensorAudit(node,{},frames)
        deadline=time.monotonic()+5
        while len(audit.observed_mounts)<3 and time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.1)
        assert set(audit.observed_mounts)==set(frames)
        ry=np.array([[math.cos(theta),0,math.sin(theta)],[0,1,0],[-math.sin(theta),0,math.cos(theta)]])
        optical=np.array([[0,0,1],[-1,0,0],[0,-1,0]])
        for name,receipt in audit.observed_mounts.items():
            assert receipt['parent']=='base_link'
            assert receipt['position']==frames[name]['position']
            assert np.allclose(rotation(receipt['quaternion']),np.eye(3) if name=='vio_imu' else ry@optical)
    finally:
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        context.shutdown()
