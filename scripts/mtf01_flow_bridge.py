#!/usr/bin/env python3
"""Image-derived flow for a fixed +90 degree pitch camera, no truth input."""
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from px4_msgs.msg import SensorOpticalFlow


def track(previous, current, fov_deg=42):
    """Return PX4 body-axis integrated flow, quality; poor texture returns zero."""
    if previous.shape!=current.shape or previous.ndim!=2:
        raise ValueError('Expected equal grayscale images')
    corners = cv2.goodFeaturesToTrack(previous, maxCorners=40, qualityLevel=.03, minDistance=5, blockSize=3)
    if corners is None or len(corners)<6:
        return [0.,0.],0
    moved, valid, errors = cv2.calcOpticalFlowPyrLK(previous,current,corners,None,winSize=(15,15),maxLevel=2)
    if moved is None or valid is None:
        return [0.,0.],0
    returned, back_valid, _ = cv2.calcOpticalFlowPyrLK(current,previous,moved,None,winSize=(15,15),maxLevel=2)
    if returned is None or back_valid is None:
        return [0.,0.],0
    good = (valid.ravel()>0) & (back_valid.ravel()>0) & (errors.ravel()<20)
    good &= np.linalg.norm(returned-corners,axis=2).ravel()<.5
    delta=(moved-corners).reshape(-1,2)
    good &= np.isfinite(delta).all(axis=1)
    if np.count_nonzero(good)<6:
        return [0.,0.],0
    median=np.median(delta[good],axis=0)
    good &= np.linalg.norm(delta-median,axis=1)<1.5
    count=np.count_nonzero(good)
    if count<6:
        return [0.,0.],0
    # A median of feature displacements confuses image rotation (yaw) and
    # expansion (vertical motion) with translation when corners are unevenly
    # distributed. Fit the image similarity transform, then measure motion at
    # the optical principal point. Roll/pitch motion remains in this output
    # for PX4's IMU compensation; optical-axis rotation has zero centre flow.
    transform,inliers=cv2.estimateAffinePartial2D(corners.reshape(-1,2)[good],
        moved.reshape(-1,2)[good],method=cv2.RANSAC,ransacReprojThreshold=.5,
        maxIters=100,confidence=.99,refineIters=10)
    if transform is None or inliers is None or not np.isfinite(transform).all():
        return [0.,0.],0
    count=int(np.count_nonzero(inliers))
    if count<6:return [0.,0.],0
    centre=np.array([current.shape[1]/2,current.shape[0]/2])
    displacement=transform[:,:2]@centre+transform[:,2]-centre
    focal=current.shape[1]/(2*math.tan(math.radians(fov_deg)/2))
    # Optical +X is body right; optical +Y is body backward. EKF2.cpp
    # negates uORB pixel_flow before predicting [v_right/range,-v_forward/range].
    flow=[math.atan(float(x)/focal) for x in displacement]
    quality=max(1,min(255,round(255*count/len(corners))))
    return flow,quality


class FlowBridge(Node):
    def __init__(self):
        super().__init__('mtf01_flow_bridge')
        self.last_image=None
        self.last_stamp=0
        self.publisher=self.create_publisher(SensorOpticalFlow,'/px4_7/fmu/in/sensor_optical_flow',qos_profile_sensor_data)
        self.subscription=self.create_subscription(Image,'/mtf01/flow/image',self.receive,qos_profile_sensor_data)

    def receive(self,image):
        stamp=image.header.stamp.sec*1000000+image.header.stamp.nanosec//1000
        if stamp<=self.last_stamp or image.encoding not in ('rgb8','bgr8','mono8') or image.width!=100 or image.height!=100:
            return
        channels=1 if image.encoding=='mono8' else 3
        if image.step<image.width*channels or len(image.data)!=image.step*image.height:
            return
        pixels=np.frombuffer(image.data,dtype=np.uint8).reshape(image.height,image.step)[:,:image.width*channels]
        gray=pixels.copy() if channels==1 else cv2.cvtColor(pixels.reshape(image.height,image.width,3),cv2.COLOR_RGB2GRAY if image.encoding=='rgb8' else cv2.COLOR_BGR2GRAY)
        if self.last_image is not None and 0<stamp-self.last_stamp<=50000:
            flow,quality=track(self.last_image,gray)
            msg=SensorOpticalFlow()
            msg.timestamp=stamp
            msg.timestamp_sample=stamp
            msg.device_id=1442052  # SIMULATION flow type, address 1
            msg.pixel_flow=flow
            msg.integration_timespan_us=stamp-self.last_stamp
            msg.quality=quality
            msg.delta_angle=[float('nan')]*3
            msg.delta_angle_available=False
            msg.distance_available=False
            msg.max_flow_rate=7.
            msg.min_ground_distance=.08
            msg.max_ground_distance=8.
            msg.mode=SensorOpticalFlow.MODE_UNKNOWN
            self.publisher.publish(msg)
        self.last_image=gray
        self.last_stamp=stamp


def main():
    rclpy.init()
    node=FlowBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    main()
