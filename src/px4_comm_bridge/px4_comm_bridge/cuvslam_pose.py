"""cuVSLAM 15 right-tangent base-link covariance -> ROS world fixed axes.

The SDK rig is base_link because the upstream node supplies base_link camera
extrinsics. Its published SDK covariance already changes OpenCV to FLU axes,
but remains a right perturbation: T_random = T_mean exp(u). It is not the
sliding-window covariance in the upstream Odometry topic. No velocity inferred.
"""
import copy
import math
import numpy as np
from .vio_input import covariance, stamp_s

CONTRACT = 'cuvslam15_right_tangent_base_link_v1'
SOURCE_PARAMETERS = dict(base_frame='base_link',odom_frame='odom',tracking_mode=1,
    enable_localization_n_mapping=False,enable_ground_constraint_in_odometry=False,
    override_publishing_stamp=False)


def rotation(q):
    x,y,z,w = q.x,q.y,q.z,q.w
    if not all(math.isfinite(v) for v in (x,y,z,w)) or abs(x*x+y*y+z*z+w*w-1.) > .01:
        raise ValueError('VIO_QUATERNION_INVALID')
    n = math.sqrt(x*x+y*y+z*z+w*w)
    x,y,z,w = (v/n for v in (x,y,z,w))
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def normalize_pose(message, now_s):
    if message.header.frame_id != 'odom':
        raise ValueError('VIO_FRAME_INVALID')
    t = stamp_s(message.header.stamp)
    if t <= 0 or not -.05 <= now_s-t <= .2:
        raise ValueError('VIO_SAMPLE_STALE')
    p = message.pose.pose.position
    if not all(math.isfinite(v) for v in (p.x,p.y,p.z)):
        raise ValueError('VIO_NONFINITE')
    r = rotation(message.pose.pose.orientation)
    body = covariance(message.pose.covariance,'SDK_POSE')
    # Remove only float-rounding antisymmetry, not negative eigenvalues.
    body = (body+body.T)/2
    transform = np.zeros((6,6))
    transform[:3,:3] = transform[3:,3:] = r
    world = transform@body@transform.T
    if np.any(np.diag(world) <= 0) or np.any(np.diag(world) > .25):
        raise ValueError('VIO_UNCERTAINTY_INVALID')
    result = copy.deepcopy(message)
    q = result.pose.pose.orientation
    n = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
    q.x,q.y,q.z,q.w = q.x/n,q.y/n,q.z/n,q.w/n
    result.pose.covariance = world.ravel().tolist()
    return result
