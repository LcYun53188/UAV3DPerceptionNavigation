"""Validated ROS ENU/FLU VIO -> PX4 NED/FRD; no command or control output."""
import math
import numpy as np
from px4_msgs.msg import VehicleOdometry


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec / 1e9


def covariance(values, name):
    matrix = np.asarray(values, dtype=float).reshape(6, 6)
    if (not np.isfinite(matrix).all() or not np.allclose(matrix, matrix.T, atol=1e-8)
            or np.linalg.eigvalsh(matrix).min() < -1e-8):
        raise ValueError(name + '_COVARIANCE_INVALID')
    return matrix


def convert_vio(message, now_s, reset_counter=0):
    """Only shared-clock SITL timestamps. Hardware requires verified time mapping."""
    if message.header.frame_id != 'odom' or message.child_frame_id != 'base_link':
        raise ValueError('VIO_FRAME_INVALID')
    sample = stamp_s(message.header.stamp)
    if sample <= 0 or not -.05 <= now_s - sample <= .2:
        raise ValueError('VIO_SAMPLE_STALE')
    p, q = message.pose.pose.position, message.pose.pose.orientation
    v, a = message.twist.twist.linear, message.twist.twist.angular
    values = [p.x, p.y, p.z, q.w, q.x, q.y, q.z, v.x, v.y, v.z, a.x, a.y, a.z]
    if not all(math.isfinite(float(x)) for x in values):
        raise ValueError('VIO_NONFINITE')
    norm = math.sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z)
    if abs(norm - 1.) > .01:
        raise ValueError('VIO_QUATERNION_INVALID')
    pose = covariance(message.pose.covariance, 'POSE')
    twist = covariance(message.twist.covariance, 'TWIST')
    pd, td = np.diag(pose), np.diag(twist)
    # Zero/unknown uncertainties must not become perfect EKF measurements.
    if (np.any(pd <= 0) or np.any(pd[:3] > .25) or np.any(pd[3:] > .25)
            or np.any(td[:3] <= 0) or np.any(td[:3] > .25)):
        raise ValueError('VIO_UNCERTAINTY_INVALID')
    s = math.sqrt(.5) / norm
    result = VehicleOdometry(timestamp=int(now_s*1e6), timestamp_sample=int(round(sample*1e6)),
        pose_frame=VehicleOdometry.POSE_FRAME_NED,
        position=[p.y, p.x, -p.z], q=[s*(q.w+q.z), s*(q.x+q.y), s*(q.x-q.y), s*(q.w-q.z)],
        velocity_frame=VehicleOdometry.VELOCITY_FRAME_BODY_FRD,
        velocity=[v.x, -v.y, -v.z], angular_velocity=[a.x, -a.y, -a.z],
        position_variance=[float(pd[1]), float(pd[0]), float(pd[2])],
        orientation_variance=[float(pd[4]), float(pd[3]), float(pd[5])],
        velocity_variance=[float(td[0]), float(td[1]), float(td[2])],
        reset_counter=reset_counter, quality=100)
    return result


class SourceContinuity:
    """Never renew stale input, never silently reseed after a restart/jump."""
    def __init__(self):
        self.last_stamp = None
        self.last_position = None
        self.last_quaternion = None
        self.publisher = None
        self.fault = ''

    def accept(self, odometry, publisher):
        if self.fault:
            raise ValueError(self.fault)
        stamp = stamp_s(odometry.header.stamp)
        p = odometry.pose.pose.position
        position = np.array([p.x, p.y, p.z])
        q = odometry.pose.pose.orientation
        quaternion = np.array([q.w,q.x,q.y,q.z])
        quaternion = quaternion / np.linalg.norm(quaternion)
        if not publisher or (self.publisher is not None and publisher != self.publisher):
            self.fault = 'VIO_PUBLISHER_CHANGED'
        elif self.last_stamp is not None:
            dt = stamp - self.last_stamp
            if dt <= 0 or dt > .2:
                self.fault = 'VIO_TIME_DISCONTINUITY'
            elif np.linalg.norm(position-self.last_position) > .05 + 3.*dt:
                self.fault = 'VIO_POSE_DISCONTINUITY'
            elif 2*math.acos(float(np.clip(abs(np.dot(quaternion,self.last_quaternion)),0,1))) > .1+3.*dt:
                self.fault = 'VIO_ATTITUDE_DISCONTINUITY'
        if self.fault:
            raise ValueError(self.fault)
        self.publisher = publisher
        self.last_stamp, self.last_position = stamp, position
        self.last_quaternion = quaternion
