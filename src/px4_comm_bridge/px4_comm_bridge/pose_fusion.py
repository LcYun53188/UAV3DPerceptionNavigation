"""Fixed launch alignment and pose-only PX4 conversion. No publisher or estimator feedback.

Input covariance is already SDK-normalized world-fixed ROS covariance. Velocity
is unknown. The configured launch anchor is an explicit external assumption.
"""
import copy
import hashlib
import json
import math
import re
import numpy as np
from px4_msgs.msg import VehicleOdometry
from .cuvslam_pose import rotation
from .vio_input import covariance,stamp_s

FRAME = 'px4_local_enu'
PROFILE = 'aligned_pose_v1'


def checked_pose(m,now,frame):
    t=stamp_s(m.header.stamp)
    if m.header.frame_id!=frame: raise ValueError('VIO_FRAME_INVALID')
    if not math.isfinite(now) or t<=0 or not -.05<=now-t<=.2: raise ValueError('VIO_SAMPLE_STALE')
    p=m.pose.pose.position
    if not all(math.isfinite(v) for v in (p.x,p.y,p.z)): raise ValueError('VIO_NONFINITE')
    r=rotation(m.pose.pose.orientation)
    c=covariance(m.pose.covariance,'POSE')
    if np.any(np.diag(c)<=0) or np.any(np.diag(c)>.25): raise ValueError('VIO_UNCERTAINTY_INVALID')
    return np.array([p.x,p.y,p.z]),r,c


class PoseAlignment:
    """Bind once, on a stationary disarmed launch, and retire on subsequent faults."""
    def __init__(self,position_enu,yaw_enu,anchor_config_id):
        self.anchor=np.array(position_enu,dtype=float,copy=True)
        if self.anchor.shape!=(3,) or not np.isfinite(self.anchor).all() or not math.isfinite(yaw_enu):
            raise ValueError('VIO_ANCHOR_INVALID')
        if not re.fullmatch('[0-9a-f]{64}',anchor_config_id): raise ValueError('VIO_ANCHOR_UNREVIEWED')
        self.yaw,self.anchor_id=yaw_enu,anchor_config_id
        self.binding=None
        self.fault=''

    def bind(self,samples,now,*,source_session,calibration_id,reset_counter=0,disarmed,landed):
        if self.binding is not None: raise ValueError('VIO_ALIGNMENT_ALREADY_BOUND')
        if not disarmed or not landed: raise ValueError('VIO_ALIGNMENT_REQUIRES_GROUND')
        if not source_session or not re.fullmatch('[0-9a-f]{64}',calibration_id): raise ValueError('VIO_SOURCE_IDENTITY_INVALID')
        if not samples or not 0<=reset_counter<=255: raise ValueError('VIO_ALIGNMENT_WINDOW_MISSING')
        ts=np.array([stamp_s(m.header.stamp) for m in samples])
        if len(ts)<40 or ts[-1]-ts[0]<2 or np.any(np.diff(ts)<=0) or np.max(np.diff(ts))>.081:
            raise ValueError('VIO_ALIGNMENT_WINDOW_INVALID')
        values=[checked_pose(m,stamp_s(m.header.stamp),'odom') for m in samples]
        checked_pose(samples[-1],now,'odom')
        if max(np.linalg.norm(p-values[0][0]) for p,_,_ in values)>.03:
            raise ValueError('VIO_ALIGNMENT_NOT_STATIONARY')
        if any(math.acos(float(np.clip((np.trace(values[0][1].T@r)-1)/2,-1,1)))>.03 for _,r,_ in values):
            raise ValueError('VIO_ALIGNMENT_NOT_STATIONARY')
        p,r,initial_covariance=values[-1]
        self.initial_covariance=initial_covariance.copy()
        self.initial_position=p
        self.initial_yaw_jacobian=euler_jacobian(r)[2]
        if math.acos(float(np.clip(r[2,2],-1,1)))>math.radians(10): raise ValueError('VIO_ALIGNMENT_TILT_INVALID')
        delta=self.yaw-math.atan2(r[1,0],r[0,0])
        c,s=math.cos(delta),math.sin(delta)
        self.r=np.array([[c,-s,0],[s,c,0],[0,0,1.]])
        self.offset=self.anchor-self.r@p
        self.delta=delta
        self.identity=(source_session,calibration_id,reset_counter)
        record=dict(schema=1,profile=PROFILE,anchor_config_id=self.anchor_id,source_session=source_session,
            calibration_id=calibration_id,reset_counter=reset_counter,sample_stamp=float(ts[-1]),
            position_enu=self.anchor.tolist(),yaw_enu=self.yaw,rotation=self.r.tolist(),translation=self.offset.tolist(),
            covariance_policy='unknown_temporal_correlation_bound_v1',
            initial_covariance=self.initial_covariance.tolist(),initial_yaw_jacobian=self.initial_yaw_jacobian.tolist())
        self.alignment_id=hashlib.sha256(json.dumps(record,sort_keys=True,separators=(',',':')).encode()).hexdigest()
        self.binding=dict(record,alignment_id=self.alignment_id)
        self.last_stamp=float(ts[-1])
        return copy.deepcopy(self.binding)

    def apply(self,m,now,*,source_session,calibration_id,reset_counter=0):
        if self.fault: raise ValueError(self.fault)
        if self.binding is None: raise ValueError('VIO_ALIGNMENT_MISSING')
        try:
            if (source_session,calibration_id,reset_counter)!=self.identity: raise ValueError('VIO_ALIGNMENT_IDENTITY_CHANGED')
            p,_,c=checked_pose(m,now,'odom')
            if stamp_s(m.header.stamp)<=self.last_stamp: raise ValueError('VIO_TIME_DISCONTINUITY')
            if stamp_s(m.header.stamp)-self.last_stamp>.2: raise ValueError('VIO_TIME_DISCONTINUITY')
            a=np.zeros((6,6));a[:3,:3]=a[3:,3:]=self.r
            out=copy.deepcopy(m);out.header.frame_id=FRAME
            out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z=self.r@p+self.offset
            q=out.pose.pose.orientation
            n=math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
            x,y,z,w=q.x/n,q.y/n,q.z/n,q.w/n
            co,si=math.cos(self.delta/2),math.sin(self.delta/2)
            q.x,q.y,q.z,q.w=co*x-si*y,co*y+si*x,co*z+si*w,co*w-si*z
            # Current and anchor VIO errors are temporally correlated, but the
            # SDK exposes no joint covariance. For any cross covariance,
            # Cov(X+Y) <= 2*(Cov(X)+Cov(Y)) in PSD order. Never assume independence.
            displacement=self.r@(p-self.initial_position)
            yaw_position=np.cross([0.,0.,1.],displacement)
            initial=np.zeros((6,6));initial[:3,:3]=-self.r
            initial[:3,3:]=-np.outer(yaw_position,self.initial_yaw_jacobian)
            initial[3:,3:]=-np.outer([0.,0.,1.],self.initial_yaw_jacobian)
            propagated=2*(a@c@a.T+initial@self.initial_covariance@initial.T)
            out.pose.covariance=propagated.ravel().tolist()
            checked_pose(out,now,FRAME)
            self.last_stamp=stamp_s(m.header.stamp)
            return out
        except ValueError as exc:
            self.fault=str(exc)
            raise


def euler_jacobian(r):
    """ZYX angles w.r.t. a world-fixed small rotation. PX4 EV fuses Euler yaw."""
    yaw=math.atan2(r[1,0],r[0,0]); cp=math.hypot(r[0,0],r[1,0])
    if cp<.5: raise ValueError('VIO_EULER_SINGULARITY')
    tp=-r[2,0]/cp;c,s=math.cos(yaw),math.sin(yaw)
    return np.array([[c/cp,s/cp,0],[-s,c,0],[c*tp,s*tp,1]])


def convert_aligned_pose(m,now,reset_counter=0):
    p,r,c=checked_pose(m,now,FRAME)
    if not 0<=reset_counter<=255: raise ValueError('VIO_RESET_COUNTER_INVALID')
    ned=np.array([[0,1,0],[1,0,0],[0,0,-1.]])
    frd=np.diag([1.,-1.,-1.])
    j=euler_jacobian(ned@r@frd)
    position_cov=ned@c[:3,:3]@ned.T
    angles_cov=j@ned@c[3:,3:]@ned.T@j.T
    if np.any(np.diag(angles_cov)<=0) or np.any(np.diag(angles_cov)>.25): raise ValueError('VIO_UNCERTAINTY_INVALID')
    q=m.pose.pose.orientation;n=math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);s=math.sqrt(.5)/n
    nan=[float('nan')]*3
    return VehicleOdometry(timestamp=int(now*1e6),timestamp_sample=int(round(stamp_s(m.header.stamp)*1e6)),
        pose_frame=VehicleOdometry.POSE_FRAME_NED,position=(ned@p).tolist(),
        q=[s*(q.w+q.z),s*(q.x+q.y),s*(q.x-q.y),s*(q.w-q.z)],
        velocity_frame=VehicleOdometry.VELOCITY_FRAME_UNKNOWN,velocity=nan,angular_velocity=nan,
        velocity_variance=nan,position_variance=np.diag(position_cov).tolist(),
        orientation_variance=np.diag(angles_cov).tolist(),reset_counter=reset_counter,quality=100)
