"""Explicit frame/session binding for isolated shadow planning, without controls."""
from copy import deepcopy
import hashlib
import json
import math

import numpy as np
from scipy.spatial.transform import Rotation

from .core import grid_from_message, spline, validate_trajectory


def stamp(value):
    return value.sec + value.nanosec / 1e9


def xyz(value):
    return np.array([value.x, value.y, value.z], dtype=float)


def transform_parts(transform):
    translation = xyz(transform.translation)
    q = transform.rotation
    quat = np.array([q.x, q.y, q.z, q.w], dtype=float)
    if not np.isfinite(translation).all() or not np.isfinite(quat).all() or np.linalg.norm(quat) < 1e-9:
        raise ValueError('INVALID_ALIGNMENT_TRANSFORM')
    rotation = Rotation.from_quat(quat)
    return translation, rotation


def transform_odometry(message, transform):
    """Transform world pose/covariance to map; ROS twist stays in child FLU."""
    if message.header.frame_id != 'odom' or message.child_frame_id != 'base_link':
        raise ValueError('INVALID_ODOMETRY_FRAME')
    translation, rotation = transform_parts(transform)
    pose = message.pose.pose
    q = pose.orientation
    values = np.r_[xyz(pose.position), [q.x,q.y,q.z,q.w],
                   xyz(message.twist.twist.linear), xyz(message.twist.twist.angular)]
    covariance = np.array(message.pose.covariance).reshape(6,6)
    twist_covariance = np.array(message.twist.covariance).reshape(6,6)
    if not np.isfinite(values).all() or np.linalg.norm(values[3:7]) < 1e-9:
        raise ValueError('INVALID_ODOMETRY_POSE')
    for matrix in (covariance, twist_covariance):
        if (not np.isfinite(matrix).all() or not np.allclose(matrix, matrix.T, atol=1e-8) or
                np.linalg.eigvalsh(matrix).min() < -1e-8):
            raise ValueError('INVALID_ODOMETRY_COVARIANCE')
    result = deepcopy(message)
    result.header.frame_id = 'map'
    point = translation + rotation.apply(xyz(pose.position))
    result.pose.pose.position.x, result.pose.pose.position.y, result.pose.pose.position.z = map(float,point)
    quat = (rotation * Rotation.from_quat(values[3:7])).as_quat()
    target = result.pose.pose.orientation
    target.x,target.y,target.z,target.w = map(float,quat)
    basis = np.zeros((6,6));basis[:3,:3]=basis[3:,3:]=rotation.as_matrix()
    result.pose.covariance = (basis@covariance@basis.T).ravel().tolist()
    return result


class PlanningGate:
    def __init__(self, radius=.3, limits=(.5,1.,2.)):
        if not math.isfinite(radius) or radius <= 0 or len(limits)!=3 or not all(math.isfinite(v) and v>0 for v in limits):
            raise ValueError('INVALID_PLANNING_LIMITS')
        self.radius,self.limits=radius,limits
        self.inputs={};self.grid=None;self.alignment_signature=None
        self.alignment_generation=-1;self.invalidated_alignment_generation=-1;self.alignment_conflict=False
        self.last_goal_stamp=(-1,-1);self.goal=None;self.context_id='';self.reason='NO_INPUTS'
        self.last_ros=None;self.last_advance=None;self.clock_fault=False

    def clock(self, now, mono):
        if not math.isfinite(now) or not math.isfinite(mono) or now<=0:
            self.invalidate('CLOCK_UNAVAILABLE');return False
        if self.last_ros is not None and now < self.last_ros:
            self.clock_fault=True
        if self.last_ros is None or now>self.last_ros:self.last_advance=mono
        if self.last_advance is not None and mono-self.last_advance>.5:self.clock_fault=True
        self.last_ros=now
        if self.clock_fault:self.invalidate('CLOCK_FAULT');return False
        return True

    def invalidate(self, reason):
        self.context_id='';self.goal=None;self.reason=reason

    def update(self, name, message, mono):
        if name not in ('map','odom','alignment'):raise ValueError('UNKNOWN_INPUT')
        old=self.inputs.get(name)
        if old and stamp(message.header.stamp)<=stamp(old[0].header.stamp):
            return False  # Replay cannot renew receive freshness.
        if name=='map' and old:
            previous=old[0]
            if (message.epoch<previous.epoch or (message.epoch==previous.epoch and
                (message.map_id!=previous.map_id or (message.valid and previous.valid and message.version<=previous.version)))):
                return False
        self.inputs[name]=(deepcopy(message),mono)
        if name=='alignment':
            m=message
            if m.generation<self.alignment_generation:
                self.inputs[name]=old;return False
            try:
                translation,rotation=transform_parts(m.map_to_odom)
                quat=rotation.as_quat()
                if quat[3]<0:quat=-quat
                signature=(m.map_id,m.map_epoch,m.localization_session,m.alignment_id,
                           tuple(map(int,m.reset_counters)),tuple(map(float,translation)),tuple(map(float,quat)))
            except ValueError:
                self.alignment_conflict=True;self.invalidate('INVALID_ALIGNMENT_TRANSFORM');return False
            if m.generation==self.alignment_generation and signature!=self.alignment_signature:
                self.alignment_conflict=True
            elif m.generation>self.alignment_generation:
                self.alignment_conflict=False
                self.alignment_generation=m.generation;self.alignment_signature=signature
        if name=='map':
            try:
                if np.prod(message.shape,dtype=object)>4_000_000:raise ValueError('MAP_TOO_LARGE')
                self.grid=grid_from_message(message)
            except ValueError:
                self.grid=None
        return True

    def ready(self, now, mono):
        if not self.clock(now,mono):return False
        for name,age in (('map',2.),('odom',.5),('alignment',.5)):
            if name not in self.inputs:self.invalidate('MISSING_'+name.upper());return False
            message,received=self.inputs[name]
            if not (0<=mono-received<=age and -.05<=now-stamp(message.header.stamp)<=age):
                self.invalidate('STALE_'+name.upper());return False
        m,o,a=(self.inputs[name][0] for name in ('map','odom','alignment'))
        if (self.grid is None or not m.valid or m.header.frame_id!='map' or not m.map_id or
                not a.valid or a.header.frame_id!='map' or not a.localization_session or not a.alignment_id or
                a.generation==0 or (a.map_id,a.map_epoch)!=(m.map_id,m.epoch)):
            self.invalidate('MAP_ALIGNMENT_MISMATCH');return False
        if self.alignment_conflict:self.invalidate('ALIGNMENT_GENERATION_CONFLICT');return False
        identity_mismatch=(o.localization_session!=a.localization_session or
                           tuple(o.reset_counters)!=tuple(a.reset_counters))
        if identity_mismatch:self.invalidated_alignment_generation=max(self.invalidated_alignment_generation,a.generation)
        if (a.generation<=self.invalidated_alignment_generation or o.header!=o.odometry.header):
            self.invalidate('LOCALIZATION_SESSION_MISMATCH');return False
        try:self.map_odometry=transform_odometry(o.odometry,a.map_to_odom)
        except ValueError as error:self.invalidate(str(error));return False
        token=hashlib.sha256(json.dumps([self.alignment_generation,self.alignment_signature],
            separators=(',',':')).encode()).hexdigest()
        if self.context_id and self.context_id!=token:self.goal=None
        self.context_id=token;self.reason='READY';return True

    def dispatch(self, goal, now, mono):
        if not self.ready(now,mono):raise ValueError(self.reason)
        if (goal.header.frame_id!='map' or not -.05<=now-stamp(goal.header.stamp)<=.5 or
                stamp(goal.header.stamp)<=0 or not np.isfinite(xyz(goal.pose.position)).all()):
            raise ValueError('INVALID_GOAL')
        key=(goal.header.stamp.sec,goal.header.stamp.nanosec)
        if key<=self.last_goal_stamp:raise ValueError('REPLAYED_GOAL')
        if np.linalg.norm(xyz(self.map_odometry.twist.twist.linear))>.05:
            raise ValueError('REQUIRE_STOPPED_START')
        self.last_goal_stamp=key
        self.goal=(deepcopy(goal),self.context_id,now,self.inputs['map'][0].version)

    def bind(self, trajectory, now, mono):
        if not self.ready(now,mono):raise ValueError(self.reason)
        if not self.goal or self.goal[1]!=self.context_id:raise ValueError('NO_CURRENT_GOAL')
        goal,_,dispatched,min_version=self.goal
        m=self.inputs['map'][0]
        if (trajectory.goal_stamp!=goal.header.stamp or now-dispatched>2. or
                trajectory.header.frame_id!='map' or trajectory.parent_trajectory_id!=0 or
                not trajectory.trajectory_id or (trajectory.map_id,trajectory.epoch)!=(m.map_id,m.epoch) or
                not min_version<=trajectory.map_version<=m.version or
                not -.05<=now-stamp(trajectory.header.stamp)<=.5 or
                not .05<=stamp(trajectory.start_time)-now<=2.):
            raise ValueError('STALE_OR_UNBOUND_TRAJECTORY')
        curve=spline([xyz(p) for p in trajectory.control_points],trajectory.knot_interval)
        validate_trajectory(curve,self.grid,self.radius,self.limits)
        if (np.linalg.norm(curve(0)-xyz(self.map_odometry.pose.pose.position))>.1 or
                np.linalg.norm(curve(0,1))>.05 or np.linalg.norm(curve(0,2))>1e-5 or
                np.linalg.norm(curve(curve.t[-4])-xyz(goal.pose.position))>.15):
            raise ValueError('TRAJECTORY_ENDPOINT_MISMATCH')
        self.goal=None  # A raw result can be bound exactly once.
        return deepcopy(trajectory)
