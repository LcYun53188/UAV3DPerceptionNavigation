"""Selected-person preview lifecycle. No flight authority or navigation output."""
from collections import deque
import math
import numpy as np


def context_identity(msg):
    return (msg.map_id, msg.map_epoch, msg.localization_session, msg.alignment_id,
            msg.alignment_generation, msg.context_id)


def camera_pixel(position, transform, intrinsic):
    """Map point through camera<-map TF; CameraInfo pinhole optical convention."""
    t, q = transform.translation, transform.rotation
    quat = np.array([q.x, q.y, q.z, q.w], float)
    if not np.isfinite(quat).all() or abs(np.linalg.norm(quat)-1.) > .01:
        raise ValueError('INVALID_CAMERA_TRANSFORM')
    quat /= np.linalg.norm(quat)
    p = np.asarray(position, float)
    cross = 2*np.cross(quat[:3], p)
    p = p + quat[3]*cross + np.cross(quat[:3], cross) + [t.x,t.y,t.z]
    k = np.asarray(intrinsic, float).reshape(3,3)
    if not np.isfinite(p).all() or not np.isfinite(k).all() or p[2] <= .05 or k[0,0] <= 0 or k[1,1] <= 0:
        raise ValueError('TARGET_OUTSIDE_CAMERA')
    uv = k @ p
    return (uv[:2]/uv[2]).tolist()


def candidate_goal(person, aircraft, distance=3., max_step=.5, minimum=2.5):
    person, aircraft = np.asarray(person, float), np.asarray(aircraft, float)
    if (person.shape != (3,) or aircraft.shape != (3,) or not np.isfinite(person).all()
            or not np.isfinite(aircraft).all() or not all(math.isfinite(v) and v > 0 for v in (distance,max_step,minimum))
            or minimum >= distance):
        raise ValueError('INVALID_FOLLOW_GEOMETRY')
    delta = aircraft[:2]-person[:2]
    separation = float(np.linalg.norm(delta))
    if separation < minimum:
        return None, separation, 'TARGET_TOO_CLOSE'
    if separation <= distance+.2:
        return None, separation, 'WITHIN_STANDOFF'
    desired = person[:2] + delta/separation*distance
    movement = desired-aircraft[:2]
    length = float(np.linalg.norm(movement))
    xy = aircraft[:2] + movement*min(1.,max_step/length)
    return [float(xy[0]),float(xy[1]),float(aircraft[2])], separation, 'CANDIDATE_ONLY_NOT_PLANNED'


class FollowPreview:
    """Every active state requires external fresh, identity-bound observations."""
    def __init__(self):
        self.reset()

    def reset(self):
        self.state, self.reason = 'IDLE', 'SELECT_TARGET'
        self.active = self.paused = False
        self.track_id = None
        self.goal = self.distance = self.velocity = None
        self.motion = 'UNKNOWN'
        self.samples = deque(maxlen=16)

    def invalidate(self, reason):
        self.active = self.paused = False
        self.goal = None
        self.state, self.reason = 'LOST', reason

    def observe(self, track_id, person, aircraft, source_stamp):
        if self.state == 'LOST':
            return  # New selection is required after any observed interruption.
        if not track_id or (self.track_id is not None and track_id != self.track_id):
            self.invalidate('TARGET_TRACK_CHANGED')
            return
        self.track_id = track_id
        if self.samples and source_stamp == self.samples[-1][0]:
            if not np.array_equal(np.asarray(person,float),self.samples[-1][1]):
                self.invalidate('CONFLICTING_CAPTURE_POSITION')
                return
        if not self.samples or source_stamp > self.samples[-1][0]:
            point = np.asarray(person, float)
            if point.shape != (3,) or not np.isfinite(point).all() or not math.isfinite(source_stamp):
                self.invalidate('INVALID_TARGET_POSITION')
                return
            if self.samples:
                dt = source_stamp-self.samples[-1][0]
                instantaneous = np.linalg.norm(point[:2]-self.samples[-1][1][:2])/dt
                if instantaneous > 2.:
                    self.invalidate('TARGET_JUMP_OR_EXCESS_SPEED')
                    return
            self.samples.append((source_stamp,point))
            if len(self.samples) >= 3 and source_stamp-self.samples[0][0] >= .3:
                self.velocity = ((point[:2]-self.samples[0][1][:2])/(source_stamp-self.samples[0][0])).tolist()
                speed = float(np.linalg.norm(self.velocity))
                if speed > .15:
                    self.motion = 'MOVING'
                elif speed < .08:
                    self.motion = 'STATIONARY'
        elif source_stamp < self.samples[-1][0]:
            self.invalidate('TARGET_TIME_REVERSED')
            return
        goal, self.distance, reason = candidate_goal(person,aircraft)
        self.goal = goal if self.active and not self.paused else None
        self.state = 'PAUSED' if self.paused else ('PREVIEW' if self.active else 'READY')
        self.reason = reason if self.active else 'TARGET_BOUND'

    def command(self, verb):
        if verb == 'EXECUTE':
            return False, 'DYNAMIC_SAFETY_UNQUALIFIED;PX4_FOLLOW_BT_STEP_UNAVAILABLE'
        if verb == 'CANCEL':
            self.active = self.paused = False
            self.goal = None
            if self.state != 'LOST':
                self.state = 'READY' if self.track_id else 'IDLE'
            self.reason = 'PREVIEW_CANCELED_NO_FLIGHT_COMMAND'
            return True, self.reason
        if self.state in ('LOST','IDLE') or self.track_id is None:
            return False, self.reason
        if verb == 'START' and self.state == 'READY':
            self.active, self.paused, self.state = True, False, 'PREVIEW'
            return True, 'PREVIEW_STARTED_NO_FLIGHT_COMMAND'
        if verb == 'PAUSE' and self.active and not self.paused:
            self.paused, self.state, self.goal = True, 'PAUSED', None
            return True, 'PREVIEW_PAUSED_NO_FLIGHT_COMMAND'
        if verb == 'RESUME' and self.active and self.paused:
            self.paused, self.state = False, 'PREVIEW'
            return True, 'PREVIEW_RESUMED_NO_FLIGHT_COMMAND'
        return False, 'INVALID_FOLLOW_PHASE'
