"""Task-scoped automatic person episodes and bounded map-frame trajectory history.

This policy observes the requested pause/follow/observe/resume sequence in shadow
mode. It never pauses the mission, acquires control or authorizes flight.
"""
from collections import deque
from dataclasses import asdict, dataclass
import math
import uuid

import numpy as np


@dataclass(frozen=True)
class EpisodeConfig:
    motion_confirm_s: float = 1.5
    motion_speed_mps: float = .2
    motion_displacement_m: float = .3
    min_samples: int = 5
    stationary_speed_mps: float = .08
    stationary_confirm_s: float = 2.
    observe_s: float = 3.
    follow_s: float = 10.
    cooldown_s: float = 15.
    max_gap_s: float = .5
    max_episodes: int = 16
    max_points: int = 1000

    def __post_init__(self):
        for key,value in asdict(self).items():
            if isinstance(value,bool) or not math.isfinite(value) or value <= 0:
                raise ValueError('INVALID_EPISODE_CONFIG:'+key)
        if self.stationary_speed_mps >= self.motion_speed_mps or self.min_samples < 3:
            raise ValueError('INVALID_MOTION_HYSTERESIS')
        if any(type(getattr(self,key)) is not int for key in ('min_samples','max_episodes','max_points')):
            raise ValueError('INVALID_EPISODE_LIMIT')


class MissionPersonObservation:
    def __init__(self, config=None):
        self.config = config or EpisodeConfig()
        self.enabled = False
        self.phase, self.reason = 'DISABLED','MODE_OFF'
        self.context = self.mission = self.target = None
        self.samples = deque(maxlen=200)
        self.episodes = deque(maxlen=self.config.max_episodes)
        self.active = None
        self.cooldown_until = 0.
        self.last_source = self.last_mono = None
        self.speed = None
        self.static_since = None

    def enable(self, value, mono):
        if type(value) is not bool:
            raise ValueError('INVALID_MODE')
        if self.enabled == value:
            return
        self.finish('MODE_DISABLED',mono)
        self.enabled = value
        self.phase,self.reason = ('MONITORING','WAITING_MISSION_AND_PERSON') if value else ('DISABLED','MODE_OFF')
        self.samples.clear()
        self.target = self.last_source = self.last_mono = None

    def finish(self, reason, mono):
        if self.active is not None:
            self.active.update(end_reason=reason,ended_mono=mono)
            self.active = None
            self.cooldown_until = mono+self.config.cooldown_s
        self.static_since = None

    def unavailable(self, reason, mono):
        self.finish(reason,mono)
        self.samples.clear()
        self.target = self.last_source = self.last_mono = None
        self.speed = None
        self.phase,self.reason = ('MONITORING',reason) if self.enabled else ('DISABLED','MODE_OFF')

    def sample(self, *, track_id, point, source_stamp, mono, context, mission):
        if not self.enabled:
            return
        c = self.config
        p = np.asarray(point,float)
        if (not track_id or p.shape != (3,) or not np.isfinite(p).all()
                or not all(math.isfinite(v) for v in (source_stamp,mono))):
            self.unavailable('INVALID_OBSERVATION',mono)
            return
        context,mission = tuple(context),tuple(mission)
        if self.context is not None and context != self.context:
            self.unavailable('MAP_CONTEXT_CHANGED',mono)
        if self.mission is not None and mission != self.mission:
            self.unavailable('MISSION_IDENTITY_CHANGED',mono)
        self.context,self.mission = context,mission
        if self.target is not None and track_id != self.target:
            self.unavailable('TARGET_CHANGED',mono)
        if self.last_source is not None:
            if source_stamp == self.last_source:
                if not np.array_equal(p,self.samples[-1][1]):
                    self.unavailable('CONFLICTING_CAPTURE',mono)
                return
            if source_stamp < self.last_source or mono < self.last_mono:
                self.unavailable('TIME_REVERSED',mono)
                return
            if source_stamp-self.last_source>c.max_gap_s or mono-self.last_mono>c.max_gap_s:
                self.unavailable('OBSERVATION_GAP',mono)
            elif np.linalg.norm(p[:2]-self.samples[-1][1][:2])/(source_stamp-self.last_source)>2.:
                self.unavailable('TARGET_JUMP',mono)
                return
        self.target,self.last_source,self.last_mono = track_id,source_stamp,mono
        self.samples.append((source_stamp,p))
        # A time window, rather than a fixed sample count, defines confirmation.
        while len(self.samples)>2 and source_stamp-self.samples[1][0]>=c.motion_confirm_s:
            self.samples.popleft()
        duration = source_stamp-self.samples[0][0]
        displacement = float(np.linalg.norm(p[:2]-self.samples[0][1][:2]))
        qualified = len(self.samples)>=c.min_samples and duration>=c.motion_confirm_s
        self.speed = displacement/duration if duration>0 else None
        moving = qualified and self.speed>c.motion_speed_mps and displacement>=c.motion_displacement_m
        stationary = qualified and self.speed<c.stationary_speed_mps
        if self.active is None:
            if mono<self.cooldown_until:
                self.phase,self.reason = 'COOLDOWN','PREVENT_REPEAT_INTERRUPTION'
                return
            self.phase,self.reason = 'MONITORING','CONFIRMING_PERSON_MOTION'
            if not moving:
                return
            self.active = dict(episode_id=str(uuid.uuid4()),track_id=track_id,
                context=list(context),mission=list(mission),started_mono=mono,
                source_start=self.samples[0][0],mode='SHADOW_ONLY',mission_interrupted=False,
                point_semantics='visible_torso_surface',
                points=[dict(stamp=t,position=point.tolist()) for t,point in self.samples],end_reason=None)
            self.episodes.append(self.active)
        points = self.active['points']
        if not points or points[-1]['stamp'] != source_stamp:
            points.append(dict(stamp=source_stamp,position=p.tolist()))
        if len(points)>c.max_points:
            del points[:len(points)-c.max_points]
            self.active['truncated'] = True
        if mono-self.active['started_mono']>=c.follow_s:
            self.finish('FOLLOW_TIME_BUDGET_REACHED',mono)
            self.phase,self.reason = 'COOLDOWN','SHADOW_EPISODE_FINISHED_ORIGINAL_TASK_UNCHANGED'
            return
        if not stationary:
            self.static_since = None
        elif stationary and self.static_since is None:
            self.static_since = mono
        if self.static_since is not None and mono-self.static_since>=c.stationary_confirm_s:
            self.phase,self.reason = 'SHADOW_OBSERVE','PERSON_STATIONARY_OBSERVING'
            if mono-self.static_since>=c.stationary_confirm_s+c.observe_s:
                self.finish('PERSON_STATIC_OBSERVED',mono)
                self.phase,self.reason = 'COOLDOWN','WOULD_RESUME_ORIGINAL_TASK_NOT_INTERRUPTED'
        else:
            self.phase,self.reason = 'SHADOW_FOLLOW','WOULD_PAUSE_AND_FOLLOW_EXECUTION_BLOCKED'

    def status(self):
        return dict(enabled=self.enabled,phase=self.phase,reason=self.reason,target=self.target,
            speed_mps=self.speed,episode_count=len(self.episodes),
            active_episode=self.active['episode_id'] if self.active else None,
            mission_interrupted=False,execution_ready=False,config=asdict(self.config))
