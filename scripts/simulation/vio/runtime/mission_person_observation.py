"""Task-scoped follow/observe/reacquire shadow episodes; never interrupts real flight."""
from collections import deque
from dataclasses import asdict,dataclass
import math
import uuid

import numpy as np
from person_follow_exit_rules import FollowExitClock,FollowExitLimits


@dataclass(frozen=True)
class EpisodeConfig:
    motion_confirm_s: float = 1.5
    motion_speed_mps: float = .2
    motion_displacement_m: float = .3
    min_samples: int = 5
    stationary_speed_mps: float = .08
    stationary_observe_s: float = 5.
    moving_follow_s: float = 10.
    lost_wait_s: float = 5.
    multi_observe_s: float = 10.
    multi_confirm_s: float = .5
    multi_min_samples: int = 3
    cooldown_s: float = 15.
    max_gap_s: float = .5
    max_episodes: int = 16
    max_points: int = 1000

    def __post_init__(self):
        if any(type(v) not in (int,float) or not math.isfinite(v) or v<=0 for v in asdict(self).values()):
            raise ValueError('INVALID_EPISODE_CONFIG')
        if self.stationary_speed_mps>=self.motion_speed_mps or self.min_samples<3 or self.multi_min_samples<2:
            raise ValueError('INVALID_MOTION_HYSTERESIS')
        if any(type(getattr(self,key)) is not int for key in ('min_samples','multi_min_samples','max_episodes','max_points')):
            raise ValueError('INVALID_EPISODE_LIMIT')

    def exit_limits(self):
        return FollowExitLimits(self.moving_follow_s,self.stationary_observe_s,
                                self.lost_wait_s,self.multi_observe_s,self.max_gap_s)


class MissionPersonObservation:
    def __init__(self,config=None):
        self.config=config or EpisodeConfig()
        self.enabled=False
        self.phase,self.reason='DISABLED','MODE_OFF'
        self.context=self.mission=self.target=None
        self.samples=deque(maxlen=200)
        self.episodes=deque(maxlen=self.config.max_episodes)
        self.active=None
        self.clock=FollowExitClock(self.config.exit_limits())
        self.cooldown_until=0.
        self.last_source=self.last_mono=None
        self._capture=None
        self.speed=None
        self._segment=0
        self._now=0.

    def _reset_motion(self):
        self.samples.clear()
        self.last_source=self.last_mono=None
        self.speed=None

    def enable(self,value,mono):
        if type(value) is not bool:
            raise ValueError('INVALID_MODE')
        if self.enabled==value:
            return
        self.finish('MODE_DISABLED',mono)
        self.enabled=value
        if value:
            self.context=self.mission=None
            self._capture=None
            self.cooldown_until=mono
        self.phase,self.reason=('MONITORING','WAITING_MISSION_AND_PERSON') if value else ('DISABLED','MODE_OFF')
        self._reset_motion()
        self.target=None

    def finish(self,reason,mono):
        self._now=mono
        if self.active is not None:
            self.active.update(end_reason=reason,ended_mono=mono,timers=self.clock.status(mono))
            self.active=None
            self.cooldown_until=mono+self.config.cooldown_s

    def unavailable(self,reason,mono):
        """Hard health/context failure, distinct from a person temporarily disappearing."""
        self.finish(reason,mono)
        self._reset_motion()
        self.target=None
        self.phase,self.reason=('MONITORING',reason) if self.enabled else ('DISABLED','MODE_OFF')

    def _scope(self,context,mission,mono):
        context,mission=tuple(context),tuple(mission)
        reason=('MAP_CONTEXT_CHANGED' if self.context is not None and context!=self.context else
                'MISSION_IDENTITY_CHANGED' if self.mission is not None and mission!=self.mission else None)
        if reason:
            self.unavailable(reason,mono)
            self.enabled=False
            self.phase,self.reason='DISABLED',reason+'_REAUTHORIZE_MODE'
            return False
        self.context,self.mission=context,mission
        return True

    def lost(self,mono,reason='TARGET_LOST'):
        self._now=mono
        if self.active is None:
            self._reset_motion()
            self.phase,self.reason=('MONITORING',reason) if self.enabled else ('DISABLED','MODE_OFF')
            return
        if self.clock.multiple_since is not None:
            return  # Once confirmed, multiple-person observation stays locked.
        if self.clock.lost_since is None:
            self.clock.lost(mono)
            self._reset_motion()
        self.phase,self.reason='SHADOW_LOST_WAIT',reason

    def multiple(self,*,mono,context,mission,count):
        if not self.enabled or count<2 or not self._scope(context,mission,mono):
            return
        self._now=mono
        if self.active is None:
            if mono<self.cooldown_until:
                self.phase,self.reason='COOLDOWN','PREVENT_REPEAT_INTERRUPTION'
                return
            self._new_episode(None,mono)
        self.active['multiple_count']=max(count,self.active.get('multiple_count',0))
        self.clock.multiple(mono)
        self.phase,self.reason='SHADOW_MULTI_OBSERVE','MULTIPLE_PEOPLE_LOCKED'
        self.advance(mono)

    def _new_episode(self,target,mono):
        self.clock=FollowExitClock(self.config.exit_limits())
        self._segment=0
        points=[dict(stamp=t,position=p.tolist(),segment=0) for t,p in self.samples] if target else []
        self.target=target
        self.active=dict(episode_id=str(uuid.uuid4()),track_id=target,
            context=list(self.context),mission=list(self.mission),started_mono=mono,
            source_start=points[0]['stamp'] if points else None,
            mode='SHADOW_ONLY',mission_interrupted=False,point_semantics='visible_torso_surface',
            points=points,end_reason=None)
        self.episodes.append(self.active)

    def advance(self,mono):
        if not self.enabled:
            return
        if not math.isfinite(mono):
            raise ValueError('INVALID_MONOTONIC_TIME')
        self._now=mono
        if self.last_mono is not None and mono<self.last_mono:
            self.unavailable('TIME_REVERSED',mono)
            self.enabled=False
            self.phase='DISABLED'
            return
        if (self.active is not None and self.clock.multiple_since is None
                and self.clock.lost_since is None and self.last_mono is not None
                and mono-self.last_mono>self.config.max_gap_s):
            self.lost(mono,'OBSERVATION_GAP')
        if self.active is not None:
            reason=self.clock.reason(mono)
            if reason:
                self.finish(reason,mono)
                self.phase,self.reason='COOLDOWN',reason+'_ROUTE_RECHECK_REQUIRED'

    def sample(self,*,track_id,point,source_stamp,mono,context,mission):
        if not self.enabled:
            return
        c=self.config
        p=np.asarray(point,float)
        if (not track_id or p.shape!=(3,) or not np.isfinite(p).all()
                or not all(math.isfinite(v) for v in (source_stamp,mono))):
            self.unavailable('INVALID_OBSERVATION',mono)
            return
        if not self._scope(context,mission,mono):
            return
        self.advance(mono)
        if not self.enabled or self.active is not None and self.clock.multiple_since is not None:
            return
        if self.active is not None and track_id!=self.active['track_id']:
            self.lost(mono,'WAITING_ORIGINAL_TARGET_IDENTITY')
            return
        scope=(self.context,self.mission,track_id)
        if self._capture is not None and self._capture[:3]==scope:
            old_stamp,old_point=self._capture[3:]
            if source_stamp<=old_stamp:
                if source_stamp<old_stamp:
                    self.unavailable('TIME_REVERSED',mono)
                elif not np.array_equal(p,old_point):
                    self.unavailable('CONFLICTING_CAPTURE',mono)
                return
        self._capture=(*scope,source_stamp,p.copy())
        if self.last_source is not None:
            gap=max(source_stamp-self.last_source,mono-self.last_mono)
            if gap>c.max_gap_s:
                self.lost(mono,'OBSERVATION_GAP')
            elif np.linalg.norm(p[:2]-self.samples[-1][1][:2])/(source_stamp-self.last_source)>2.:
                self.lost(mono,'TARGET_JUMP')
                return
        self.target,self.last_source,self.last_mono=track_id,source_stamp,mono
        self.samples.append((source_stamp,p))
        while len(self.samples)>2 and source_stamp-self.samples[1][0]>=c.motion_confirm_s:
            self.samples.popleft()
        duration=source_stamp-self.samples[0][0]
        displacement=float(np.linalg.norm(p[:2]-self.samples[0][1][:2]))
        qualified=len(self.samples)>=c.min_samples and duration>=c.motion_confirm_s
        self.speed=displacement/duration if duration>0 else None
        initial_moving=qualified and self.speed>c.motion_speed_mps and displacement>=c.motion_displacement_m
        stationary=qualified and self.speed<c.stationary_speed_mps
        moving=initial_moving or self.active is not None and qualified and not stationary
        if self.active is None:
            if mono<self.cooldown_until:
                self.phase,self.reason='COOLDOWN','PREVENT_REPEAT_INTERRUPTION'
                return
            self.phase,self.reason='MONITORING','CONFIRMING_PERSON_MOTION'
            if not initial_moving:
                return
            self._new_episode(track_id,mono)
        if self.clock.lost_since is not None:
            if not qualified or not (moving or stationary):
                self.phase,self.reason='SHADOW_LOST_WAIT','RECONFIRMING_ORIGINAL_TARGET'
                return
            if not self.clock.reacquire(mono):
                self.advance(mono)
                return
            self._segment+=1
            for t,point in self.samples:
                self._append(t,point)
        self._append(source_stamp,p)
        motion='STATIONARY' if stationary else 'MOVING' if moving else 'UNKNOWN'
        self.clock.sample(source_stamp,mono,motion)
        self.phase,self.reason=('SHADOW_OBSERVE','PERSON_STATIC_OBSERVING') if stationary else ('SHADOW_FOLLOW','PERSON_MOVING')
        self.active['timers']=self.clock.status(mono)
        self.advance(mono)

    def _append(self,stamp,point):
        points=self.active['points']
        if not points or stamp>points[-1]['stamp']:
            points.append(dict(stamp=stamp,position=point.tolist(),segment=self._segment))
        if len(points)>self.config.max_points:
            del points[:len(points)-self.config.max_points]
            self.active['truncated']=True

    def status(self):
        return dict(enabled=self.enabled,phase=self.phase,reason=self.reason,target=self.target,
            speed_mps=self.speed,episode_count=len(self.episodes),
            active_episode=self.active['episode_id'] if self.active else None,
            timers=self.clock.status(self._now) if self.active is not None else
                self.episodes[-1].get('timers',self.clock.status(self._now)) if self.episodes else self.clock.status(self._now),
            mission_interrupted=False,execution_ready=False,
            authorized_mission=list(self.mission) if self.enabled and self.mission is not None else None,
            config=asdict(self.config))
