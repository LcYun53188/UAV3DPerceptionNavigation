"""Shared follow-exit timers. Capture replay, pauses and reacquisition never add moving time."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class FollowExitLimits:
    moving_s: float = 10.
    stationary_s: float = 5.
    lost_s: float = 5.
    multiple_s: float = 10.
    max_gap_s: float = .5

    def __post_init__(self):
        if not all(type(value) in (float,int) and math.isfinite(value) and value>0
                   for value in vars(self).values()):
            raise ValueError('INVALID_FOLLOW_EXIT_LIMITS')


class FollowExitClock:
    def __init__(self,limits=None):
        self.limits=limits or FollowExitLimits()
        self.moving_seconds=0.
        self.last_source=self.last_mono=self.last_motion=None
        self.static_source=self.static_mono=None
        self.lost_since=self.multiple_since=None

    def pause_motion(self):
        self.last_source=self.last_mono=self.last_motion=None
        self.static_source=self.static_mono=None

    def lost(self,now):
        if self.multiple_since is None and self.lost_since is None:
            self.lost_since=now
            self.pause_motion()

    def multiple(self,now):
        if self.multiple_since is None:
            self.multiple_since=now
            self.lost_since=None
            self.pause_motion()

    def reacquire(self,now):
        if self.multiple_since is not None or self.lost_since is None:
            return False
        if now-self.lost_since>=self.limits.lost_s:
            return False
        self.lost_since=None
        self.pause_motion()
        return True

    def sample(self,source,mono,motion):
        if motion not in ('MOVING','STATIONARY','UNKNOWN') or not all(math.isfinite(v) for v in (source,mono)):
            raise ValueError('INVALID_FOLLOW_MOTION_SAMPLE')
        if self.multiple_since is not None or self.lost_since is not None:
            return
        if self.last_source is not None:
            if source<self.last_source or mono<self.last_mono:
                raise ValueError('FOLLOW_TIME_REVERSED')
            if source==self.last_source:
                return
            source_dt,mono_dt=source-self.last_source,mono-self.last_mono
            if max(source_dt,mono_dt)>self.limits.max_gap_s+1e-9:
                self.static_source=self.static_mono=None
            if (motion=='MOVING' and self.last_motion=='MOVING'
                    and max(source_dt,mono_dt)<=self.limits.max_gap_s+1e-9):
                self.moving_seconds+=min(source_dt,mono_dt)
        self.last_source,self.last_mono,self.last_motion=source,mono,motion
        if motion!='STATIONARY':
            self.static_source=self.static_mono=None
        elif self.static_source is None:
            self.static_source,self.static_mono=source,mono

    def reason(self,now):
        if self.multiple_since is not None:
            return 'MULTIPLE_PEOPLE_OBSERVED' if now-self.multiple_since>=self.limits.multiple_s-1e-9 else None
        if self.lost_since is not None:
            return 'TARGET_LOST_TIMEOUT' if now-self.lost_since>=self.limits.lost_s-1e-9 else None
        if self.moving_seconds>=self.limits.moving_s-1e-9:
            return 'MOVING_TIME_BUDGET_REACHED'
        if (self.static_source is not None and self.last_source-self.static_source>=self.limits.stationary_s-1e-9
                and now-self.static_mono>=self.limits.stationary_s-1e-9):
            return 'PERSON_STATIC_OBSERVED'
        return None

    def status(self,now):
        return dict(moving_seconds=self.moving_seconds,
            stationary_seconds=min(self.last_source-self.static_source,now-self.static_mono)
                if self.static_source is not None else 0.,
            lost_seconds=max(0.,now-self.lost_since) if self.lost_since is not None else 0.,
            multiple_seconds=max(0.,now-self.multiple_since) if self.multiple_since is not None else 0.)
