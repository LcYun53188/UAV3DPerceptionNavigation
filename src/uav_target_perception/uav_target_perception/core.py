"""ROS-independent depth geometry, source gates and temporary person tracks."""
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class SurfacePoint:
    position: np.ndarray
    samples: int
    spread: float


def locate_surface(depth, box, intrinsic, near=0.3, far=8., min_samples=20,
                   max_spread=0.35):
    """Estimate a visible torso surface, rejecting sparse or mixed-depth patches.

    Requires rectified RGB and depth registered on the same pixel grid. Quality
    measures below describe depth support, not global position uncertainty.
    """
    depth = np.asarray(depth)
    if depth.ndim != 2 or len(box) != 4 or not np.isfinite(box).all():
        raise ValueError('Expected a depth image and finite xyxy box')
    fx, fy, cx, cy = intrinsic
    if not np.isfinite(intrinsic).all() or min(fx, fy) <= 0:
        raise ValueError('Invalid camera intrinsics')
    if not 0 < near < far or min_samples < 1 or max_spread <= 0:
        raise ValueError('Invalid depth policy')
    x1, y1, x2, y2 = box
    if x2 <= x1 or y2 <= y1:
        return None
    left = max(0, math.ceil(x1 + (x2-x1)*0.3))
    right = min(depth.shape[1], math.floor(x1 + (x2-x1)*0.7))
    top = max(0, math.ceil(y1 + (y2-y1)*0.2))
    bottom = min(depth.shape[0], math.floor(y1 + (y2-y1)*0.6))
    if right <= left or bottom <= top:
        return None
    patch = depth[top:bottom, left:right]
    valid = np.isfinite(patch) & (patch >= near) & (patch <= far)
    if valid.sum() < max(min_samples, patch.size*0.3):
        return None
    q1, median, q3 = np.percentile(patch[valid], [25, 50, 75])
    if q3-q1 > max_spread:
        return None
    support = valid & (np.abs(patch-median) <= max_spread/2)
    if support.sum() < min_samples:
        return None
    rows, cols = np.nonzero(support)
    u, v = np.median(cols)+left, np.median(rows)+top
    return SurfacePoint(np.array([(u-cx)*median/fx, (v-cy)*median/fy, median]),
                        int(support.sum()), float(q3-q1))


def transform_point(point, translation, quaternion):
    """Apply a finite translation and unit xyzw quaternion."""
    point, translation, q = (np.asarray(a, dtype=float) for a in
                             (point, translation, quaternion))
    if point.shape != (3,) or translation.shape != (3,) or q.shape != (4,):
        raise ValueError('Invalid transform shape')
    if not all(np.isfinite(a).all() for a in (point, translation, q)):
        raise ValueError('Nonfinite transform')
    norm = np.linalg.norm(q)
    if abs(norm-1.) > 0.01:
        raise ValueError('Transform quaternion is not unit length')
    q = q/norm
    return point + 2*np.cross(q[:3], np.cross(q[:3], point)+q[3]*point) + translation


def fresh(stamp, now, max_age):
    # Zero in tf2 means latest; never let an unset stamp select that shortcut.
    return stamp > 0 and math.isfinite(stamp) and 0 <= now-stamp <= max_age


class ContextGate:
    def __init__(self, timeout=2.):
        self.timeout = timeout
        self.key = None
        self.valid = False
        self.received = self.stamp = self.barrier = None

    def update(self, key, valid, stamp, steady):
        changed = key != self.key or bool(valid) != self.valid
        # Reject older contexts even if discovery delivers an old transient sample.
        if self.stamp is not None and stamp < self.stamp:
            return False
        if changed:
            self.barrier = stamp
        self.key, self.valid, self.stamp, self.received = key, bool(valid), stamp, steady
        return changed

    def ready(self, now, steady):
        return (self.valid and self.key is not None and self.received is not None
                and fresh(self.stamp, now, self.timeout)
                and 0 <= steady-self.received <= self.timeout)

    def accepts(self, stamp, now, steady, max_age):
        return (self.ready(now, steady) and stamp >= self.barrier
                and fresh(stamp, now, max_age))


@dataclass
class Track:
    id: int
    surface: SurfacePoint
    confidence: float
    stamp: float
    received: float
    hits: int = 1


class PersonTracker:
    """Bounded one-to-one spatial association; IDs do not identify individuals."""
    def __init__(self, radius=0.8, ttl=1.5, min_hits=3, max_tracks=50):
        if not (np.isfinite([radius, ttl]).all() and radius > 0 and ttl > 0
                and min_hits >= 1 and max_tracks >= 1):
            raise ValueError('Invalid tracking parameters')
        self.radius, self.ttl, self.min_hits = radius, ttl, min_hits
        self.max_tracks = max_tracks
        self.tracks = {}
        self.next_id = 1
        self.last_stamp = None

    def clear(self):
        self.tracks.clear()
        self.last_stamp = None

    def expire(self, now, steady):
        self.tracks = {key: track for key, track in self.tracks.items()
                       if 0 <= now-track.stamp <= self.ttl
                       and 0 <= steady-track.received <= self.ttl}

    def update(self, observations, stamp, steady):
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return
        if not all(np.isfinite(surface.position).all() and 0 <= score <= 1
                   for surface, score in observations):
            raise ValueError('Invalid observation')
        self.expire(stamp, steady)
        self.last_stamp = stamp
        pairs = sorted((float(np.linalg.norm(track.surface.position-surface.position)), key, index)
                       for key, track in self.tracks.items()
                       for index, (surface, _) in enumerate(observations))
        used_tracks, used_observations = set(), set()
        for distance, key, index in pairs:
            if distance > self.radius:
                break
            if key in used_tracks or index in used_observations:
                continue
            surface, confidence = observations[index]
            track = self.tracks[key]
            track.surface, track.confidence = surface, confidence
            track.stamp, track.received = stamp, steady
            track.hits += 1
            used_tracks.add(key)
            used_observations.add(index)
        for key in self.tracks.keys()-used_tracks:
            self.tracks[key].hits = 0
        for index, (surface, confidence) in enumerate(observations):
            if index not in used_observations and len(self.tracks) < self.max_tracks:
                key = self.next_id
                self.next_id += 1
                self.tracks[key] = Track(key, surface, confidence, stamp, steady)

    def confirmed(self):
        return [track for track in self.tracks.values() if track.hits >= self.min_hits]
