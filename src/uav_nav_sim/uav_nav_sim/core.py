"""ROS-independent map parsing and conservative cubic B-spline validation."""
from array import array
from dataclasses import dataclass
from enum import IntEnum
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.interpolate import BSpline


class CellState(IntEnum):
    UNKNOWN = 0
    FREE = 1
    OCCUPIED = 2
    OUT_OF_MAP = 3


def pack_grid_data(grid):
    """Owned native buffers for ROS sequences, avoiding per-voxel Python lists."""
    return (array('f', np.asarray(grid.distance, dtype=np.float32).tobytes(order='C')),
            array('B', np.asarray(grid.observed, dtype=np.uint8).tobytes(order='C')))


def parse_esdf(response, max_voxels=4_000_000):
    if not response.success or response.header.frame_id != 'map':
        raise ValueError('ESDF response failed or wrong frame')
    r = float(response.voxel_size_m)
    origin = np.array([response.origin_m.x, response.origin_m.y, response.origin_m.z])
    if not np.isfinite(r) or r <= 0 or not np.all(np.isfinite(origin)):
        raise ValueError('Invalid ESDF geometry')
    a = response.esdf_and_gradients
    dims = a.layout.dim
    if len(dims) != 3 or [d.label for d in dims] != ['x', 'y', 'z']:
        raise ValueError('Expected distance-only x/y/z grid')
    shape = tuple(int(d.size) for d in dims)
    if min(shape) <= 0 or np.prod(shape, dtype=object) > max_voxels:
        raise ValueError('Invalid or oversized grid')
    # ROS MultiArray strides include the dimension itself, not numpy strides.
    sx, sy, sz = [int(d.stride) for d in dims]
    if sz < shape[2] or sy < shape[1]*sz or sx < shape[0]*sy:
        raise ValueError('Overlapping strides')
    offset = int(a.layout.data_offset)
    last = offset + (shape[0]-1)*sy + (shape[1]-1)*sz + shape[2]-1
    data = np.asarray(a.data, dtype=np.float32)
    if offset < 0 or last >= len(data):
        raise ValueError('Truncated ESDF data')
    indices = offset + np.arange(shape[0])[:, None, None]*sy + np.arange(shape[1])[None, :, None]*sz + np.arange(shape[2])[None, None, :]
    distance = data[indices].copy()
    observed = np.isfinite(distance) & (distance != -1000.0)
    return Grid(origin, r, distance, observed)


@dataclass(frozen=True)
class Grid:
    origin: np.ndarray
    resolution: float
    distance: np.ndarray
    observed: np.ndarray

    def __post_init__(self):
        if (self.distance.ndim != 3 or self.distance.shape != self.observed.shape or
                min(self.distance.shape) <= 0 or not np.isfinite(self.resolution) or self.resolution <= 0 or
                not np.all(np.isfinite(self.origin))):
            raise ValueError('Invalid grid')
        self.origin.setflags(write=False)
        self.distance.setflags(write=False)
        self.observed.setflags(write=False)

    def index(self, point):
        p = np.asarray(point)
        if p.shape != (3,) or not np.all(np.isfinite(p)):
            return None
        i = np.floor((p-self.origin)/self.resolution).astype(int)
        return i if np.all(i >= 0) and np.all(i < self.distance.shape) else None

    def state(self, point):
        i = self.index(point)
        if i is None:
            return CellState.OUT_OF_MAP
        i = tuple(i)
        if not self.observed[i] or not np.isfinite(self.distance[i]):
            return CellState.UNKNOWN
        return CellState.FREE if self.distance[i] > 0 else CellState.OCCUPIED

    def collision(self, point, radius):
        """Distance bound plus observed-volume check, including voxel quantization."""
        i = self.index(point)
        if i is None or not np.isfinite(radius) or radius < 0:
            return True
        p = np.asarray(point)
        lo = np.floor((p-radius-self.origin)/self.resolution).astype(int)
        hi = np.floor((p+radius-self.origin)/self.resolution).astype(int)
        if np.any(lo < 0) or np.any(hi >= self.distance.shape):
            return True
        region = tuple(slice(a, b+1) for a, b in zip(lo, hi))
        if not np.all(self.observed[region]):
            return True
        d = self.distance[tuple(i)]
        return not np.isfinite(d) or d <= radius + np.sqrt(3)*self.resolution/2


def spline(controls, interval):
    controls = np.asarray(controls, dtype=float)
    if (controls.ndim != 2 or controls.shape[1] != 3 or len(controls) < 7 or len(controls) > 2000 or
            not np.all(np.isfinite(controls)) or not np.isfinite(interval) or interval <= 0):
        raise ValueError('Invalid cubic spline')
    return BSpline((np.arange(len(controls)+4)-3)*interval, controls, 3, extrapolate=False)


def derivative_bounds(curve):
    return [float(np.max(np.linalg.norm(curve.derivative(d).c[:len(curve.c)-d], axis=1))) for d in (1, 2, 3)]


def validate_trajectory(curve, grid, radius, limits, start=0.0):
    """Derivative convex hull bounds + swept-sphere cover of the entire curve."""
    bounds = derivative_bounds(curve)
    if any(not np.isfinite(v) or v > lim*(1+1e-6) for v, lim in zip(bounds, limits)):
        raise ValueError('Dynamic limits exceeded')
    end = float(curve.t[-4])
    if end <= 0 or end > 600 or not 0 <= start <= end:
        raise ValueError('Invalid trajectory time span')
    if np.linalg.norm(curve(end, 1)) > 1e-5 or np.linalg.norm(curve(end, 2)) > 1e-5:
        raise ValueError('Terminal state is not at rest')
    step = min(0.1, grid.resolution / max(bounds[0], 0.01) / 2)
    count = max(1, int(np.ceil((end-start)/step)))
    times = np.linspace(start, end, count+1)
    # Every curve point is within v_bound * dt / 2 of a checked endpoint.
    padding = bounds[0]*(end-start)/count/2
    if any(grid.collision(p, radius+padding) for p in curve(times)):
        raise ValueError('Trajectory intersects occupied, unknown or outside map')
    return bounds


def grid_from_message(msg):
    shape = tuple(msg.shape)
    if np.prod(shape, dtype=object) != len(msg.distance) or len(msg.distance) != len(msg.observed):
        raise ValueError('Snapshot size mismatch')
    return Grid(np.array([msg.origin.x, msg.origin.y, msg.origin.z]), msg.resolution,
                np.asarray(msg.distance, dtype=np.float32).reshape(shape).copy(),
                np.asarray(msg.observed, dtype=bool).reshape(shape).copy())


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def validate_bundle(folder, scene_id, resolution, build_id=None):
    folder = Path(folder).resolve()
    manifest = json.loads((folder/'manifest.json').read_text())
    if (manifest['schema'] != 1 or manifest['frame_id'] != 'map' or
            manifest['scene_id'] != scene_id or manifest['resolution'] != resolution or
            manifest['localization'] != 'gazebo_world_identity'):
        raise ValueError('Incompatible scene, frame, resolution or localization')
    if build_id is not None and manifest.get('build_id') != build_id:
        raise ValueError('Map producer version/patch mismatch')
    if sha256(folder/'static_map.nvblx') != manifest['sha256']:
        raise ValueError('Map checksum mismatch')
    return manifest
