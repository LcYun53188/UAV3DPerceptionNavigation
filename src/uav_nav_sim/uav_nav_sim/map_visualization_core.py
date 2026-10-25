"""Read-only display math. Colors never change navigation's map or clearance."""
import math
import numpy as np


def ramp(values):
    """Blue -> cyan -> green -> yellow -> red, with opaque alpha."""
    v = np.clip(np.asarray(values, dtype=float), 0., 1.)
    return np.column_stack((np.clip(4*v-2, 0, 1), np.clip(2-np.abs(4*v-2), 0, 1),
                            np.clip(2-4*v, 0, 1), np.ones(v.size)))


def slice_colors(distance, resolution, mode, *, distance_max=3., clearance=.8, caution_margin=.3):
    d = np.asarray(distance, dtype=float)
    if mode == 'esdf':
        return ramp(1. - d/distance_max)
    if mode != 'traversability':
        raise ValueError('Expected esdf or traversability')
    # Conservative clearance for every point in the displayed voxel, not just
    # its center. This display does not assert route connectivity or dynamics.
    margin = d - resolution*math.sqrt(3)/2 - clearance
    color = np.tile([1., .1, .1, 1.], (d.size, 1))
    color[margin > 0] = [1., .75, 0., 1.]
    color[margin >= caution_margin] = [.1, .9, .2, 1.]
    return color


def snapshot_slice(msg, height, mode, **scales):
    shape = tuple(int(n) for n in msg.shape)
    r = float(msg.resolution)
    origin = np.array([msg.origin.x, msg.origin.y, msg.origin.z])
    if (len(shape) != 3 or min(shape) <= 0 or math.prod(shape) > 4_000_000
            or not math.isfinite(r) or r <= 0 or not np.all(np.isfinite(origin))):
        raise ValueError('Invalid snapshot geometry')
    if len(msg.distance) != math.prod(shape) or len(msg.observed) != math.prod(shape):
        raise ValueError('Truncated snapshot')
    iz = math.floor((height-origin[2])/r)
    if not 0 <= iz < shape[2]:
        return np.empty((0, 3)), np.empty((0, 4))
    d = np.asarray(msg.distance).reshape(shape)[:, :, iz]
    observed = np.asarray(msg.observed).reshape(shape)[:, :, iz]
    mask = (observed == 1) & np.isfinite(d) & (d != -1000.)
    xy = np.argwhere(mask)
    points = np.column_stack((origin[0] + (xy[:, 0]+.5)*r,
                              origin[1] + (xy[:, 1]+.5)*r,
                              np.full(len(xy), origin[2]+(iz+.5)*r)))
    return points, slice_colors(d[mask], r, mode, **scales)


def surface_colors(vertices, mode, *, age=0., age_max=30., people=(), radius=.4):
    vertices = np.asarray(vertices, dtype=float).reshape((-1, 3))
    if not np.all(np.isfinite(vertices)):
        raise ValueError('Nonfinite mesh')
    if mode == 'age':
        return ramp(np.full(len(vertices), max(0., age)/age_max))
    if mode != 'semantic':
        raise ValueError('Expected age or semantic')
    colors = np.tile([.45, .45, .45, 1.], (len(vertices), 1))
    # The detector provides one visible torso surface point, not segmentation.
    # Magenta means spatially associated with that point; all else is unknown.
    for person in people:
        p = np.asarray(person, dtype=float)
        if p.shape == (3,) and np.all(np.isfinite(p)):
            colors[np.sum((vertices-p)**2, axis=1) <= radius**2] = [1., .1, .8, 1.]
    return colors


def stamp_seconds(stamp):
    return float(stamp.sec) + stamp.nanosec/1e9
