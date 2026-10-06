"""Observation estimates for the fixed Gazebo depth camera, never flight clearance."""
import numpy as np


def goal_unknown_points(grid, goal, radius, maximum=64):
    lo = np.floor((goal-radius-grid.origin)/grid.resolution).astype(int)
    hi = np.floor((goal+radius-grid.origin)/grid.resolution).astype(int)
    if np.any(lo < 0) or np.any(hi >= grid.distance.shape):
        return np.empty((0, 3))
    region = tuple(slice(a, b+1) for a, b in zip(lo, hi))
    indices = np.argwhere(~grid.observed[region] | ~np.isfinite(grid.distance[region]))+lo
    if len(indices) > maximum:
        indices = indices[np.linspace(0, len(indices)-1, maximum, dtype=int)]
    return grid.origin+(indices+.5)*grid.resolution


def view_gain(points, goal, targets):
    """Fraction in view when level and yawed toward goal.

    Matches uav_quad_mid360 camera: 0.18 m forward, 0.16 m up, 5 deg
    downward, 1.21 rad horizontal FOV, 640x400. Reserve angular margins.
    """
    points = np.asarray(points)
    direction = goal[:2]-points[:, :2]
    direction /= np.maximum(np.linalg.norm(direction, axis=1, keepdims=True), 1e-9)
    camera = points.copy()
    camera[:, :2] += .18*direction
    camera[:, 2] += .16
    gain = np.zeros(len(points))
    for target in targets:
        delta = target-camera
        forward = np.sum(delta[:, :2]*direction, axis=1)
        lateral = delta[:, 0]*direction[:, 1]-delta[:, 1]*direction[:, 0]
        vertical = np.arctan2(delta[:, 2], forward)+np.deg2rad(5.)
        gain += ((forward >= .3) & (forward <= 6.) &
                 (np.abs(np.arctan2(lateral, forward)) <= .55) &
                 (np.abs(vertical) <= .36))
    return gain/max(1, len(targets))


def visible_gain(grid, point, goal, targets):
    """Reject rays blocked by known occupied voxels; unseen rays estimate gain only."""
    direction = goal[:2]-point[:2]
    direction /= max(np.linalg.norm(direction), 1e-9)
    camera = point+np.array([.18*direction[0], .18*direction[1], .16])
    delta = targets-camera
    forward = delta[:, :2] @ direction
    lateral = delta[:, 0]*direction[1]-delta[:, 1]*direction[0]
    vertical = np.arctan2(delta[:, 2], forward)+np.deg2rad(5.)
    in_view = ((forward >= .3) & (forward <= 6.) &
               (np.abs(np.arctan2(lateral, forward)) <= .55) &
               (np.abs(vertical) <= .36))
    visible = 0
    for target in targets[in_view]:
        count = max(1, int(np.ceil(np.linalg.norm(target-camera)/(grid.resolution/2))))
        ray = np.linspace(camera, target, count+1)
        indices = np.floor((ray-grid.origin)/grid.resolution).astype(int)
        if np.any(indices < 0) or np.any(indices >= np.array(grid.distance.shape)):
            continue
        observed = grid.observed[tuple(indices.T)]
        distance = grid.distance[tuple(indices.T)]
        if not np.any(observed & np.isfinite(distance) & (distance <= 0)):
            visible += 1
    return visible/max(1, len(targets))


def focused_scan_yaw(grid, position, goal, radius, yaw):
    """Choose one useful first view; return None for the ordinary sweep.

    Samples are bounded and never authorize motion. Include goal body gaps
    and gaps along the first four metres of the direct corridor.
    """
    targets = goal_unknown_points(grid, goal, radius+grid.resolution/2, maximum=24)
    delta = goal-position
    length = np.linalg.norm(delta)
    if length > 1e-6:
        corridor = [goal_unknown_points(grid, position+delta/length*d,
                    radius+grid.resolution/2, maximum=8)
                    for d in np.linspace(.8, min(4., length), 5) if d <= length]
        if corridor:
            targets = np.concatenate([targets, *corridor])
    if not len(targets):
        return None
    targets = np.unique(targets, axis=0)
    angles = np.arctan2(targets[:, 1]-position[1], targets[:, 0]-position[0])
    candidates = np.r_[yaw, angles[np.linspace(0, len(angles)-1,
                                             min(8, len(angles)), dtype=int)]]
    best, score = None, -float('inf')
    for angle in candidates:
        direction_goal = position+np.array([np.cos(angle), np.sin(angle), 0.])
        gain = visible_gain(grid, position, direction_goal, targets)
        if gain <= 0:
            continue
        turn = abs(np.arctan2(np.sin(angle-yaw), np.cos(angle-yaw)))
        value = gain-.08*turn/np.pi
        if value > score:
            best, score = float(angle), value
    return best
