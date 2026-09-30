"""Conservative, bounded selection of observation positions (no ROS dependency).

Unknown space is never a flight corridor. EGO and the executor still validate
all paths; connectivity here only filters candidates, it does not authorize motion.
"""
from dataclasses import dataclass

import numpy as np
from scipy.ndimage import maximum_filter, label, generate_binary_structure

from .core import CellState


@dataclass(frozen=True)
class ExplorationSettings:
    step_radius: float = 2.0
    detour_budget: float = 2.0
    revisit_radius: float = 0.45
    observation_time: float = 1.5
    blocked_timeout: float = 15.0
    progress_timeout: float = 60.0
    planning_timeout: float = 8.0
    max_failures: int = 4
    max_segments: int = 40

    def __post_init__(self):
        for name, value in vars(self).items():
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if self.step_radius <= self.revisit_radius:
            raise ValueError('step_radius must exceed revisit_radius')


def segment_free(grid, start, end, radius):
    length = np.linalg.norm(end-start)
    count = max(1, int(np.ceil(length/(grid.resolution/2))))
    padding = length/count/2
    return all(not grid.collision(p, radius+padding)
               for p in np.linspace(start, end, count+1))


def choose_subgoal(grid, start, goal, radius, settings, visited=(), rejected=(),
                   explore=True, best_distance=None):
    """Return (position, is_final), or None when no safe candidate remains.

    A six-connected, eroded free component excludes inaccessible frontiers.
    Frontier viewpoints are set back from unknown voxels by the body clearance.
    Progress can temporarily move away from the goal within detour_budget.
    """
    start, goal = np.asarray(start, dtype=float), np.asarray(goal, dtype=float)
    if grid.collision(start, radius) or grid.state(goal) == CellState.OCCUPIED:
        return None
    if not grid.collision(goal, radius) and segment_free(grid, start, goal, radius):
        if not any(np.linalg.norm(goal-p) < settings.revisit_radius for p in rejected):
            return goal.copy(), True
    if not explore:
        # Let EGO find a detour in the known map even when the direct ray is blocked.
        if not grid.collision(goal, radius) and not rejected:
            return goal.copy(), True
        return None

    r = grid.resolution
    # Extra half-voxel padding conservatively covers axis-adjacent center edges.
    padding = int(np.ceil((radius+r/2)/r))
    unknown = ~grid.observed | ~np.isfinite(grid.distance)
    near_unknown = maximum_filter(unknown, size=2*padding+1, mode='constant', cval=1)
    safe = ~near_unknown & (grid.distance > radius + (np.sqrt(3)+1)*r/2)
    components, _ = label(safe, generate_binary_structure(3, 1))
    index = grid.index(start)
    component = components[tuple(index)]
    if component == 0:
        # Actual start need not be at a voxel center. Connect it to a nearby safe
        # center by a checked segment, rather than assuming the nearest is safe.
        lo = np.maximum(0, index-2)
        hi = np.minimum(safe.shape, index+3)
        indices = np.argwhere(safe[tuple(slice(a, b) for a, b in zip(lo, hi))])+lo
        for i in sorted(indices, key=lambda i: np.linalg.norm(grid.origin+(i+.5)*r-start)):
            if segment_free(grid, start, grid.origin+(i+.5)*r, radius):
                component = components[tuple(i)]
                break
    if component == 0:
        return None
    connected = components == component
    gi = grid.index(goal)
    if (gi is not None and connected[tuple(gi)] and not grid.collision(goal, radius)
            and not any(np.linalg.norm(goal-p) < settings.revisit_radius for p in rejected)):
        return goal.copy(), True

    # Outside the configured map is not a frontier: increasing the map extent
    # requires an explicit launch setting, not exploration beyond map coverage.
    fringe = maximum_filter(unknown, size=2*(padding+2)+1, mode='constant', cval=0)
    frontier_indices = np.argwhere(connected & fringe)
    if len(frontier_indices) == 0:
        return None
    frontiers = grid.origin+(frontier_indices+.5)*r
    baseline = np.linalg.norm(goal-start) if best_distance is None else best_distance

    def eligible(points):
        keep = np.linalg.norm(points-goal, axis=1) <= baseline+settings.detour_budget
        for point in (*visited, *rejected):
            keep &= np.linalg.norm(points-point, axis=1) >= settings.revisit_radius
        return keep

    frontiers = frontiers[eligible(frontiers)]
    if len(frontiers) == 0:
        return None
    distance = np.linalg.norm(frontiers-start, axis=1)
    # Favor target-directed frontiers, with a small travel cost to avoid jumping
    # between distant sides of the same boundary.
    score = np.linalg.norm(frontiers-goal, axis=1)+0.2*distance
    local = (distance <= settings.step_radius) & (distance >= settings.revisit_radius)
    if np.any(local):
        candidates = frontiers[local]
        candidate = candidates[np.argmin(score[local])]
    else:
        destination = frontiers[np.argmin(score)]
        # An intermediate point inside the known component approaches a farther
        # frontier. EGO handles any obstacle detour between these safe endpoints.
        indices = np.argwhere(connected)
        points = grid.origin+(indices+.5)*r
        distance = np.linalg.norm(points-start, axis=1)
        keep = eligible(points) & (distance <= settings.step_radius) & (distance >= settings.revisit_radius)
        points = points[keep]
        if len(points) == 0:
            return None
        score = np.linalg.norm(points-destination, axis=1)
        candidate = points[np.argmin(score)]
        if np.linalg.norm(candidate-destination) >= np.linalg.norm(start-destination)-r:
            return None
    return candidate.copy(), False
