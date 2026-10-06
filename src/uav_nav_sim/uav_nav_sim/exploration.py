"""Conservative, bounded selection of observation positions (no ROS dependency).

Unknown space is never a flight corridor. EGO and the executor still validate
all paths; connectivity here only filters candidates, it does not authorize motion.
"""
from dataclasses import dataclass

import numpy as np
from scipy.ndimage import maximum_filter, label, generate_binary_structure
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

from .core import CellState
from .viewpoints import goal_unknown_points, view_gain, visible_gain


@dataclass(frozen=True)
class ExplorationSettings:
    step_radius: float = 3.0
    detour_budget: float = 2.0
    revisit_radius: float = 0.45
    observation_time: float = 1.5
    blocked_timeout: float = 15.0
    progress_timeout: float = 60.0
    planning_timeout: float = 8.0
    max_failures: int = 4
    max_segments: int = 40
    clearance_weight: float = 0.4
    altitude_weight: float = 0.75
    minimum_map_gain: float = 0.5
    goal_view_weight: float = 2.0

    def __post_init__(self):
        for name, value in vars(self).items():
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if self.step_radius <= self.revisit_radius:
            raise ValueError('step_radius must exceed revisit_radius')


def segment_free(grid, start, end, radius, maximum_checks=4096):
    """Certify covered intervals, refining padding near voxel boundaries.

    A free midpoint box enlarged by half the interval length covers the
    entire swept body. Uncertified intervals subdivide at most six times;
    exhausted work or any blocked body sample still rejects the segment.
    """
    start, end = np.asarray(start), np.asarray(end)
    if (not np.all(np.isfinite(start)) or not np.all(np.isfinite(end)) or
            grid.collision(start, radius) or grid.collision(end, radius)):
        return False
    length = np.linalg.norm(end-start)
    if not np.isfinite(length):
        return False
    count = max(1, int(np.ceil(length/(grid.resolution/2))))
    if count > maximum_checks-2:
        return False
    checks = 2
    for i in range(count):
        stack = [(start+(end-start)*(i/count), start+(end-start)*((i+1)/count), 0)]
        while stack:
            a, b, depth = stack.pop()
            middle = (a+b)/2
            if checks >= maximum_checks:
                return False
            checks += 1
            if not grid.collision(middle, radius+np.linalg.norm(b-a)/2):
                continue
            if depth >= 6 or checks >= maximum_checks:
                return False
            checks += 1
            if grid.collision(middle, radius):
                return False
            stack.extend([(middle, b, depth+1), (a, middle, depth+1)])
    return True


def reachable_routes(connected, start_index, traversal_cost=None):
    """Six-connected route costs/predecessors in the eroded known-free map.

    Symmetric edge costs prefer roomy corridors while retaining narrow routes.
    """
    indices = np.argwhere(connected)
    ids = np.full(connected.shape, -1, dtype=np.int32)
    ids[connected] = np.arange(len(indices), dtype=np.int32)
    rows, cols = [], []
    for axis in range(3):
        left, right = [slice(None)]*3, [slice(None)]*3
        left[axis], right[axis] = slice(None, -1), slice(1, None)
        a, b = ids[tuple(left)], ids[tuple(right)]
        edges = (a >= 0) & (b >= 0)
        rows.append(a[edges]); cols.append(b[edges])
    rows, cols = np.concatenate(rows), np.concatenate(cols)
    weights = np.ones(len(rows))
    if traversal_cost is not None:
        costs = traversal_cost[connected]
        weights = (costs[rows]+costs[cols])/2
    graph = csr_matrix((weights, (rows, cols)), shape=(len(indices), len(indices)))
    distances, predecessors = dijkstra(graph, directed=False, indices=int(ids[tuple(start_index)]),
                                       return_predecessors=True)
    return ids, indices, distances, predecessors


def choose_subgoal(grid, start, goal, radius, settings, visited=(), rejected=(),
                   explore=True, best_distance=None, continuous=False, diagnostics=None):
    """Return (position, is_final), or None when no safe candidate remains.

    A six-connected, eroded free component excludes inaccessible frontiers.
    Frontier viewpoints are set back from unknown voxels by the body clearance.
    Progress can temporarily move away from the goal within detour_budget.
    """
    start, goal = np.asarray(start, dtype=float), np.asarray(goal, dtype=float)
    if diagnostics is not None:
        diagnostics['reason'] = 'NO_REACHABLE_FRONTIER'
    # Match the planner's seed clearance. Selecting viewpoints with body-only
    # clearance can leave EGO unable to connect them to its search lattice.
    radius += grid.resolution*0.5
    if grid.collision(start, radius) or grid.state(goal) == CellState.OCCUPIED:
        if diagnostics is not None and grid.collision(start, radius):
            diagnostics['reason'] = 'START_VOLUME_BLOCKED'
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
    near_blocked = maximum_filter(unknown | (grid.distance <= 0),
                                  size=2*padding+1, mode='constant', cval=1)
    safe = ~near_blocked & (grid.distance > radius + (np.sqrt(3)+1)*r/2)
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
                index = i
                break
    if component == 0:
        if diagnostics is not None:
            diagnostics['reason'] = 'START_NOT_CONNECTED'
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

    keep = eligible(frontiers) & (np.linalg.norm(frontiers-start, axis=1) >= settings.revisit_radius)
    frontiers, frontier_indices = frontiers[keep], frontier_indices[keep]
    if len(frontiers) == 0:
        return None
    # Clearance is a preference, never a substitute for observed-volume checks.
    # Saturate at two voxels of spare space to avoid singular boundary costs.
    clearance_cost = settings.clearance_weight / np.maximum(grid.distance-radius, 2*r)
    ids, indices, distances, predecessors = reachable_routes(connected, index, 1+clearance_cost)
    frontier_ids = ids[tuple(frontier_indices.T)]
    # Rank the whole reachable frontier, not just viewpoints within one step.
    # Route distance accounts for obstacles; a soft neighborhood penalty avoids
    # visiting many almost-identical viewpoints along the same wall.
    score = (np.linalg.norm(frontiers-goal, axis=1)+0.2*distances[frontier_ids]*r
             + settings.altitude_weight*np.abs(frontiers[:, 2]-goal[2])
             + clearance_cost[tuple(frontier_indices.T)])
    for point in visited:
        score += 1.5*np.exp(-np.sum((frontiers-point)**2, axis=1)/(2*max(settings.revisit_radius, .8)**2))
    # A free centre can still have unseen body voxels, especially above a
    # downward camera. Prefer viewpoints that can observe those missing cells.
    # This changes ranking only: all flight candidates remain in the same safe
    # connected component, and occupied sight lines get no observation credit.
    targets = (goal_unknown_points(grid, goal, radius+r/2)
               if grid.state(goal) == CellState.FREE else np.empty((0, 3)))
    if len(targets):
        estimated = score-settings.goal_view_weight*view_gain(frontiers, goal, targets)
        shortlist = np.argsort(estimated)[:32]
        adjusted = score.copy()
        for candidate in shortlist:
            adjusted[candidate] -= settings.goal_view_weight*visible_gain(
                grid, frontiers[candidate], goal, targets)
        score = adjusted
    connector = np.linalg.norm(grid.origin+(index+.5)*r-start)
    for destination in np.argsort(score)[:128]:
        cursor = int(frontier_ids[destination])
        if continuous:
            # The entire connected route is known. Let the trajectory planner
            # traverse it in one curve instead of stopping at artificial horizons.
            return frontiers[destination].copy(), False
        # The horizon is physical route length, not clearance-weighted cost.
        # Transit may revisit old points when a detour needs to backtrack.
        route = []
        while cursor >= 0:
            route.append(cursor)
            cursor = int(predecessors[cursor])
        horizon = int(np.floor((settings.step_radius-connector)/r))
        for cursor in route[::-1][:max(0, horizon+1)][::-1]:
            candidate = grid.origin+(indices[cursor]+.5)*r
            distance = np.linalg.norm(candidate-start)
            if (distance >= settings.revisit_radius and
                    not any(np.linalg.norm(candidate-p) < settings.revisit_radius for p in rejected)):
                return candidate.copy(), False
    return None
