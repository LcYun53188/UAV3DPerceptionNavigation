import numpy as np
import pytest

from uav_nav_sim.core import Grid
from uav_nav_sim.exploration import ExplorationSettings, choose_subgoal, segment_free, reachable_routes


def grid(observed_until=25, wall=False, opening=False):
    shape = (50, 40, 20)
    observed = np.zeros(shape, dtype=bool)
    observed[:observed_until] = True
    distance = np.full(shape, 3., dtype=np.float32)
    if wall:
        # Exact distance to a slab spanning the map (or with a side opening).
        x = -2+(np.arange(shape[0])+.5)*.2
        dx = np.maximum(1.2-x, x-1.6)
        distance[:] = dx[:, None, None]
        if opening:
            y = -4+(np.arange(shape[1])+.5)*.2
            distance[:] = np.maximum(dx[:, None, None], (y-1.)[None, :, None])
    return Grid(np.array([-2., -4., -2.]), .2, distance, observed)


def test_unknown_target_uses_safe_forward_viewpoint():
    g = grid()
    p, final = choose_subgoal(g, [0., 0., 0.], [6., 0., 0.], .3, ExplorationSettings())
    assert not final and 0.45 < p[0] <= ExplorationSettings().step_radius
    assert not g.collision(p, .3)
    assert segment_free(g, np.zeros(3), p, .3)


def test_new_observations_allow_eventual_arrival():
    position = np.zeros(3)
    goal = np.array([6., 0., 0.])
    visited = []
    for edge in (25, 32, 40, 50):
        g = grid(edge)
        result = choose_subgoal(g, position, goal, .3, ExplorationSettings(), visited)
        assert result is not None
        point, final = result
        assert not g.collision(point, .3)
        assert segment_free(g, position, point, .3)
        visited.append(point)
        position = point
        if final:
            break
    assert final and np.allclose(position, goal)


def test_solid_wall_excludes_disconnected_frontiers():
    assert choose_subgoal(grid(wall=True), [0., 0., 0.], [6., 0., 0.], .3, ExplorationSettings()) is None


def test_known_detour_is_delegated_to_ego():
    g = grid(50, wall=True, opening=True)
    goal = np.array([6., 0., 0.])
    assert not segment_free(g, np.zeros(3), goal, .3)
    point, final = choose_subgoal(g, np.zeros(3), goal, .3, ExplorationSettings())
    assert final and np.allclose(point, goal)


def test_static_mode_does_not_explore_unknown_goal():
    assert choose_subgoal(grid(), [0., 0., 0.], [6., 0., 0.], .3, ExplorationSettings(), explore=False) is None


def test_unobserved_start_and_outside_boundary_never_authorize_motion():
    g = grid(observed_until=8)
    assert choose_subgoal(g, [0., 0., 0.], [6., 0., 0.], .3, ExplorationSettings()) is None
    assert choose_subgoal(grid(50), [0., 0., 0.], [20., 0., 0.], .3, ExplorationSettings()) is None


def test_rejected_candidate_is_not_reissued():
    g, settings = grid(), ExplorationSettings()
    first, _ = choose_subgoal(g, [0., 0., 0.], [6., 0., 0.], .3, settings)
    result = choose_subgoal(g, [0., 0., 0.], [6., 0., 0.], .3, settings, rejected=[first])
    assert result is None or np.linalg.norm(result[0]-first) >= settings.revisit_radius


def test_unknown_voxel_inside_body_is_not_treated_as_free():
    original = grid(50)
    observed = original.observed.copy()
    observed[10, 20, 10] = False
    g = Grid(original.origin.copy(), .2, original.distance.copy(), observed)
    assert choose_subgoal(g, [0., 0., 0.], [6., 0., 0.], .3, ExplorationSettings()) is None


def test_frontier_reserves_planner_seed_and_segment_clearance():
    original = grid()
    distance = original.distance.copy()
    # An attractive near-goal band is body-safe but too tight for EGO's
    # half-voxel seed margin plus swept-segment sampling padding.
    distance[17:25, 18:23, 8:13] = .60
    g = Grid(original.origin.copy(), .2, distance, original.observed.copy())
    point, final = choose_subgoal(g, [0., 0., 0.], [6., 0., 0.], .3, ExplorationSettings())
    assert not final
    assert g.distance[tuple(g.index(point))] > .3 + .2/2 + (np.sqrt(3)+1)*.2/2
    assert not g.collision(point, .3 + .2/2 + .2/4)


def test_exploration_prefers_level_viewpoint_over_small_low_altitude_shortcut():
    observed = np.zeros((50, 40, 30), dtype=bool)
    observed[:25] = True
    observed[18:25, :, 7:] = False
    g = Grid(np.array([-2., -4., -2.]), .2, np.full(observed.shape, 3.), observed)
    point, final = choose_subgoal(g, [0., 0., 0.], [6., 0., 0.], .3, ExplorationSettings())
    assert not final and point[0] > .45
    assert abs(point[2]) < .2
    assert segment_free(g, np.zeros(3), point, .4)


def test_nearby_side_frontier_does_not_distract_from_better_distant_frontier():
    original = grid(40)
    observed = original.observed.copy()
    observed[14:18,28:30,8:13] = False
    g = Grid(original.origin.copy(),.2,original.distance.copy(),observed)
    point, final = choose_subgoal(g,[0.,0.,0.],[7.,0.,0.],.3,ExplorationSettings())
    assert not final and point[0] > 1.4
    assert abs(point[1]) < .3
    assert segment_free(g,np.zeros(3),point,.4)


def test_routes_follow_known_corridor_around_wall():
    free = np.ones((9,9,1), dtype=bool)
    free[4,:7,:] = False
    ids, indices, distances, predecessors = reachable_routes(free,np.array([2,2,0]))
    cursor = ids[6,2,0]
    assert distances[cursor] > 4
    path = []
    while cursor >= 0:
        point = indices[cursor]
        assert free[tuple(point)]
        path.append(point)
        cursor = predecessors[cursor]
    assert any(point[1] >= 7 for point in path)
    assert all(np.sum(np.abs(a-b)) == 1 for a,b in zip(path,path[1:]))


def test_unknown_goal_starts_along_detour_instead_of_approaching_solid_wall():
    g = grid(25,wall=True,opening=True)
    point, final = choose_subgoal(g,[0.,0.,0.],[6.,0.,0.],.3,ExplorationSettings())
    assert not final and point[1] > 1.
    assert not g.collision(point,.4)


@pytest.mark.parametrize('kwargs', [{'step_radius': 0.}, {'blocked_timeout': float('nan')},
                                    {'max_segments': -1}, {'step_radius': .1}])
def test_invalid_exploration_settings(kwargs):
    with pytest.raises(ValueError):
        ExplorationSettings(**kwargs)


def test_route_cost_prefers_roomy_detour_but_keeps_narrow_only_route():
    free = np.ones((7, 5, 1), dtype=bool)
    costs = np.ones(free.shape)
    costs[1:6, 2, 0] = 8.
    ids, indices, distances, predecessors = reachable_routes(free, [0, 2, 0], costs)
    cursor = ids[6, 2, 0]
    route = []
    while cursor >= 0:
        route.append(indices[cursor])
        cursor = predecessors[cursor]
    assert any(p[1] != 2 for p in route)
    assert distances[ids[6, 2, 0]] < 6*8
    free[:, :2] = False
    free[:, 3:] = False
    ids, _, distances, _ = reachable_routes(free, [0, 2, 0], costs)
    assert np.isfinite(distances[ids[6, 2, 0]])


def test_clearance_cost_does_not_shorten_physical_step_horizon():
    g = grid()
    # A uniformly tight but traversable volume: adding the same large cost
    # everywhere must not turn a three-metre horizon into a tiny step.
    g = Grid(g.origin.copy(), g.resolution, np.full(g.distance.shape, .8), g.observed.copy())
    point, final = choose_subgoal(g, [0., 0., 0.], [6., 0., 0.], .3,
                                  ExplorationSettings(clearance_weight=4.))
    assert not final and point[0] > 1.5
    assert not g.collision(point, .4)


def test_viewpoint_prefers_clearance_over_small_goal_distance_advantage():
    g = grid()
    distance = g.distance.copy()
    distance[:, :22, :] = .8
    g = Grid(g.origin.copy(), g.resolution, distance, g.observed.copy())
    point, final = choose_subgoal(g, [0., 0., 0.], [6., 0., 0.], .3, ExplorationSettings())
    assert not final and point[0] > 1.5
    assert g.distance[tuple(g.index(point))] > 1.
