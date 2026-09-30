import numpy as np
import pytest

from uav_nav_sim.core import Grid
from uav_nav_sim.exploration import ExplorationSettings, choose_subgoal, segment_free


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
    assert not final and 0.45 < p[0] <= 2.
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


@pytest.mark.parametrize('kwargs', [{'step_radius': 0.}, {'blocked_timeout': float('nan')},
                                    {'max_segments': -1}, {'step_radius': .1}])
def test_invalid_exploration_settings(kwargs):
    with pytest.raises(ValueError):
        ExplorationSettings(**kwargs)
