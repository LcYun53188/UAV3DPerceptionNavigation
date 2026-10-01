import numpy as np
from uav_nav_sim.core import Grid
from uav_nav_sim.autonomous import frontier_viewpoint


def scene():
    observed = np.zeros((60, 40, 30), dtype=bool)
    observed[:35] = True
    return Grid(np.array([-2., -4., -2.]), .2, np.full(observed.shape, 3.), observed)


def test_selects_known_frontier_without_user_goal():
    g = scene()
    point = frontier_viewpoint(g, [0., 0., 0.], .3, preferred_height=0.)
    assert point is not None and point[0] > 2.
    assert not g.collision(point, .4)


def test_disconnected_frontier_is_not_selected():
    g = scene()
    observed = g.observed.copy()
    observed[:20] = True
    distance = g.distance.copy()
    distance[20:23] = 0.
    g = Grid(g.origin.copy(), .2, distance, observed)
    assert frontier_viewpoint(g, [0.,0.,0.], .3) is None


def test_fully_known_map_does_not_treat_outside_as_frontier():
    g = scene()
    g = Grid(g.origin.copy(), .2, g.distance.copy(), np.ones(g.observed.shape, dtype=bool))
    assert frontier_viewpoint(g, [0.,0.,0.], .3) is None


def test_blocked_start_and_rejected_viewpoints_are_respected():
    g = scene()
    assert frontier_viewpoint(g, [6.,0.,0.], .3) is None
    first = frontier_viewpoint(g, [0.,0.,0.], .3)
    second = frontier_viewpoint(g, [0.,0.,0.], .3, rejected=[first])
    assert second is None or np.linalg.norm(first-second) >= 1.
