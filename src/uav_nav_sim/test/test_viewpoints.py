import numpy as np

from uav_nav_sim.core import Grid
from uav_nav_sim.exploration import ExplorationSettings, choose_subgoal
from uav_nav_sim.viewpoints import goal_unknown_points, view_gain, visible_gain


def scene():
    return Grid(np.array([-3., -3., -1.]), .1, np.full((60,60,50), 3.),
                np.ones((60,60,50), dtype=bool))


def test_downward_camera_needs_standoff_to_see_upper_body_gap():
    goal = np.array([0., 0., 1.])
    target = np.array([[0., 0., 1.4]])
    points = np.array([[-.5, 0., 1.], [-1.5, 0., 1.], [-.7, 0., 1.4]])
    assert np.array_equal(view_gain(points, goal, target), [0., 1., 1.])


def test_known_obstacle_occludes_gain_but_unknown_ray_does_not_authorize_flight():
    g = scene();goal = np.array([0.,0.,1.]);point = np.array([-1.5,0.,1.])
    target = np.array([[0.,0.,1.3]])
    distance, observed = g.distance.copy(), g.observed.copy()
    distance[20:22,29:32,21:24] = 0.
    blocked = Grid(g.origin.copy(),g.resolution,distance,observed)
    assert visible_gain(blocked,point,goal,target) == 0.
    observed = observed.copy()
    observed[20:22,29:32,21:24] = False
    unseen = Grid(g.origin.copy(),g.resolution,distance,observed)
    assert visible_gain(unseen,point,goal,target) == 1.
    assert unseen.collision([- .85,0.,1.2], .3)


def test_goal_gap_targets_are_bounded_and_stay_inside_goal_volume():
    g = scene();observed = g.observed.copy();observed[25:35,25:35,15:25] = False
    g = Grid(g.origin.copy(),g.resolution,g.distance.copy(),observed)
    targets = goal_unknown_points(g,np.array([0.,0.,1.]),.35,maximum=8)
    assert len(targets) == 8
    assert np.all(np.abs(targets-[0.,0.,1.]) <= .4)
    assert len(goal_unknown_points(g,np.array([10.,0.,1.]),.35)) == 0


def test_upper_gap_selects_reachable_viewpoint_with_observation_gain():
    g = scene();observed = g.observed.copy();observed[30,30,23] = False
    g = Grid(g.origin.copy(),g.resolution,g.distance.copy(),observed)
    start,goal = np.array([-1.5,0.,1.]),np.array([0.,0.,1.])
    point,final = choose_subgoal(g,start,goal,.3,ExplorationSettings(),continuous=True)
    assert not final and not g.collision(point,.35)
    targets = goal_unknown_points(g,goal,.4)
    assert visible_gain(g,point,goal,targets) > 0
