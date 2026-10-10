import math
import numpy as np
from analyze_observation_grid import visible,volume_indices


def test_camera_sweep_has_vertical_blind_regions_and_near_clip():
    profile=dict(camera_pitch_deg=5,horizontal_fov_rad=1.21,height=400,width=640,
                 rig_position_flu_m=[0,0,0],near_m=.1,far_m=20.)
    points=np.array([[3,0,0],[0,3,0],[0,0,1],[0,0,-1],[.05,0,0],[3,0,3]])
    assert visible(points,[0,0,0],profile).tolist()==[True,True,False,False,False,False]


def test_indices_cover_same_inclusive_braking_box():
    grid=dict(origin=np.zeros(3),resolution=np.array(1.),distance=np.ones((5,5,5)))
    indices=volume_indices(grid,[1,1,1],1.,1.)
    assert indices.shape==(48,3)
    assert indices.min(axis=0).tolist()==[0,0,0]
    assert indices.max(axis=0).tolist()==[3,3,2]
