"""Runtime TF must use configured mount, rather than a horizontal optical constant."""
import math
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from vio_sensor_transforms import quaternion_from_rpy
from assess_vio_motion_evidence import rotation


def test_original_horizontal_optical_transform_retained():
    assert np.allclose(quaternion_from_rpy([-math.pi/2,0,-math.pi/2]),[-.5,.5,-.5,.5])
    assert quaternion_from_rpy([0,0,0])==(0,0,0,1)


def test_runtime_tf_matches_pitched_rendering():
    theta=math.radians(15)
    q=quaternion_from_rpy([-math.pi/2-theta,0,-math.pi/2])
    ry=np.array([[math.cos(theta),0,math.sin(theta)],[0,1,0],[-math.sin(theta),0,math.cos(theta)]])
    optical=np.array([[0,0,1],[-1,0,0],[0,-1,0]])
    assert np.allclose(rotation(q),ry@optical)
    assert not np.allclose(q,[-.5,.5,-.5,.5])
