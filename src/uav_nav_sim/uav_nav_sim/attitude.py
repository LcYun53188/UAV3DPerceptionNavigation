"""Level attitude for the kinematic Gazebo model, not a rotor flight controller."""
import math

import numpy as np


def level_body_rates(rotation, yaw_rate=0.0):
    """Return body angular velocity for level roll/pitch and an ENU yaw rate.

    Euler angle rates cannot be sent as body rates while the model is tilted:
    doing so couples a yaw scan into roll and pitch. Convert all three together.
    """
    roll, pitch, _ = rotation.as_euler('xyz')
    roll_rate = float(np.clip(-2.0*roll, -.4, .4))
    pitch_rate = float(np.clip(-2.0*pitch, -.4, .4))
    return np.array([
        roll_rate-yaw_rate*math.sin(pitch),
        pitch_rate*math.cos(roll)+yaw_rate*math.sin(roll)*math.cos(pitch),
        -pitch_rate*math.sin(roll)+yaw_rate*math.cos(roll)*math.cos(pitch),
    ])
