"""Sensor mount rotations shared by generated geometry and runtime TF."""
import math


def quaternion_from_rpy(rpy):
    if len(rpy)!=3 or not all(math.isfinite(v) for v in rpy):
        raise ValueError('Expected finite roll/pitch/yaw')
    roll,pitch,yaw=(v/2 for v in rpy)
    cr,sr=math.cos(roll),math.sin(roll)
    cp,sp=math.cos(pitch),math.sin(pitch)
    cy,sy=math.cos(yaw),math.sin(yaw)
    return (sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy)
