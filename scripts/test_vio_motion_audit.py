"""The error audit must reject drift, scaling, timing gaps, and missing axes."""
import copy
import math
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from px4_vio_motion_audit import assess_motion,rotation


def dataset():
    truth,poses,imu = [],[],[]
    for i in range(8001):
        t = i/100
        xyz = [.5*math.sin(.45*t)**3,.4*math.sin(.55*t)**3,1.3+.3*math.sin(.65*t)**3]
        yaw = .4*math.sin(.4*t)**3
        q = [0,0,math.sin(yaw/2),math.cos(yaw/2)]
        truth.append(dict(stamp=t,position=xyz,quaternion=q))
        if i%4==0: poses.append(dict(stamp=t,position=xyz[:],quaternion=q[:]))
        imu.append(dict(stamp=t,values=[math.sin(t),math.sin(2*t),9.8+math.sin(3*t),0,0,.2*math.sin(t)]))
    return truth,poses,imu


def test_fixed_rigid_alignment_accepts_offset_and_rotation():
    truth,poses,imu = dataset()
    yaw = -.7
    r = rotation([0,0,math.sin(yaw/2),math.cos(yaw/2)])
    for p in poses:
        p['position'] = (r@p['position']+[10,-4,2]).tolist()
        q = p['quaternion']
        a = 2*math.atan2(q[2],q[3])+yaw
        p['quaternion'] = [0,0,math.sin(a/2),math.cos(a/2)]
    result = assess_motion(truth,poses,imu)
    assert result['passed'] and result['position_rmse_m']<1e-10
    assert result['orientation_max_deg']<1e-5


def test_drift_is_not_removed_by_best_fit():
    truth,poses,imu = dataset()
    for p in poses: p['position'][2] += p['stamp']*.01
    result = assess_motion(truth,poses,imu)
    assert not result['passed']
    assert not result['checks']['position_rmse'] and not result['checks']['position_max']


def test_scale_is_not_corrected():
    truth,poses,imu = dataset()
    for p in poses: p['position'] = (3*np.array(p['position'])).tolist()
    assert not assess_motion(truth,poses,imu)['checks']['position_rmse']


def test_source_retirement_and_gaps_fail_even_with_perfect_poses():
    truth,poses,imu = dataset()
    result = assess_motion(truth,poses[:-50],imu)
    assert not result['checks']['source_covers_end']
    result = assess_motion(truth,poses[:500]+poses[510:],imu)
    assert not result['checks']['source_continuous']


def test_missing_axes_and_rotation_fail():
    truth,poses,imu = dataset()
    for r in truth: r['position'][1] = 0; r['quaternion'] = [0,0,0,1]
    result = assess_motion(truth,poses,imu)
    assert not result['checks']['three_axis_excitation']
    assert not result['checks']['yaw_excitation']
    assert not result['checks']['orientation_max']


def test_disordered_or_missing_truth_fails():
    truth,poses,imu = dataset()
    assert not assess_motion([],poses,imu)['passed']
    truth[100]['stamp'] = truth[99]['stamp']
    assert not assess_motion(truth,poses,imu)['passed']
