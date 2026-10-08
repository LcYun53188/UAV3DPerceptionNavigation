"""Reject plausible but unsafe timestamp streams and incorrect IMU axes."""
import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parent))
from vio_sensor_quality import sample_window, stereo_pairs, static_imu


def samples(hz=25):
    return np.arange(5.,10.00001,1./hz).tolist()


def test_nominal_stream_and_asynchronous_observer_boundary():
    a = samples()
    assert sample_window(a,10.,25)['passed']
    # The right callback has not yet delivered the last frame.
    assert stereo_pairs(a,a[:-1],10.)['passed']
    assert static_imu([[0,0,9.8,0,0,0]]*1250)['passed']


@pytest.mark.parametrize('fault',['stale','duplicate','reversed','slow','burst','gap','nan','future'])
def test_timestamp_faults_rejected(fault):
    a = samples()
    if fault == 'stale': a = [t-1 for t in a]
    if fault == 'duplicate': a.insert(20,a[20])
    if fault == 'reversed': a[20],a[21] = a[21],a[20]
    if fault == 'slow': a = a[::2]
    if fault == 'burst': a = [9.99+i/10000 for i in range(100)]
    if fault == 'gap': del a[60:65]
    if fault == 'nan': a[10] = float('nan')
    if fault == 'future': a = [t+1 for t in a]
    assert not sample_window(a,10.,25)['passed']


def test_unsynchronized_and_one_sided_loss_rejected():
    a = samples()
    assert not stereo_pairs(a,[t+.01 for t in a],10.)['passed']
    assert not stereo_pairs(a,a[::2],10.)['passed']
    assert not stereo_pairs([],a,10.)['passed']


@pytest.mark.parametrize('imu',[[0,0,-9.8,0,0,0],[9.8,0,0,0,0,0],
                              [0,0,0,0,0,0],[0,0,9.8,0,0,1],
                              [0,0,9.8,float('nan'),0,0]])
def test_bad_static_imu_rejected(imu):
    assert not static_imu([imu]*1250)['passed']
