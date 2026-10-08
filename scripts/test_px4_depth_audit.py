"""Validate sensor acceptance against malformed, stale and mismatched inputs."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from px4_depth_audit import DepthAudit


def messages(stamp=1, bigendian=False):
    header = NS(stamp=NS(sec=stamp, nanosec=0), frame_id='reference_depth_optical')
    data = np.full((480, 640), 2., dtype='>f4' if bigendian else '<f4')
    data[0, 0] = np.inf
    image = NS(header=header, width=640, height=480, encoding='32FC1',
               step=2560, is_bigendian=bigendian, data=data.tobytes())
    f = 640 / (2 * math.tan(1.274 / 2))
    info = NS(header=NS(stamp=header.stamp, frame_id=header.frame_id), width=640, height=480,
              k=[f, 0., 320., 0., f, 240., 0., 0., 1.],
              p=[f, 0., 320., 0., 0., f, 240., 0., 0., 0., 1., 0.], r=[1., 0., 0., 0., 1., 0., 0., 0., 1.], d=[])
    return image, info


def populated():
    audit = DepthAudit()
    for stamp in range(1, 6):
        image, info = messages(stamp)
        audit.image(image, 10.)
        audit.info(info, 10.)
    return audit


WRITERS = {'/px4_depth/image': 1, '/px4_depth/camera_info': 1}


@pytest.mark.parametrize('endian', [False, True])
def test_metric_depth_and_infinite_background(endian):
    audit = populated()
    image, info = messages(6, endian)
    audit.image(image, 10.)
    audit.info(info, 10.)
    result = audit.result(10.1, 6.1, WRITERS)
    assert result['passed']
    assert result['image']['finite_pixels'] == 640 * 480 - 1
    assert result['image']['min_m'] == 2.


@pytest.mark.parametrize('mutation', [
    lambda m: setattr(m, 'encoding', '16UC1'),
    lambda m: setattr(m, 'step', 100),
    lambda m: setattr(m, 'data', b''),
    lambda m: setattr(m.header, 'frame_id', ''),
    lambda m: setattr(m, 'data', np.full((480, 640), np.nan, dtype='<f4').tobytes()),
    lambda m: setattr(m, 'data', np.zeros((480, 640), dtype='<f4').tobytes()),
])
def test_bad_image_latches_failure(mutation):
    audit = populated()
    image, _ = messages(6)
    mutation(image)
    audit.image(image, 10.)
    assert not audit.result(10.1, 5.1, WRITERS)['passed']


def test_duplicate_timestamp_does_not_refresh():
    audit = populated()
    image, _ = messages(5)
    audit.image(image, 20.)
    assert audit.last_image_receive == 10.
    assert audit.regressions == 1
    assert not audit.result(20.1, 5.1, WRITERS)['passed']


@pytest.mark.parametrize('now,clock,writers', [(12., 5.1, WRITERS), (10.1, 7., WRITERS),
                                             (10.1, 4.8, WRITERS), (10.1, None, WRITERS),
                                             (10.1, 5.1, {})])
def test_stale_or_missing_source_rejected(now, clock, writers):
    assert not populated().result(now, clock, writers)['passed']


def test_calibration_mismatch():
    audit = populated()
    _, info = messages(6)
    info.k[0] *= .5
    audit.info(info, 10.)
    assert not audit.result(10.1, 5.1, WRITERS)['passed']


def test_distinct_frames_rejected():
    audit = populated()
    _, info = messages(5)
    info.header.frame_id = 'other_camera'
    audit.info(info, 10.)
    assert not audit.result(10.1, 5.1, WRITERS)['passed']


def test_no_data_is_failure():
    assert not DepthAudit().result(10., 5., WRITERS)['passed']


def test_duplicate_publisher_rejected():
    writers = dict(WRITERS, **{'/px4_depth/image': 2})
    assert not populated().result(10.1, 5.1, writers)['passed']


def test_padded_rows_use_step():
    audit = populated()
    image, info = messages(6)
    row = np.full(641, 2., dtype='<f4')
    row[-1] = -999.
    image.step = 641 * 4
    image.data = row.tobytes() * 480
    audit.image(image, 10.)
    audit.info(info, 10.)
    assert audit.result(10.1, 6.1, WRITERS)['passed']
    assert audit.last_image['finite_pixels'] == 640 * 480


@pytest.mark.parametrize('field,index,value', [('p', 2, 300.), ('k', 8, 0.),
                                               ('r', 0, -1.), ('d', 0, .1)])
def test_full_calibration_mismatch(field, index, value):
    audit = populated()
    _, info = messages(6)
    if field == 'd':
        info.d = [0.] * 5
    getattr(info, field)[index] = value
    audit.info(info, 10.)
    assert not audit.result(10.1, 5.1, WRITERS)['passed']
