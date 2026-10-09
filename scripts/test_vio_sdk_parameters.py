import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parent))
from vio_sdk_parameters import stream_parameters,validate_buffers


@pytest.mark.parametrize('value',[0,256,400,-1,1.5,True])
def test_native_capacity_overflow_or_invalid_size_rejected(value):
    with pytest.raises(ValueError):validate_buffers(dict(image_buffer_size=100,imu_buffer_size=value))


def test_explicit_capacity_matches_previous_native_conversion():
    parameters=stream_parameters()
    assert parameters['imu_buffer_size']==400%256
    assert parameters['image_qos_depth']==10
    assert stream_parameters(1)['image_qos_depth']==1


@pytest.mark.parametrize('value',[0,11,-1,1.5,True])
def test_invalid_queue_depth_rejected(value):
    with pytest.raises(ValueError):stream_parameters(value)
