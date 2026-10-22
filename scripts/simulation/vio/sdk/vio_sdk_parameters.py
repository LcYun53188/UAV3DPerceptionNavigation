"""Parameters compatible with the reviewed cuVSLAM 15 stream sequencer."""

# Support canonical direct execution and historical flat imports.
import sys as _sys
from pathlib import Path as _Path
_scripts = str(_Path(__file__).resolve().parents[3])
if _scripts not in _sys.path:
    _sys.path.insert(0, _scripts)
from support.paths import enable_imports as _enable_imports
_enable_imports()



def validate_buffers(parameters):
    # MessageStreamSequencer accepts uint8_t sizes before constructing int buffers.
    for name in ('image_buffer_size','imu_buffer_size'):
        value=parameters[name]
        if type(value) is not int or not 1<=value<=255:
            raise ValueError(name+' must fit the SDK uint8_t capacity (1..255)')


def stream_parameters(image_depth=10):
    if type(image_depth) is not int or not 1<=image_depth<=10:
        raise ValueError('SDK image depth must be an integer within 1..10')
    # Earlier 400 implicitly became 144. State that actual capacity explicitly.
    parameters=dict(image_buffer_size=100,imu_buffer_size=144,image_qos='DEFAULT',image_qos_depth=image_depth)
    validate_buffers(parameters)
    return parameters
