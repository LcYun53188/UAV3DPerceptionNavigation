"""Parameters compatible with the reviewed cuVSLAM 15 stream sequencer."""


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
