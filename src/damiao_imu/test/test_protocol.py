import binascii
import math
import struct
import pytest
from damiao_imu.protocol import Parser, unit_scales, dm_crc16


def frame(rid=2, values=(1., 2., 3.), device=1, mode='include_header'):
    data = bytes([0x55, 0xaa, device, rid]) + struct.pack('<' + 'f'*len(values), *values)
    crc = dm_crc16(data if mode == 'include_header' else data[2:])
    return data + crc.to_bytes(2, 'little') + b'\x0a'


def test_known_crc_and_units():
    assert binascii.crc_hqx(b'123456789', 0xffff) == 0x29b1
    assert unit_scales('deg_s', 'g') == (math.pi/180, 9.80665)
    with pytest.raises(ValueError):
        unit_scales('unset', 'm_s2')


@pytest.mark.parametrize('mode', ['include_header', 'exclude_header'])
def test_fragmented_multiple_frames(mode):
    parser = Parser(crc_mode=mode)
    stream = b'noise' + frame(mode=mode) + frame(4, (1., 0., 0., 0.), mode=mode) + frame(1, mode=mode)
    result = []
    for byte in stream:
        result.extend(parser.feed(bytes([byte])))
    assert [rid for rid, _ in result] == [2, 4, 1]
    assert parser.feed(b'') == []


def test_corruption_resync_and_wrong_device():
    parser = Parser()
    corrupt = bytearray(frame()); corrupt[8] ^= 1
    result = parser.feed(corrupt + frame(device=9) + frame(values=(float('nan'), 0., 0.)) + frame())
    assert result == [(2, (1., 2., 3.))]
    assert parser.errors > 0
    parser.feed(b'x'*100000 + b'\x55')
    assert len(parser.buffer) <= 3
    assert parser.feed(frame()[1:]) == [(2, (1., 2., 3.))]


def test_tail_and_crc_mode_are_enforced():
    assert Parser().feed(frame()[:-1] + b'x' + frame()) == [(2, (1., 2., 3.))]
    assert Parser(crc_mode='exclude_header').feed(frame()) == []


def test_hardware_capture_crc():
    # Independent golden frame from connected DM-IMU-L1, 2026-09-24.
    packet = bytes.fromhex('55aa010168e994bebc04dd3e32882041b7550a')
    assert dm_crc16(packet[:-3]) == 0x55b7
    assert Parser().feed(packet)[0][0] == 1
