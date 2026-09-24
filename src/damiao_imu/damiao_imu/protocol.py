"""DM USB framing; protocol reference pinned in docs/DAMIAO_IMU_USB.md."""
import binascii
import math
import struct


def dm_crc16(data):
    """Vendor recurrence: CCITT table but LEFT SHIFT 1, not standard CCITT CRC."""
    crc = 0xffff
    for byte in data:
        index = ((crc >> 8) ^ byte) & 0xff
        table_entry = binascii.crc_hqx(bytes([index]), 0)
        crc = ((crc << 1) ^ table_entry) & 0xffff
    return crc


class Parser:
    def __init__(self, device_id=1, crc_mode='include_header'):
        if crc_mode not in ('include_header', 'exclude_header'):
            raise ValueError('crc_mode must be include_header or exclude_header')
        if not 0 <= device_id <= 255:
            raise ValueError('device_id must be in [0, 255]')
        self.device_id = device_id
        self.crc_mode = crc_mode
        self.buffer = bytearray()
        self.errors = 0

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while len(self.buffer) >= 4:
            start = self.buffer.find(b'\x55\xaa')
            if start < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1] == 0x55 else b''
                break
            del self.buffer[:start]
            if len(self.buffer) < 4:
                break
            rid = self.buffer[3]
            if rid not in (1, 2, 3, 4):
                del self.buffer[0]
                self.errors += 1
                continue
            size = 23 if rid == 4 else 19
            if len(self.buffer) < size:
                break
            frame = self.buffer[:size]
            begin = 0 if self.crc_mode == 'include_header' else 2
            crc = dm_crc16(frame[begin:-3])
            if frame[-1] != 0x0a or crc != int.from_bytes(frame[-3:-1], 'little'):
                del self.buffer[0]
                self.errors += 1
                continue
            del self.buffer[:size]
            values = struct.unpack('<4f' if rid == 4 else '<3f', frame[4:-3])
            if frame[2] == self.device_id and all(math.isfinite(v) for v in values):
                frames.append((rid, values))
        return frames


def unit_scales(gyro_unit, accel_unit):
    # Explicit units: sensor range in a datasheet does not specify wire units.
    gyro = {'rad_s': 1.0, 'deg_s': math.pi / 180.0}
    accel = {'m_s2': 1.0, 'g': 9.80665}
    if gyro_unit not in gyro or accel_unit not in accel:
        raise ValueError('Set gyro_unit=rad_s|deg_s and accel_unit=m_s2|g after verification')
    return gyro[gyro_unit], accel[accel_unit]
