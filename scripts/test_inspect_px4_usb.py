"""PTY integration checks: decode real MAVLink bytes without transmitting."""
import importlib.util
import os
from pathlib import Path
import pty
import select
import termios
import threading
import time

import pytest
from pymavlink.dialects.v20 import common as mavlink

spec = importlib.util.spec_from_file_location('inspect_px4_usb', Path(__file__).with_name('inspect_px4_usb.py'))
usb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(usb)


def test_passive_capture_decodes_heartbeat_and_never_transmits():
    master, slave = pty.openpty()
    device = os.ttyname(slave)
    saved = termios.tcgetattr(slave)
    os.close(slave)
    sender = mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
    data = sender.heartbeat_encode(mavlink.MAV_TYPE_QUADROTOR, mavlink.MAV_AUTOPILOT_PX4,
                                   0, 0, mavlink.MAV_STATE_STANDBY).pack(sender)
    def feed():
        time.sleep(.1)
        os.write(master, data)
    worker = threading.Thread(target=feed)
    worker.start()
    try:
        result = usb.capture(device, .3)
        worker.join()
        assert result['result'] == 'OBSERVED'
        assert result['autopilots'][0]['armed'] is False
        assert result['message_counts'] == {'HEARTBEAT': 1}
        reopened = os.open(device, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            assert termios.tcgetattr(reopened) == saved
            assert not select.select([master], [], [], 0)[0], 'Tool transmitted data'
        finally:
            os.close(reopened)
    finally:
        worker.join()
        os.close(master)


def test_refuses_existing_serial_reader():
    master, slave = pty.openpty()
    try:
        with pytest.raises(RuntimeError, match='already open'):
            usb.capture(os.ttyname(slave), .1)
    finally:
        os.close(slave)
        os.close(master)


def test_silence_is_inconclusive():
    master, slave = pty.openpty()
    device = os.ttyname(slave)
    os.close(slave)
    try:
        result = usb.capture(device, .05)
        assert result['result'] == 'INCONCLUSIVE'
        assert result['autopilots'] == []
        assert result['transmitted_bytes'] == result['raw_bytes'] == 0
    finally:
        os.close(master)
