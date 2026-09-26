"""Deterministic state/feedback tests; no ROS graph or PX4 required."""
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from px4_comm_bridge.px4_state_machine import PX4State, PX4StateMachine


class MockNode:
    def __init__(self, auto_arm=False):
        self.parameters = dict(auto_arm=auto_arm, cmd_timeout_sec=0.5, emergency_action='land')
        self.time = 0.0
        self.logger = Mock()

    def declare_parameter(self, name, default):
        self.parameters.setdefault(name, default)

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])

    def get_logger(self):
        return self.logger


@pytest.fixture
def rig():
    node = MockNode(auto_arm=True)
    sm = PX4StateMachine(node, True, object, monotonic=lambda: node.time)
    sm.send_vehicle_command = Mock()
    return node, sm


def tick(rig, dt=0.1, armed=False, offboard=False, command=True, status=True):
    node, sm = rig
    node.time += dt
    if command:
        sm.on_navigation_command_received()
    if status:
        sm.on_px4_status_update(armed, offboard, 14 if offboard else 0)
    sm.update()


def enter_arm(rig):
    tick(rig)
    assert rig[1].current_state == PX4State.PRESTREAM
    for _ in range(16):
        tick(rig)
    assert rig[1].current_state == PX4State.ARM


def enter_flying(rig):
    enter_arm(rig)
    tick(rig, armed=True)
    assert rig[1].current_state == PX4State.OFFBOARD
    tick(rig, armed=True, offboard=True)
    assert rig[1].current_state == PX4State.FLYING


def test_default_and_legacy_parameter_cannot_arm(rig):
    node, sm = rig
    node.parameters['auto_arm'] = False
    node.parameters['sm_auto_arm'] = True
    for _ in range(30):
        tick(rig)
    assert MockNode().parameters['auto_arm'] is False
    assert sm.current_state == PX4State.IDLE
    sm.send_vehicle_command.assert_not_called()


def test_no_status_no_start(rig):
    tick(rig, status=False)
    assert rig[1].current_state == PX4State.IDLE


def test_prestream_and_feedback_required(rig):
    _, sm = rig
    tick(rig)
    sm.send_vehicle_command.assert_not_called()
    for _ in range(16):
        tick(rig)
    sm.send_vehicle_command.assert_called_with(400, param1=1.0)
    sm.on_command_ack(400, 0)
    tick(rig)
    assert sm.current_state == PX4State.ARM
    tick(rig, armed=True)
    sm.send_vehicle_command.assert_called_with(176, param1=1.0, param2=6.0)
    sm.on_command_ack(176, 0)
    tick(rig, armed=True)
    assert sm.current_state == PX4State.OFFBOARD
    tick(rig, armed=True, offboard=True)
    assert sm.current_state == PX4State.FLYING


@pytest.mark.parametrize('armed', [False, True])
def test_retries_do_not_extend_transition_deadline(rig, armed):
    enter_arm(rig)
    if armed:
        tick(rig, armed=True)
    for _ in range(52):
        tick(rig, armed=armed)
    assert rig[1].current_state == PX4State.FAULT
    count = rig[1].send_vehicle_command.call_count
    tick(rig, armed=armed)
    assert rig[1].send_vehicle_command.call_count == count


@pytest.mark.parametrize('result', [2, 3, 4, 6])
def test_rejection_latches_fault(rig, result):
    enter_arm(rig)
    sm = rig[1]
    sm.on_command_ack(176, result)  # Unrelated command.
    assert sm.current_state == PX4State.ARM
    sm.on_command_ack(400, result)
    tick(rig)
    assert sm.current_state == PX4State.FAULT


def test_temporary_rejection_waits_for_status(rig):
    enter_arm(rig)
    rig[1].on_command_ack(400, 1)
    tick(rig)
    assert rig[1].current_state == PX4State.ARM


@pytest.mark.parametrize('stage', ['arm', 'offboard', 'flying'])
def test_safety_interrupts_all_active_stages(rig, stage):
    enter_arm(rig)
    if stage != 'arm':
        tick(rig, armed=True)
    if stage == 'flying':
        tick(rig, armed=True, offboard=True)
    rig[1].on_safety_status(2)
    tick(rig, armed=True, offboard=stage == 'flying')
    assert rig[1].current_state == PX4State.EMERGENCY
    rig[1].send_vehicle_command.assert_called_with(21)


def test_command_timeout_uses_shared_parameter(rig):
    enter_flying(rig)
    tick(rig, dt=0.6, armed=True, offboard=True, command=False)
    assert rig[1].current_state == PX4State.EMERGENCY


def test_status_loss_faults_without_commands(rig):
    enter_flying(rig)
    rig[1].send_vehicle_command.reset_mock()
    tick(rig, dt=1.1, status=False)
    assert rig[1].current_state == PX4State.FAULT
    rig[1].send_vehicle_command.assert_not_called()


def test_emergency_never_assumes_landed_from_time(rig):
    enter_flying(rig)
    sm = rig[1]
    sm.on_emergency_signal(True)
    for _ in range(120):
        tick(rig, armed=True, offboard=True)
    assert sm.current_state == PX4State.EMERGENCY
    tick(rig, armed=False)
    assert sm.current_state == PX4State.LANDED
    for _ in range(30):
        tick(rig)
    assert sm.current_state == PX4State.LANDED


def test_manual_takeover_latches_even_between_timer_ticks(rig):
    enter_flying(rig)
    sm = rig[1]
    sm.send_vehicle_command.reset_mock()
    sm.on_px4_status_update(True, False, 3)
    sm.on_px4_status_update(True, True, 14)
    for _ in range(20):
        tick(rig, armed=True, offboard=True)
    assert sm.current_state == PX4State.MANUAL
    sm.send_vehicle_command.assert_not_called()


def test_revoking_authorization_aborts(rig):
    enter_flying(rig)
    rig[0].parameters['auto_arm'] = False
    tick(rig, armed=True, offboard=True)
    assert rig[1].current_state == PX4State.EMERGENCY
