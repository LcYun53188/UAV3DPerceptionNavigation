"""Bridge callback tests using actual ROS message types, without a ROS graph."""
from unittest.mock import Mock

from geometry_msgs.msg import TwistStamped
from px4_msgs.msg import VehicleCommandAck, VehicleStatus

from px4_comm_bridge.control_bridge import Px4ControlBridge
from px4_comm_bridge.px4_state_machine import PX4State
from test_state_machine_verification import MockNode


def make_bridge(auto_arm=False):
    node = MockNode(auto_arm)
    node.get_clock = lambda: Mock(now=lambda: node.time)
    bridge = Px4ControlBridge(node, True, object, object, object)
    bridge.state_machine._clock = lambda: node.time
    bridge.state_machine.state_enter_time = node.time
    bridge.send_vehicle_command = Mock()
    bridge.state_machine.send_vehicle_command = bridge.send_vehicle_command
    bridge.publish_offboard_mode = Mock()
    bridge.publish_halt_setpoint = Mock()
    bridge.publish_setpoint = Mock()
    # Callback receipt time still uses ROS time; state deadlines use monotonic time.
    node.get_clock = lambda: Mock(now=lambda: node.time)
    return node, bridge


def status(bridge, armed=False, offboard=False):
    msg = VehicleStatus()
    msg.arming_state = (VehicleStatus.ARMING_STATE_ARMED if armed
                        else VehicleStatus.ARMING_STATE_DISARMED)
    msg.nav_state = (VehicleStatus.NAVIGATION_STATE_OFFBOARD if offboard
                     else VehicleStatus.NAVIGATION_STATE_MANUAL)
    bridge.vehicle_status_cb(msg)


def test_callback_cannot_bypass_default_authorization():
    _, bridge = make_bridge()
    status(bridge)
    bridge.cmd_cb(TwistStamped())
    bridge.publish_control()
    bridge.send_vehicle_command.assert_not_called()
    bridge.publish_offboard_mode.assert_not_called()


def test_bridge_streams_before_mode_request_and_stops_after_takeover():
    node, bridge = make_bridge(True)
    status(bridge)
    cmd = TwistStamped()
    bridge.cmd_cb(cmd)
    bridge.publish_control()
    bridge.publish_offboard_mode.assert_called_once()
    bridge.publish_halt_setpoint.assert_called_once()
    bridge.publish_setpoint.assert_not_called()
    for _ in range(16):
        node.time += 0.1
        status(bridge)
        bridge.cmd_cb(cmd)
        bridge.publish_control()
    bridge.send_vehicle_command.assert_called_with(400, param1=1.0)
    status(bridge, armed=True)
    bridge.publish_control()
    bridge.send_vehicle_command.assert_called_with(176, param1=1.0, param2=6.0)
    status(bridge, armed=True, offboard=True)
    bridge.publish_control()
    bridge.publish_setpoint.assert_called_once_with(cmd)
    status(bridge, armed=True, offboard=False)
    bridge.publish_offboard_mode.reset_mock()
    bridge.send_vehicle_command.reset_mock()
    bridge.cmd_cb(cmd)
    bridge.publish_control()
    assert bridge.state_machine.current_state == PX4State.MANUAL
    bridge.publish_offboard_mode.assert_not_called()
    bridge.send_vehicle_command.assert_not_called()


def test_ack_callback_filters_recipient():
    _, bridge = make_bridge(True)
    bridge.state_machine.on_command_ack = Mock()
    ack = VehicleCommandAck()
    ack.command = 400
    ack.result = VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED
    ack.target_system = 2
    ack.target_component = 1
    bridge.command_ack_cb(ack)
    bridge.state_machine.on_command_ack.assert_not_called()
    ack.target_system = 1
    bridge.command_ack_cb(ack)
    bridge.state_machine.on_command_ack.assert_called_once_with(400, 2)
