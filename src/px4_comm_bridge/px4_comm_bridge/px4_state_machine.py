"""Feedback-driven legacy velocity bridge; not a trajectory executor."""

from enum import Enum, auto
import time


class PX4State(Enum):
    IDLE = auto()
    PRESTREAM = auto()
    ARM = auto()
    OFFBOARD = auto()
    FLYING = auto()
    EMERGENCY = auto()
    LANDED = auto()
    MANUAL = auto()
    FAULT = auto()


class PX4StateMachine:
    CMD_ARM = 400
    CMD_LAND = 21
    CMD_RTL = 20
    CMD_SET_MODE = 176

    def __init__(self, node, px4_available, vehicle_command_type, monotonic=time.monotonic):
        self.node = node
        self.px4_available = px4_available
        self.vehicle_command_type = vehicle_command_type
        self._clock = monotonic
        self.current_state = PX4State.IDLE
        self.previous_state = None
        self.state_enter_time = self._clock()
        self.last_action_time = None
        self.last_cmd_time = None
        self.last_status_time = None
        self.received_navigation_cmd = False
        self.emergency_signal_active = False
        self.safety_level = 0
        self.px4_armed = False
        self.px4_offboard_active = False
        self.px4_nav_state = 0
        self.state_entry_count = {state: 0 for state in PX4State}
        # Shared parameters are declared by the node. No sm_auto_arm override.
        for name, default in (
            ('sm_transition_timeout_sec', 5.0),
            ('sm_status_timeout_sec', 1.0),
            ('sm_prestream_sec', 1.5),
        ):
            self.node.declare_parameter(name, default)

    def send_vehicle_command(self, command, **kwargs):
        raise NotImplementedError('Control bridge must install command callback')

    def on_navigation_command_received(self):
        self.received_navigation_cmd = True
        self.last_cmd_time = self._clock()

    def on_emergency_signal(self, active):
        self.emergency_signal_active = bool(active)

    def on_safety_status(self, level):
        self.safety_level = int(level)

    def on_px4_status_update(self, armed, offboard_active, nav_state):
        self.last_status_time = self._clock()
        self.px4_armed = bool(armed)
        self.px4_offboard_active = bool(offboard_active)
        self.px4_nav_state = int(nav_state)
        # Latch loss of control even if another status arrives before the timer.
        if self.current_state == PX4State.FLYING and not (
            self.px4_armed and self.px4_offboard_active
        ):
            self._transition_to(PX4State.MANUAL)

    def on_command_ack(self, command, result):
        expected = {PX4State.ARM: self.CMD_ARM, PX4State.OFFBOARD: self.CMD_SET_MODE}
        if self.last_action_time is None or command != expected.get(self.current_state):
            return
        # ACCEPTED/IN_PROGRESS are not VehicleStatus confirmation. Temporary
        # rejection may retry within the original deadline; other failures latch.
        if result not in (0, 1, 5):
            self._transition_to(PX4State.FAULT)

    def _transition_to(self, state):
        if state == self.current_state:
            return
        self.previous_state = self.current_state
        self.current_state = state
        self.state_enter_time = self._clock()
        self.last_action_time = None
        self.state_entry_count[state] += 1
        if state in (PX4State.MANUAL, PX4State.FAULT, PX4State.LANDED):
            self.received_navigation_cmd = False
        self.node.get_logger().info(
            f'[SM] {self.previous_state.name} -> {state.name}')

    def _fresh(self, stamp, timeout):
        return stamp is not None and 0 <= self._clock() - stamp <= timeout

    def _parameter(self, name):
        return self.node.get_parameter(name).value

    def _check_state_transitions(self):
        state = self.current_state
        if state in (PX4State.MANUAL, PX4State.FAULT, PX4State.LANDED):
            return  # Restart explicitly after operator review; nav cannot reset.
        status_fresh = self._fresh(self.last_status_time, self._parameter('sm_status_timeout_sec'))
        command_fresh = self._fresh(self.last_cmd_time, self._parameter('cmd_timeout_sec'))
        unsafe = self.emergency_signal_active or self.safety_level >= 2
        if state == PX4State.IDLE:
            if (self.px4_available and self._parameter('auto_arm') and
                    self.received_navigation_cmd and command_fresh and
                    status_fresh and not unsafe):
                self._transition_to(PX4State.PRESTREAM)
            return
        if not status_fresh:
            self._transition_to(PX4State.FAULT)
            return
        if state == PX4State.EMERGENCY:
            # Disarmed feedback ends control; never infer landing from elapsed time.
            if not self.px4_armed:
                self._transition_to(PX4State.LANDED)
            return
        if unsafe or not command_fresh or not self._parameter('auto_arm'):
            self._transition_to(PX4State.EMERGENCY if self.px4_armed else PX4State.FAULT)
            return
        elapsed = self._clock() - self.state_enter_time
        if state == PX4State.PRESTREAM:
            if elapsed >= self._parameter('sm_prestream_sec'):
                self._transition_to(PX4State.OFFBOARD if self.px4_armed else PX4State.ARM)
        elif state == PX4State.ARM:
            if self.px4_armed:
                self._transition_to(PX4State.OFFBOARD)
            elif elapsed >= self._parameter('sm_transition_timeout_sec'):
                self._transition_to(PX4State.FAULT)
        elif state == PX4State.OFFBOARD:
            if not self.px4_armed:
                self._transition_to(PX4State.MANUAL)
            elif self.px4_offboard_active:
                self._transition_to(PX4State.FLYING)
            elif elapsed >= self._parameter('sm_transition_timeout_sec'):
                self._transition_to(PX4State.FAULT)

    def _execute_state_actions(self):
        state = self.current_state
        if self.last_action_time is not None and self._clock() - self.last_action_time < 0.5:
            return
        if state == PX4State.ARM:
            self.send_vehicle_command(self.CMD_ARM, param1=1.0)
        elif state == PX4State.OFFBOARD:
            self.send_vehicle_command(self.CMD_SET_MODE, param1=1.0, param2=6.0)
        elif state == PX4State.EMERGENCY:
            action = str(self._parameter('emergency_action')).lower()
            if action == 'disarm':
                self.send_vehicle_command(self.CMD_ARM, param1=0.0)
            else:
                self.send_vehicle_command(self.CMD_RTL if action == 'rtl' else self.CMD_LAND)
        else:
            return
        self.last_action_time = self._clock()

    def update(self):
        self._check_state_transitions()
        self._execute_state_actions()

    def get_state_info(self):
        return {
            'current_state': self.current_state.name,
            'safety_level': self.safety_level,
            'emergency_active': self.emergency_signal_active,
            'px4_armed': self.px4_armed,
            'px4_offboard_active': self.px4_offboard_active,
            'received_nav_cmd': self.received_navigation_cmd,
            'time_in_state_sec': self._clock() - self.state_enter_time,
        }
