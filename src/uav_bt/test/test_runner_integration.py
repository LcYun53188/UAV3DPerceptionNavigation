"""Real ROS Action transport/BT lifecycle tests; fake flight, no PX4 publishers."""
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import pytest
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.context import Context
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from uav_nav_interfaces.action import ExecuteMission
from uav_nav_interfaces.msg import MissionProgress, TaskStatus
from uav_nav_interfaces.srv import AdvanceFlightStep

ROOT = Path(__file__).resolve().parents[3]
BINARY = ROOT/'.deps/mission-install/uav_bt/lib/uav_bt/mission_runner'


class Backend(Node):
    def __init__(self, context, case):
        super().__init__('bt_contract_backend', context=context)
        self.case = case
        self.goals = 0
        self.active_goal = None
        self.pending = None
        self.accepted = threading.Event()
        self.cancel_at = None
        self.progress = []
        self.step_mode = case.startswith('step-')
        self.step_index, self.step_phase = -1, 'PRESTREAM'
        self.advances = []
        self.create_service(AdvanceFlightStep, '/uav/px4/advance_step', self.advance)
        self.create_subscription(MissionProgress, '/uav/px4/mission_progress', self.progress.append, 10)
        self.server = ActionServer(self, ExecuteMission, '/uav/px4/execute_mission',
                                   self.execute, goal_callback=self.goal, cancel_callback=self.cancel,
                                   handle_accepted_callback=self.accept, callback_group=ReentrantCallbackGroup())
        self.create_timer(.02, self.tick)

    def goal(self, _):
        self.goals += 1
        if self.case == 'halt-before-accept':
            time.sleep(.8)
        return GoalResponse.REJECT if self.case == 'reject' else GoalResponse.ACCEPT

    def accept(self, handle):
        self.active_goal = handle
        self.start = time.monotonic()
        self.pending = Future()
        self.accepted.set()
        handle.execute()

    def cancel(self, _):
        if self.case == 'landing':
            return CancelResponse.REJECT
        self.cancel_at = time.monotonic()
        return CancelResponse.ACCEPT

    def advance(self, request, response):
        if (request.mission_uuid != self.active_goal.goal_id or
                request.coordinator_instance != 'test-instance' or
                request.step_index != self.step_index+1 or self.case == 'step-reject'):
            response.reason = 'TEST_REJECTED'; return response
        self.step_index = request.step_index
        self.advances.append(self.step_index)
        self.step_phase = 'LANDING' if request.step_type == 'LAND' else request.step_type
        self.step_started = time.monotonic()
        response.accepted = True
        return response

    async def execute(self, handle):
        await self.pending
        result = ExecuteMission.Result(result_code='SUCCEEDED', reason='TEST',
                                       cleanup_confirmed=True, mock=False)
        if self.cancel_at:
            result.result_code = 'CANCELED'
            handle.canceled()
        elif self.case == 'abort':
            result.result_code = 'ABORTED'
            result.cleanup_confirmed = False
            handle.abort()
        else:
            if self.case == 'mock':
                result.mock = True
            if self.case == 'unconfirmed':
                result.cleanup_confirmed = False
            if self.case == 'contradictory':
                result.result_code = 'ABORTED'
            handle.succeed()
        return result

    def tick(self):
        if not self.active_goal or self.pending.done():
            return
        if self.step_mode and self.advances and time.monotonic()-self.step_started >= .3:
            self.step_phase = 'AWAIT_STEP' if self.step_index < 4 else 'LANDING'
        status = TaskStatus(mission_uuid=self.active_goal.goal_id, coordinator_instance='test-instance',
                            phase=self.step_phase if self.step_mode else 'LANDING' if self.case == 'landing' else 'NAVIGATE')
        self.active_goal.publish_feedback(ExecuteMission.Feedback(status=status,
            tree_node=f'PX4_STEP[{self.step_index}]' if self.step_mode else ''))
        now = time.monotonic()
        if self.cancel_at:
            if now-self.cancel_at >= .7:
                self.pending.set_result(True)
        elif self.step_mode:
            if self.case == 'step-early-success' or (self.step_index == 4 and now-self.step_started >= .5):
                self.pending.set_result(True)
        elif (self.case not in ('halt', 'halt-before-accept') and now-self.start >= 1.2
              and (self.case != 'success' or len(self.progress) >= 5)):
            self.pending.set_result(True)


@pytest.mark.parametrize('case,expected', [
    ('success',0), ('abort',1), ('reject',1), ('mock',1), ('unconfirmed',1),
    ('contradictory',1), ('halt',130), ('halt-before-accept',130), ('landing',0),
    ('step-success',0), ('step-reject',1), ('step-halt',130), ('step-early-success',1)])
def test_ros_bt_dispatch_result_and_halt_cleanup(tmp_path, case, expected):
    assert BINARY.exists(), 'Build scripts/build_px4_flight.sh before integration tests'
    context = Context()
    domain=80+['success','abort','reject','mock','unconfirmed','contradictory','halt','halt-before-accept','landing',
               'step-success','step-reject','step-halt','step-early-success'].index(case)
    rclpy.init(context=context, domain_id=domain)
    node = Backend(context, case)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    params = tmp_path/'parameters.json'
    params.write_text(json.dumps(dict(runner_progress_required=True, coordinator_instance='test-instance',
        step_controlled=node.step_mode, steps=[{'type': t} for t in ['TAKEOFF','NAVIGATE','HOVER','RETURN','LAND']])))
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain))
    log_path = tmp_path/'runner.log'
    try:
        with log_path.open('w') as log:
            process = subprocess.Popen([str(BINARY), '--ros-args', '-p', f'parameters_file:={params}',
                                        '-p', 'coordinator_instance:=test-instance'],
                                       env=env, stdout=log, stderr=subprocess.STDOUT)
            try:
                if case in ('halt','landing','step-halt'):
                    assert node.accepted.wait(8)
                    time.sleep(.25)  # Let LANDING feedback reach the runner.
                    process.send_signal(signal.SIGTERM)
                elif case == 'halt-before-accept':
                    until = time.monotonic()+8
                    while 'Dispatched root mission once' not in log_path.read_text() and time.monotonic()<until:
                        time.sleep(.02)
                    process.send_signal(signal.SIGTERM)
                process.wait(timeout=12)
                text = log_path.read_text()
                assert process.returncode == expected, text
                assert node.goals == 1, text
                assert text.count('Dispatched root mission once') == 1, text
                assert text.count('BT_RESULT') == 1, text
                if case in ('halt','halt-before-accept','step-halt'):
                    assert node.cancel_at is not None
                    assert time.monotonic()-node.cancel_at >= .7
                elif case == 'landing':
                    assert node.cancel_at is None
                elif case == 'step-success':
                    assert node.advances == list(range(5))
                    assert text.count('BT_STEP_ACCEPTED') == 5 and text.count('BT_STEP_COMPLETE') == 5
                before = len(node.progress)
                time.sleep(.25)
                assert len(node.progress) == before
                assert all(p.coordinator_instance=='test-instance' for p in node.progress)
                assert all(a.tick_sequence < b.tick_sequence for a,b in zip(node.progress,node.progress[1:]))
                if case == 'success':
                    assert len(node.progress) >= 3
                    assert all(bytes(p.mission_uuid.uuid)==bytes(node.active_goal.goal_id.uuid) for p in node.progress)
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait(timeout=3)
    finally:
        executor.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        node.destroy_node()
        context.shutdown()
