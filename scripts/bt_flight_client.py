"""Owned BT subprocess and read-only root-result observer for SITL acceptance."""
import json
import os
import signal
import subprocess
import time

from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalInfo
from uav_nav_interfaces.action import ExecuteMission


class BtFlightClient:
    def __init__(self, node, request, out, root):
        self.node = node
        self.parameters_file = out/'bt-parameters.private.json'
        params = json.loads(request.parameters_json)
        params.update(runner_progress_required=True, coordinator_instance=node.instance, step_controlled=True)
        # Authorization is never placed in a command line or archived evidence.
        fd = os.open(self.parameters_file, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, 'w') as stream:
            json.dump(params, stream)
        self.log = (out/'bt-runner.log').open('w')
        binary = root/'.deps/mission-install/uav_bt/lib/uav_bt/mission_runner'
        self.process = subprocess.Popen([
            str(binary), '--ros-args', '-p', f'parameters_file:={self.parameters_file}',
            '-p', f'coordinator_instance:={node.instance}', '-p', f'timeout_s:={request.timeout_s}',
            '-p', f'tree_output_file:={out/"bt-flight-tree.xml"}'],
            stdout=self.log, stderr=subprocess.STDOUT)
        self.result_client = node.create_client(
            ExecuteMission.Impl.GetResultService, '/uav/px4/execute_mission/_action/get_result')
        self.cancel_client = node.create_client(CancelGoal, '/uav/px4/execute_mission/_action/cancel_goal')
        self.accepted = False
        self.goal_id = None

    def wait_accepted(self, timeout=10):
        until = time.monotonic()+timeout
        while time.monotonic() < until and self.process.poll() is None:
            if self.node.active_goal:
                self.goal_id = self.node.active_goal.goal_id
                self.accepted = True
                return self
            time.sleep(.02)
        raise RuntimeError('BT runner failed to submit a root mission; inspect bt-runner.log')

    def get_result_async(self):
        if not self.result_client.wait_for_service(timeout_sec=5):
            raise RuntimeError('Root result service unavailable')
        return self.result_client.call_async(ExecuteMission.Impl.GetResultService.Request(goal_id=self.goal_id))

    def cancel_goal_async(self):
        # During motion, exercise Runner's halt/cleanup path, not a second Action client.
        if self.node.phase not in ('LAND_REQUEST', 'LANDING'):
            self.process.send_signal(signal.SIGTERM)
            from rclpy.task import Future
            future = Future()
            def observe():
                if self.node.canceling:
                    response = CancelGoal.Response()
                    response.goals_canceling = [GoalInfo(goal_id=self.goal_id)]
                    future.set_result(response)
                    self.node.destroy_timer(timer)
            timer = self.node.create_timer(.02, observe)
            return future
        return self.cancel_client.call_async(CancelGoal.Request(
            goal_info=GoalInfo(goal_id=self.goal_id)))

    def close(self):
        if self.process.poll() is None:
            self.process.kill()
        self.process.wait(timeout=5)
        self.log.close()
        self.parameters_file.unlink(missing_ok=True)
        self.node.destroy_client(self.result_client)
        self.node.destroy_client(self.cancel_client)
