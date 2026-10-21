"""Real BT/ROS proof of early SD fallback, using a fake aircraft, no FMU output."""
import json
import os
import subprocess
import threading
import time

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future
from uav_nav_interfaces.action import ExecuteMission
from uav_nav_interfaces.msg import TaskStatus
from test_runner_decisions import DecisionBackend, wait_for
from test_runner_integration import BINARY


class SafetyBackend(DecisionBackend):
    def __init__(self,context,phase,case):
        super().__init__(context,'safety-end')
        self.held_phase=phase
        self.safety_case=case
        self.end_requested=None
        self.end_calls=0
        self.advance_wait=None

    async def advance(self,request,response):
        response=super().advance(request,response)
        if response.accepted and self.safety_case=='late-step-reject':
            self.advance_wait=Future()
            await self.advance_wait
            response.accepted=False
            response.reason='RETIRED_BY_SAFETY_PLAN'
        return response

    def request_return(self,request,response):
        assert request.finish_mission
        self.end_calls+=1
        self.end_requested=time.monotonic()
        self.plan_revision+=1
        response.accepted=True
        return response

    def tick(self):
        if self.end_requested is not None and self.active_goal and not self.pending.done():
            age=time.monotonic()-self.end_requested
            if self.safety_case=='late-step-reject' and age<.35:
                if age>.1 and self.advance_wait is not None and not self.advance_wait.done():
                    self.advance_wait.set_result(True)
                # Delayed old feedback retains its old revision; the safety
                # revision is first published with its own LAND identity.
                status=TaskStatus(mission_uuid=self.active_goal.goal_id,
                    coordinator_instance='test-instance',phase=self.held_phase,plan_revision=0)
                self.active_goal.publish_feedback(ExecuteMission.Feedback(status=status,
                    tree_node=f'PX4_STEP[{self.step_index}]'))
                return
            phase='LAND_REQUEST' if age<.25 or self.safety_case=='unconfirmed' else 'LANDING'
            status=TaskStatus(mission_uuid=self.active_goal.goal_id,coordinator_instance='test-instance',
                phase=phase,plan_revision=self.plan_revision)
            self.active_goal.publish_feedback(ExecuteMission.Feedback(status=status,
                tree_node=f'PX4_END_LAND[{self.step_index}]'))
            if age>.6:self.pending.set_result(True)
            return
        if self.advances:
            self.step_phase=self.held_phase
            self.step_started=time.monotonic()+30
        super().tick()

    async def execute(self,handle):
        await self.pending
        if self.cancel_at:
            handle.canceled()
            return ExecuteMission.Result(result_code='CANCELED',cleanup_confirmed=True)
        handle.succeed()
        return ExecuteMission.Result(result_code='SUCCEEDED',cleanup_confirmed=True,
            mock=self.safety_case=='mock',reason='LANDED_AND_DISARMED' if self.safety_case=='wrong-reason'
            else 'MISSION_ENDED_LOCAL_LANDED_AND_DISARMED')


CASES=[(p,'success') for p in ('PRESTREAM','OFFBOARD_REQUEST','ARM_REQUEST','TAKEOFF','OBSERVE',
    'PAUSING','HOLD_HANDOFF')]+[('HOVER','pending-pause'),('OBSERVE','unconfirmed'),
    ('OBSERVE','wrong-reason'),('OBSERVE','mock'),('TAKEOFF','late-step-reject')]


@pytest.mark.parametrize('phase,case',CASES)
def test_sd_early_safety_landing_requires_bound_native_feedback(tmp_path,phase,case):
    domain=124+CASES.index((phase,case))
    context=Context()
    rclpy.init(context=context,domain_id=domain)
    node=SafetyBackend(context,phase,case)
    executor=SingleThreadedExecutor(context=context)
    executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True)
    thread.start()
    params=tmp_path/'parameters.json'
    params.write_text(json.dumps(dict(step_controlled=True,runner_progress_required=True,
        coordinator_instance='test-instance',steps=[dict(type=t) for t in
        ('TAKEOFF','OBSERVE','NAVIGATE','RETURN','LAND')])))
    process=None
    try:
        with (tmp_path/'runner.log').open('w') as log:
            process=subprocess.Popen([str(BINARY),'--ros-args','-p',f'parameters_file:={params}',
                '-p','coordinator_instance:=test-instance'],env=dict(os.environ,ROS_DOMAIN_ID=str(domain)),
                stdout=log,stderr=subprocess.STDOUT)
            wait_for(lambda:node.status is not None and node.status.phase==phase)
            if case=='pending-pause':assert node.command('PAUSE').accepted
            assert node.command('END_MISSION').accepted
            process.wait(timeout=10)
        log=(tmp_path/'runner.log').read_text()
        expected=0 if case in ('success','pending-pause','late-step-reject') else 1
        assert process.returncode==expected,log
        assert node.goals==1 and node.end_calls==1
        if expected==0:assert node.cancel_at is None
        assert 'BT_RETURN_COMPLETE' not in log
        assert ('BT_END_LOCAL_LAND_COMPLETE cleanup=1' in log)==(expected==0)
    finally:
        if process is not None and process.poll() is None:
            process.kill();process.wait(timeout=3)
        executor.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        node.destroy_node();context.shutdown()
