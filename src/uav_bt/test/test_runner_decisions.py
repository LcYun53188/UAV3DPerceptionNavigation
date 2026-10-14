"""Real ROS/BT decision transport; fake aircraft, no FMU/control publishers."""
import json
import os
import subprocess
import threading
import time
import uuid

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from uav_nav_interfaces.msg import TaskStatus
from uav_nav_interfaces.action import ExecuteMission
from uav_nav_interfaces.srv import PauseMission, ResumeMission, SetMissionIntent, RequestReturn, ResumeOriginalMission
from test_runner_integration import Backend, BINARY


def wait_for(predicate, timeout=8):
    until = time.monotonic()+timeout
    while not predicate() and time.monotonic() < until:
        time.sleep(.01)
    assert predicate(), 'decision/feedback timeout'


class DecisionBackend(Backend):
    def __init__(self, context, case):
        super().__init__(context, 'step-decision')
        self.decision_case = case
        self.pause_calls = self.resume_calls = 0
        self.pausing_at = None
        self.resumed = False
        self.return_override = False
        self.plan_revision = 0
        self.can_restore = False
        self.restore_calls = 0
        self.advance_ids = []
        self.status = None
        self.create_subscription(TaskStatus, '/uav/bt/task_status', self.receive, 10)
        self.create_service(PauseMission, '/uav/px4/pause', self.pause)
        self.create_service(ResumeMission, '/uav/px4/resume', self.resume)
        self.create_service(RequestReturn, '/uav/px4/request_return', self.request_return)
        self.create_service(ResumeOriginalMission, '/uav/px4/resume_original_mission', self.restore)
        self.intent = self.create_client(SetMissionIntent, '/uav/bt/set_intent')

    def receive(self, message):
        self.status = message

    def advance(self, request, response):
        result = super().advance(request, response)
        if result.accepted:
            self.advance_ids.append(bytes(request.request_id.uuid))
        if result.accepted and self.step_index == 1 and self.advances.count(1) == 1:
            self.step_started = time.monotonic()+2.
        if result.accepted and self.decision_case == 'restore-twice' and self.advances.count(1) == 2:
            self.step_started = time.monotonic()+1.
        return result

    def pause(self, _, response):
        self.pause_calls += 1
        response.accepted = self.decision_case != 'reject'
        if response.accepted:
            self.pausing_at = time.monotonic()
            self.step_phase = 'PAUSING'
        return response

    def resume(self, _, response):
        self.resume_calls += 1
        response.accepted = True
        self.resumed = True
        self.step_phase = 'NAVIGATE'
        self.step_started = time.monotonic()+.5
        if self.return_override:
            self.return_started = time.monotonic()-.4
        return response

    def request_return(self, request, response):
        self.checkpoint_index = self.step_index
        self.can_restore = not request.finish_mission
        self.finished = request.finish_mission
        self.plan_revision += 1
        self.return_override = True
        self.return_started = time.monotonic()
        response.accepted = True
        return response

    def restore(self, _, response):
        self.restore_calls += 1
        response.accepted = self.decision_case != 'restore-reject'
        if response.accepted and self.decision_case != 'restore-unconfirmed':
            self.return_override = self.can_restore = False
            self.plan_revision += 1
            self.step_index = self.checkpoint_index-1
            self.step_phase = 'AWAIT_STEP'
            self.pausing_at = None
            self.resumed = False
        return response

    async def execute(self, handle):
        result = await super().execute(handle)
        if self.return_override and result.result_code == 'SUCCEEDED':
            result.reason = 'MISSION_ENDED_LANDED_AND_DISARMED' if self.finished else 'RETURN_LANDED_AND_DISARMED'
        return result

    def tick(self):
        if self.return_override and self.active_goal and not self.pending.done():
            age = time.monotonic()-self.return_started
            if self.pausing_at and not self.resumed:
                phase = 'PAUSED' if time.monotonic()-self.pausing_at > .35 else 'PAUSING'
                index = 3
            else:
                phase, index = ('RETURN_BRAKE',2) if age < .35 else ('RETURN',3) if age < 1.6 else ('LANDING',4)
            status = TaskStatus(mission_uuid=self.active_goal.goal_id,coordinator_instance='test-instance',phase=phase,
                plan_revision=self.plan_revision,can_resume_mission=self.can_restore,
                resume_step_index=self.checkpoint_index if self.can_restore else -1)
            self.active_goal.publish_feedback(ExecuteMission.Feedback(status=status,tree_node=f'PX4_RETURN[{index}]'))
            if (self.decision_case == 'return-early' or (phase == 'LANDING' and age > 2.1) or
                    (self.cancel_at and time.monotonic()-self.cancel_at > .7)):
                self.pending.set_result(True)
            return
        if self.decision_case == 'restore-early' and self.restore_calls:
            self.pending.set_result(True)
            return
        if (self.decision_case == 'restore-old-feedback' and self.restore_calls and
                self.active_goal and not self.pending.done()):
            stale=TaskStatus(mission_uuid=self.active_goal.goal_id,coordinator_instance='test-instance',
                phase='RETURN',plan_revision=1,can_resume_mission=True,resume_step_index=1)
            self.active_goal.publish_feedback(ExecuteMission.Feedback(status=stale,tree_node='PX4_RETURN[3]'))
        if self.pausing_at and not self.resumed:
            self.step_started = time.monotonic()+10
            if self.decision_case != 'unconfirmed' and time.monotonic()-self.pausing_at > .35:
                self.step_phase = 'PAUSED'
        if self.decision_case == 'stale' and self.step_index == 1:
            # Keep the backend alive but stop publishing root feedback.
            if self.cancel_at and time.monotonic()-self.cancel_at > .7 and not self.pending.done():
                self.pending.set_result(True)
            return
        super().tick()

    def command(self, verb, *, root=None, instance='test-instance', request_id=None):
        wait_for(self.intent.service_is_ready)
        goal = self.status.mission_uuid if root is None else root
        request = SetMissionIntent.Request(mission_uuid=goal, coordinator_instance=instance, command=verb)
        request.request_id.uuid = list(request_id or uuid.uuid4().bytes)
        future = self.intent.call_async(request)
        wait_for(future.done)
        return future.result()


CASES = ['pause-resume', 'cancel-pausing', 'unconfirmed', 'reject', 'stale', 'start', 'identity',
         'return', 'return-pause', 'return-early', 'restore', 'restore-reject', 'restore-unconfirmed', 'restore-early', 'finish',
         'restore-twice','restore-old-feedback']


@pytest.mark.parametrize('case', CASES)
def test_priority_decisions_keep_root_and_wait_for_physical_feedback(tmp_path, case):
    context = Context()
    domain = 103+CASES.index(case)
    rclpy.init(context=context, domain_id=domain)
    node = DecisionBackend(context, case)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    parameters = tmp_path/'params.json'
    parameters.write_text(json.dumps(dict(step_controlled=True, runner_progress_required=True,
        coordinator_instance='test-instance', operator_start_required=case == 'start',
        steps=[dict(type=t) for t in ('TAKEOFF', 'NAVIGATE', 'HOVER', 'RETURN', 'LAND')])))
    process = None
    try:
        with (tmp_path/'runner.log').open('w') as log:
            process = subprocess.Popen([str(BINARY), '--ros-args', '-p', f'parameters_file:={parameters}',
                '-p', 'coordinator_instance:=test-instance'], env=dict(os.environ, ROS_DOMAIN_ID=str(domain)),
                stdout=log, stderr=subprocess.STDOUT)
            wait_for(lambda: node.status is not None)
            if case == 'start':
                assert node.status.phase == 'WAIT_START'
                time.sleep(.3)
                assert node.goals == 0 and not node.progress
                assert not node.command('START', instance='old-instance').accepted
                request_id = uuid.uuid4().bytes
                assert node.command('START', request_id=request_id).accepted
                # The duplicate is recorded admission, not another launch.
                from unique_identifier_msgs.msg import UUID
                assert node.command('START', root=UUID(), request_id=request_id).accepted
            wait_for(lambda: node.step_index == 1)
            if case != 'stale':
                wait_for(lambda: node.status.phase == 'NAVIGATE')
            if case == 'identity':
                assert not node.command('PAUSE', instance='old-instance').accepted
                request_id = uuid.uuid4().bytes
                assert node.command('PAUSE', request_id=request_id).accepted
                assert node.command('PAUSE', request_id=request_id).accepted
                assert node.command('RESUME', request_id=request_id).reason == 'REQUEST_ID_CONFLICT'
                wait_for(lambda: node.status.phase == 'PAUSED')
                assert node.pause_calls == 1
                assert node.command('RESUME').accepted
            elif case in ('pause-resume', 'cancel-pausing', 'unconfirmed', 'reject'):
                request_id = uuid.uuid4().bytes
                assert node.command('PAUSE', request_id=request_id).accepted
                wait_for(lambda: node.pause_calls == 1)
                if case == 'cancel-pausing':
                    assert node.command('CANCEL').accepted
                elif case == 'pause-resume':
                    wait_for(lambda: node.status.phase == 'PAUSED')
                    before = len(node.progress)
                    time.sleep(.65)
                    assert len(node.progress) > before
                    assert node.advances == [0, 1] and node.goals == 1
                    assert node.cancel_at is None
                    assert node.command('RESUME').accepted
            elif case.startswith(('return','restore')) or case == 'finish':
                assert not node.command('RESUME_MISSION').accepted
                assert node.command('END_MISSION' if case=='finish' else 'RETURN').accepted
                if case == 'return-pause' or case.startswith('restore') or case == 'finish':
                    wait_for(lambda: node.status.phase == 'RETURN')
                    assert node.command('PAUSE').accepted
                    wait_for(lambda: node.status.phase == 'PAUSED')
                    before = len(node.progress)
                    time.sleep(.3)
                    assert len(node.progress) > before
                    if case.startswith('restore'):
                        request_id=uuid.uuid4().bytes
                        assert node.command('RESUME_MISSION',request_id=request_id).accepted
                        if case != 'restore-early':
                            assert node.command('RESUME_MISSION',request_id=request_id).accepted
                        wait_for(lambda:node.restore_calls==1)
                        if case == 'restore-reject':
                            wait_for(lambda:not node.return_override or node.status.phase=='PAUSED')
                            time.sleep(.1)
                            assert node.command('RESUME').accepted
                        elif case=='restore-twice':
                            wait_for(lambda:node.status.plan_revision==2 and node.status.phase=='NAVIGATE')
                            assert node.command('RETURN').accepted
                            wait_for(lambda:node.status.phase=='RETURN' and node.status.plan_revision==3)
                            assert node.command('PAUSE').accepted
                            wait_for(lambda:node.status.phase=='PAUSED')
                            assert node.command('RESUME_MISSION').accepted
                            wait_for(lambda:node.restore_calls==2)
                    else:
                        if case == 'finish':assert not node.command('RESUME_MISSION').accepted
                        assert node.command('RESUME').accepted
            process.wait(timeout=15)
        text = (tmp_path/'runner.log').read_text()
        expected = 1 if case in ('unconfirmed', 'stale', 'return-early','restore-unconfirmed','restore-early') else 130 if case == 'cancel-pausing' else 0
        assert process.returncode == expected, text
        assert node.goals == 1 and text.count('BT_RESULT') == 1
        if case in ('restore','restore-twice','restore-old-feedback'):
            assert node.advances == ([0,1,1,1,2,3,4] if case=='restore-twice' else [0,1,1,2,3,4])
            assert node.cancel_at is None
            assert node.advance_ids[1] != node.advance_ids[2]
            if case=='restore-twice':assert len(set(node.advance_ids[1:4]))==3
            assert 'BT_MISSION_RESTORED index=1 revision=2' in text
            if case=='restore-twice':assert 'BT_MISSION_RESTORED index=1 revision=4' in text
            assert 'BT_TREE_TERMINAL SUCCESS' in text and 'BT_RETURN_COMPLETE' not in text
        elif case.startswith('return') or case in ('restore-reject','finish'):
            assert node.advances == [0,1]
            if expected == 0:
                assert 'BT_RETURN_COMPLETE' in text and 'BT_TREE_TERMINAL SUCCESS' in text
        elif case.startswith('restore'):
            assert node.advances == [0,1]
            if case != 'restore-early':assert node.cancel_at is not None
        elif expected == 0:
            assert node.advances == list(range(5))
        else:
            assert node.cancel_at is not None and node.advances == [0, 1]
    finally:
        if process is not None and process.poll() is None:
            process.kill()
            process.wait(timeout=3)
        executor.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        node.destroy_node()
        context.shutdown()
