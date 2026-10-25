"""Exercise the actual runner's no-authorization startup, without a flight server."""
import os
from pathlib import Path
import selectors
import subprocess
import pytest


BINARY=Path(__file__).resolve().parents[3]/'.deps/mission-install/uav_bt/lib/uav_bt/mission_runner'
pytestmark=pytest.mark.skipif(not BINARY.exists(),reason='built BT runner required')


def launch(tmp_path,**overrides):
    env=dict(os.environ,ROS_DOMAIN_ID='78',UAV_FLIGHT_REGION_PROFILE='vio_avoidance',UAV_EXPERIMENTAL_VIO_CHAIN='1')
    env.update(overrides)
    return subprocess.Popen([str(BINARY),'--ros-args','-p','coordinator_instance:=prewarm-test',
        '-p',f'parameters_file:={tmp_path/"not-authorized.private.json"}',
        '-p','parameters_wait_s:=10.0','-p','timeout_s:=20.0'],env=env,
        stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)


@pytest.mark.parametrize('profile', ['vio_avoidance', 'obstacle_course'])
def test_actual_prewarmed_runner_cannot_dispatch_without_private_parameters(tmp_path, profile):
    process=launch(tmp_path,UAV_FLIGHT_REGION_PROFILE=profile);lines=[]
    try:
        with selectors.DefaultSelector() as selector:
            selector.register(process.stdout,selectors.EVENT_READ)
            for _ in range(10):
                assert selector.select(2),'Runner did not report prewarm state'
                line=process.stdout.readline();lines.append(line)
                if 'BT_PREWARM_WAIT' in line:break
        assert 'BT_PREWARM_WAIT' in ''.join(lines) and process.poll() is None
    finally:
        if process.poll() is None:process.terminate()
        output=process.communicate(timeout=5)[0]
    assert 'Dispatched root mission' not in ''.join(lines)+output
    assert process.returncode!=0  # Closing an unauthorized runner is not mission success.


@pytest.mark.parametrize('override',[{'ROS_DOMAIN_ID':'79'},
    {'UAV_FLIGHT_REGION_PROFILE':'W0'},{'UAV_EXPERIMENTAL_VIO_CHAIN':'0'}])
def test_prewarm_is_rejected_outside_explicit_owned_profile(tmp_path,override):
    process=launch(tmp_path,**override);output=process.communicate(timeout=5)[0]
    assert process.returncode!=0 and 'Invalid owned runner prewarm wait' in output
    assert 'Dispatched root mission' not in output
