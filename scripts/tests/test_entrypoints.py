"""Exercise relocated tools from outside the workspace without starting ROS or Gazebo."""
import json
from pathlib import Path
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]


def test_public_launcher_previews_the_canonical_runner_from_another_directory(tmp_path):
    result = subprocess.run(
        [str(ROOT / 'scripts/sim.sh'), 'run', 'full', '--input-source', 'virtual',
         '--dry-run'], cwd=tmp_path, capture_output=True, text=True,
        check=True,
    )
    assert str(ROOT / 'scripts/simulation/manual.py') in result.stdout
    assert '--program full --input-source virtual' in result.stdout
    assert not list(tmp_path.iterdir())


def test_old_and_canonical_algorithm_commands_have_the_same_help(tmp_path):
    outputs = []
    for path in ('scripts/compat/sim_control.py', 'scripts/simulation/algorithm/sim_control.py'):
        result = subprocess.run([sys.executable, str(ROOT / path), '--help'],
                                cwd=tmp_path, capture_output=True, text=True, check=True)
        outputs.append(result.stdout)
    assert outputs[0] == outputs[1]
    assert 'export-mesh' in outputs[0]


def test_relocated_tools_locate_resources_and_hash_the_actual_runner(tmp_path):
    code = '''import json, sys
from pathlib import Path
sys.path.insert(0, sys.argv[1])
from support.paths import enable_imports
enable_imports()
from sim_validation import ROOT
from prepare_vio_sensor_assets import PROFILE
from simulation.backends import source_hashes
print(json.dumps(dict(root=str(ROOT), sensors=str(PROFILE), hashes=source_hashes())))
'''
    result = subprocess.run([sys.executable, '-c', code, str(ROOT / 'scripts')],
                            cwd=tmp_path, capture_output=True, text=True, check=True)
    evidence = json.loads(result.stdout)
    assert Path(evidence['root']) == ROOT
    assert Path(evidence['sensors']).is_relative_to(ROOT / 'simulation/px4/vio/profiles')
    assert Path(evidence['sensors']).is_file()
    assert 'scripts/simulation/vio/runtime/run_px4_vio_sensors.py' in evidence['hashes']
    assert 'scripts/simulation/pocket/pocket_operator_session.py' in evidence['hashes']


@pytest.mark.parametrize('tool', [
    'hardware/lidar/run_mid360_fastlio_rviz.sh',
    'hardware/px4/run_px4_microxrce.sh',
    'diagnostics/check_nav_stack.sh',
    'diagnostics/record_nav_debug_bag.sh',
    'diagnostics/run_rviz_nav.sh',
])
def test_relocated_shell_help_from_another_directory(tool, tmp_path):
    result = subprocess.run([str(ROOT / 'scripts' / tool), '--help'], cwd=tmp_path,
                            capture_output=True, text=True, check=True)
    assert 'Usage:' in result.stdout
    assert not list(tmp_path.iterdir())
