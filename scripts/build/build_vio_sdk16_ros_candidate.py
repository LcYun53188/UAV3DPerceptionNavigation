#!/usr/bin/env python3
"""Rebuild the disarmed SDK16 ROS candidate without changing installed SDK15."""

import sys as _sys
from pathlib import Path as _Path
_scripts = str(_Path(__file__).resolve().parents[1])
if _scripts not in _sys.path:
    _sys.path.insert(0, _scripts)
from support.paths import enable_imports as _enable_imports
_enable_imports()

import hashlib
import json
from pathlib import Path
import shutil
import subprocess
from build_vio_sdk_candidate import ROOT, COMMIT, protected_baseline, sha

FOLDER=ROOT/'.cache/vio-sdk-candidate/v16.0.0'
PREFIX=FOLDER/'ros-install/isaac_ros_visual_slam'
PATCH=ROOT/'simulation/px4/vio/patches/sdk16_ros_candidate.patch'
CONTRACT='cuvslam16_world_fixed_sdk_euler_v1_disarmed'
PROFILE=ROOT/'simulation/px4/vio/profiles/cuvslam16_ros_candidate.json'


def build_candidate(FOLDER=FOLDER, PREFIX=PREFIX, CONTRACT=CONTRACT, PROFILE=PROFILE, *, native_modified=False, native_inputs=None, wrapper_patch=None, wrapper_headers=()):
    profile=json.loads(PROFILE.read_text())
    if profile['contract']!=CONTRACT or profile['source_commit']!=COMMIT or profile['flight_authorized']:
        raise RuntimeError('SDK16 ROS candidate profile differs from reviewed source')
    source=FOLDER/'src'; native=FOLDER/'build/bin/libcuvslam.so'
    receipt=json.loads((FOLDER/'build-receipt.json').read_text())
    if receipt['source_commit']!=COMMIT or receipt['artifacts_sha256'][str(native.relative_to(ROOT))]!=sha(native):
        raise RuntimeError('Build the clean pinned native SDK16 candidate first')
    if not native_modified and subprocess.check_output(['git','-C',str(source),'status','--porcelain'],text=True).strip():
        raise RuntimeError('SDK16 source must be clean')
    baseline=protected_baseline()
    base=ROOT/'src/isaac_ros_visual_slam/isaac_ros_visual_slam'
    shadow=FOLDER/'ros-src/isaac_ros_visual_slam'
    if shadow.exists():shutil.rmtree(shadow)
    shutil.copytree(base,shadow,ignore=shutil.ignore_patterns('__pycache__','test_cases'))
    subprocess.run(['patch','--batch','--fuzz=0','-p1','-i',str(PATCH)],cwd=shadow,check=True)
    if wrapper_patch:
        subprocess.run(['patch','--batch','--fuzz=0','-p1','-i',str(wrapper_patch)],cwd=shadow,check=True)
    shutil.copy2(ROOT/'scripts/simulation/vio/sdk/vio_sdk16_covariance.hpp',shadow/'include/vio_sdk16_covariance.hpp')
    for header in wrapper_headers: shutil.copy2(header,shadow/'include'/header.name)
    command=[str(ROOT/'scripts/with_venv.sh'),'colcon','--log-base',str(FOLDER/'ros-log'),'build',
        '--base-paths',str(FOLDER/'ros-src'),'--build-base',str(FOLDER/'ros-build'),
        '--install-base',str(FOLDER/'ros-install'),'--cmake-args','-DBUILD_TESTING=OFF',
        '-DCMAKE_BUILD_TYPE=Release',f'-DSDK16_SOURCE={source}',f'-DSDK16_LIBRARY={native}']
    import os
    with (FOLDER/'ros-build.log').open('w') as log:
        subprocess.run(command,env=dict(os.environ,CMAKE_BUILD_PARALLEL_LEVEL='2'),stdout=log,stderr=subprocess.STDOUT,check=True)
    files=[PREFIX/'lib/libcuvslam.so',PREFIX/'lib/libvisual_slam_node.so',
           PREFIX/'lib/isaac_ros_visual_slam/isaac_ros_visual_slam',PATCH,ROOT/'scripts/simulation/vio/sdk/vio_sdk16_covariance.hpp',
           source/'libs/cuvslam/cuvslam2.h',PROFILE,FOLDER/'build-receipt.json']
    if native_modified:
        files += list(native_inputs) if native_inputs is not None else [ROOT/'simulation/px4/vio/patches/sdk16_marginal_covariance.patch',
                  ROOT/'scripts/simulation/vio/sdk/vio_sdk16_marginal_covariance.hpp',ROOT/'scripts/build/build_vio_sdk16_marginal_candidate.py']
        files += [source/n for n in receipt['source_tree_sha256'] if n.startswith('libs/') and (n.endswith('.cpp') or n.endswith('.hpp') or n.endswith('.h'))]
    if wrapper_patch: files.append(wrapper_patch)
    files += list(wrapper_headers)
    for p in sorted(shadow.rglob('*')):
        if p.suffix in ('.cpp','.hpp','.h') and p.is_file():files.append(p)
    if protected_baseline()!=baseline:raise RuntimeError('Protected SDK15 baseline drifted')
    result=dict(schema=1,contract=CONTRACT,source_commit=COMMIT,source_clean=not native_modified,source_modified=native_modified,
        sdk_version=profile['sdk_version'],scope='experimental disarmed ROS/PX4 integration only',flight_authorized=False,
        covariance=profile.get('covariance_kind', 'SDK16 world Euler -> ROS world-fixed small rotation; full 6x6 Jacobian'),
        initialization_gate='matching Track gravity present norm [8,11], finite velocity and biases',
        keyframe_max_seconds=1,protected_baseline_sha256=baseline,build_command=command,
        candidate_profile=profile,
        expected_sha256={str(p.relative_to(ROOT)):sha(p) for p in files},
        wrapper_base_sha256={str(p.relative_to(base)):sha(p) for p in sorted(base.rglob('*')) if p.is_file() and p.suffix in ('.cpp','.hpp','.h')})
    (FOLDER/'ros-contract.json').write_text(json.dumps(result,indent=2)+'\n')
    print('SDK16 isolated ROS candidate built:',PREFIX,flush=True)


if __name__=='__main__':build_candidate()
