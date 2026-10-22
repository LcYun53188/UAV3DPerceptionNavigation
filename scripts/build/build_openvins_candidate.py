#!/usr/bin/env python3
"""Build the pinned OpenVINS candidate in its own prefix; no production install."""

import sys as _sys
from pathlib import Path as _Path
_scripts = str(_Path(__file__).resolve().parents[1])
if _scripts not in _sys.path:
    _sys.path.insert(0, _scripts)
from support.paths import enable_imports as _enable_imports
_enable_imports()

import json
import os
from pathlib import Path
import subprocess
from build_vio_sdk_candidate import ROOT, sha, protected_baseline
from prepare_openvins_config import COMMIT

FOLDER=ROOT/'.cache/vio-openvins'
PREFIX=FOLDER/'install'
CONTRACT='openvins_full_state_world_base_v1_disarmed'
SOURCE=FOLDER/'src'
LIBRARIES=[PREFIX/package/'lib'/('lib'+package+'_lib.so') for package in ('ov_core','ov_init','ov_msckf')]
BINARY=FOLDER/'replay-build/openvins_node'

def main():
    FOLDER.mkdir(parents=True,exist_ok=True)
    if not SOURCE.exists():
        subprocess.run(['git','clone','https://github.com/rpng/open_vins.git',str(SOURCE)],check=True)
        subprocess.run(['git','-C',str(SOURCE),'checkout','--detach',COMMIT],check=True)
    if subprocess.check_output(['git','-C',str(SOURCE),'rev-parse','HEAD'],text=True).strip()!=COMMIT:
        raise RuntimeError('OpenVINS source differs from pinned commit')
    patch=ROOT/'simulation/px4/vio/openvins/jazzy_headers.patch'
    dirty=subprocess.check_output(['git','-C',str(SOURCE),'diff','--name-only'],text=True).splitlines()
    if not dirty:subprocess.run(['git','apply',str(patch)],cwd=SOURCE,check=True)
    elif set(dirty)!={'ov_msckf/src/ros/ROS2Visualizer.h','ov_msckf/src/ros/ROSVisualizerHelper.h'}:
        raise RuntimeError('Unexpected OpenVINS source modifications')
    subprocess.run(['git','apply','--reverse','--check',str(patch)],cwd=SOURCE,check=True)
    baseline=protected_baseline()
    commands=[['colcon','--log-base',str(FOLDER/'ros-log'),'build','--base-paths',str(SOURCE),
        '--packages-select','ov_core','ov_init','ov_msckf','--build-base',str(FOLDER/'build'),
        '--install-base',str(PREFIX),'--executor','sequential','--cmake-args','-DCMAKE_BUILD_TYPE=Release',
        '-DCMAKE_CXX_FLAGS_RELEASE=-O3 -DNDEBUG -g0','-DENABLE_ARUCO_TAGS=OFF','-DBUILD_TESTING=OFF']]
    with (FOLDER/'candidate-build.log').open('w') as log:
        subprocess.run([str(ROOT/'scripts/with_venv.sh'),*commands[0]],stdout=log,stderr=subprocess.STDOUT,
                       env=dict(os.environ,CMAKE_BUILD_PARALLEL_LEVEL='2'),check=True)
    command=[str(ROOT/'scripts/with_venv.sh'),'bash','-e','-c',
        'source .cache/vio-openvins/install/setup.bash; '
        'cmake -S simulation/px4/vio/openvins -B .cache/vio-openvins/replay-build -DCMAKE_BUILD_TYPE=Release; '
        'cmake --build .cache/vio-openvins/replay-build -j2']
    with (FOLDER/'candidate-wrapper-build.log').open('w') as log:
        subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
    files=[*LIBRARIES,BINARY,FOLDER/'replay-build/openvins_replay',patch,Path(__file__),
        ROOT/'simulation/px4/vio/openvins/CMakeLists.txt',ROOT/'scripts/simulation/vio/sdk/prepare_openvins_config.py',
        ROOT/'scripts/simulation/vio/sdk/openvins_ros_node.cpp',ROOT/'scripts/simulation/vio/sdk/openvins_covariance_replay.cpp',ROOT/'scripts/simulation/vio/sdk/openvins_pose_covariance.hpp']
    names=subprocess.check_output(['git','-C',str(SOURCE),'ls-files'],text=True).splitlines()
    files += [SOURCE/n for n in names if (SOURCE/n).is_file()]
    if protected_baseline()!=baseline:raise RuntimeError('SDK15 production baseline drift')
    receipt=dict(schema=1,contract=CONTRACT,source_commit=COMMIT,source_modified=True,
        source_patch_sha256=sha(patch),source_family='OpenVINS',sdk_version=None,
        scope='isolated stereo IMU covariance candidate; disarmed simulation only',flight_authorized=False,
        protected_baseline_sha256=baseline,commands=[*commands,command],
        expected_sha256={str(p.relative_to(ROOT)):sha(p) for p in files})
    (FOLDER/'contract.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print('OpenVINS candidate built; flight authorization remains false')

def loaded_receipt(pid):
    libraries={}
    for line in Path(f'/proc/{pid}/maps').read_text().splitlines():
        name=line.split()[-1]
        if name.endswith(('/libov_core_lib.so','/libov_init_lib.so','/libov_msckf_lib.so','/libcuvslam.so')):
            libraries[name]=sha(Path(name))
    return dict(passed=set(libraries)==set(map(str,LIBRARIES)),root_pid=pid,
        loaded_libraries_sha256=libraries,scope='actual owned native OpenVINS process /proc maps')

if __name__=='__main__':main()
