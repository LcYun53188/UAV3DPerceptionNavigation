#!/usr/bin/env python3
"""Isolated, source-pinned SDK16 covariance export repair; never installs SDK15."""

import sys as _sys
from pathlib import Path as _Path
_scripts = str(_Path(__file__).resolve().parents[1])
if _scripts not in _sys.path:
    _sys.path.insert(0, _scripts)
from support.paths import enable_imports as _enable_imports
_enable_imports()

import argparse
import json
import shutil
import subprocess
from pathlib import Path
from build_vio_sdk_candidate import ROOT, COMMIT, protected_baseline, sha, run

FOLDER=ROOT/'.cache/vio-sdk-candidate/v16.0.0-marginal-v1'
PREFIX=FOLDER/'ros-install/isaac_ros_visual_slam'
CONTRACT='cuvslam16_marginal_world_fixed_euler_v1_disarmed'
PROFILE=ROOT/'simulation/px4/vio/profiles/cuvslam16_marginal_candidate.json'
PATCH=ROOT/'simulation/px4/vio/patches/sdk16_marginal_covariance.patch'
HEADER=ROOT/'scripts/simulation/vio/sdk/vio_sdk16_marginal_covariance.hpp'

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--jobs',type=int,default=3)
    parser.add_argument('--ros',action='store_true')
    args=parser.parse_args()
    if not 1<=args.jobs<=8:parser.error('Use 1 to 8 jobs')
    baseline=protected_baseline()
    stock=ROOT/'.cache/vio-sdk-candidate/v16.0.0/src'
    if subprocess.check_output(['git','-C',str(stock),'rev-parse','HEAD'],text=True).strip()!=COMMIT:
        raise RuntimeError('Pinned stock SDK16 source required')
    if subprocess.check_output(['git','-C',str(stock),'status','--porcelain'],text=True).strip():
        raise RuntimeError('Stock SDK16 source must remain clean')
    FOLDER.mkdir(parents=True,exist_ok=True)
    source=FOLDER/'src';build=FOLDER/'build'
    if source.exists():
        receipt=json.loads((FOLDER/'build-receipt.json').read_text())
        if receipt['patch_sha256']!=sha(PATCH) or receipt['header_sha256']!=sha(HEADER):
            raise RuntimeError('Existing variant differs; use a new versioned folder')
        if any(sha(source/n)!=digest for n,digest in receipt['source_tree_sha256'].items()):
            raise RuntimeError('Modified variant source drift')
    else:
        shutil.copytree(stock,source,ignore=shutil.ignore_patterns('.git'))
        subprocess.run(['patch','--batch','--fuzz=0','-p1','-i',str(PATCH)],cwd=source,check=True)
        shutil.copy2(HEADER,source/'libs/imu'/HEADER.name)
    if not (source/'.git').exists():
        shutil.copytree(stock/'.git',source/'.git')
    commands=[['cmake','-S',str(source),'-B',str(build),'-DCMAKE_BUILD_TYPE=Release',
        '-DCMAKE_CUDA_ARCHITECTURES=89','-DCUDAToolkit_ROOT=/usr/local/cuda-13.2','-DUSE_RERUN=OFF'],
        ['cmake','--build',str(build),'--target','cuvslam','--parallel',str(args.jobs)],
        ['g++','-std=c++17','-O2','-DCUVSLAM_REPLAY_SDK_MAJOR=16',str(ROOT/'scripts/simulation/vio/sdk/vio_sdk_covariance_replay.cpp'),
         f'-I{source}/libs','-I/usr/include/eigen3',f'-L{build}/bin',f'-Wl,-rpath,{build}/bin','-lcuvslam','-o',str(FOLDER/'replay')]]
    for label,command in zip(('configure','build','replay'),commands):
        print('Marginal covariance candidate:',label,flush=True);run(command,FOLDER/(label+'.log'))
    if protected_baseline()!=baseline:raise RuntimeError('SDK15 baseline drift')
    artifacts=[build/'bin/libcuvslam.so',FOLDER/'replay',source/'libs/cuvslam/cuvslam2.h']
    receipt=dict(schema=1,source_commit=COMMIT,source_clean=False,source_modified=True,
        patch_sha256=sha(PATCH),header_sha256=sha(HEADER),flight_authorized=False,
        protected_baseline_sha256=baseline,commands=commands,
        source_tree_sha256={str(p.relative_to(source)):sha(p) for p in sorted(source.rglob('*')) if p.is_file() and '.git' not in p.relative_to(source).parts},
        artifacts_sha256={str(p.relative_to(ROOT)):sha(p) for p in artifacts})
    (FOLDER/'build-receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
    if args.ros:
        from build_vio_sdk16_ros_candidate import build_candidate
        build_candidate(FOLDER,PREFIX,CONTRACT,PROFILE,native_modified=True)

if __name__=='__main__':main()
