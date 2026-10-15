#!/usr/bin/env python3
"""Build default SDK16 VIO, preserving optimization and the SDK15 rollback."""
import argparse
import json
import os
import shutil
import subprocess
from build_vio_sdk_candidate import ROOT, COMMIT, URL, protected_baseline, sha, run

FOLDER = ROOT/'.cache/vio-sdk-candidate/v16.0.0-conditional-v2'
PREFIX = FOLDER/'ros-install/isaac_ros_visual_slam'
CONTRACT = 'cuvslam16_conditional_world_base_v2_disarmed'
PROFILE = ROOT/'simulation/px4/vio/cuvslam16_default.json'
PATCH = ROOT/'simulation/px4/vio/sdk16_conditional_world_covariance.patch'
WRAPPER_PATCH = ROOT/'simulation/px4/vio/sdk16_world_odometry_covariance.patch'
HEADERS = [ROOT/'scripts/vio_sdk16_conditional_covariance.hpp',
           ROOT/'scripts/vio_sdk16_marginal_covariance.hpp']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--jobs', type=int, default=3)
    args = parser.parse_args()
    if not 1 <= args.jobs <= 8: parser.error('Use 1 to 8 jobs')
    baseline = protected_baseline()
    stock = ROOT/'.cache/vio-sdk-candidate/v16.0.0/src'
    if not stock.exists():
        stock.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run(['git','clone','--depth','1','--branch','v16.0.0',URL,str(stock)],
                       env=dict(os.environ,GIT_LFS_SKIP_SMUDGE='1'),check=True)
    if subprocess.check_output(['git','-C',str(stock),'rev-parse','HEAD'],text=True).strip() != COMMIT:
        raise RuntimeError('Build the pinned stock SDK16 first')
    if subprocess.check_output(['git','-C',str(stock),'status','--porcelain'],text=True).strip():
        raise RuntimeError('Stock SDK16 must remain clean')
    FOLDER.mkdir(parents=True, exist_ok=True)
    source, build = FOLDER/'src', FOLDER/'build'
    expected_patch = {str(p.relative_to(ROOT)):sha(p) for p in [PATCH,*HEADERS]}
    if source.exists():
        receipt_path=FOLDER/'build-receipt.json'
        if not receipt_path.exists(): receipt_path=FOLDER/'source-receipt.json'
        receipt = json.loads(receipt_path.read_text())
        if receipt['patch_inputs_sha256'] != expected_patch:
            raise RuntimeError('Use a new variant folder for changed native patches')
        if any(sha(source/n) != value for n,value in receipt['source_tree_sha256'].items()):
            raise RuntimeError('Native source drift')
    else:
        shutil.copytree(stock, source, ignore=shutil.ignore_patterns('.git'))
        subprocess.run(['patch','--batch','--fuzz=0','-p1','-i',str(PATCH)],cwd=source,check=True)
        for header in HEADERS: shutil.copy2(header, source/'libs/imu'/header.name)
        (FOLDER/'source-receipt.json').write_text(json.dumps(dict(
            source_commit=COMMIT,patch_inputs_sha256=expected_patch,
            source_tree_sha256={str(p.relative_to(source)):sha(p) for p in sorted(source.rglob('*')) if p.is_file() and '.git' not in p.relative_to(source).parts}),indent=2)+'\n')
    # Give UpdateVersion.cmake the SDK's own Git identity, rather than allowing
    # git to walk up to the unrelated workspace repository. Git metadata is
    # excluded from source-byte receipts; the patched source remains identical.
    if not (source/'.git').exists():
        shutil.copytree(stock/'.git', source/'.git')
    if subprocess.check_output(['git','-C',str(source),'rev-parse','HEAD'],text=True).strip() != COMMIT:
        raise RuntimeError('Default SDK16 Git identity drift')
    # Seed only downloaded archives. FetchContent still validates upstream URL
    # hashes and extracts them itself; no cached binaries or mutable source dirs.
    cached=ROOT/'.cache/vio-sdk-candidate/v16.0.0-conditional-v1/build/_deps'
    seeds={}
    if cached.exists():
        for p in sorted(cached.rglob('*')):
            if p.is_file() and p.name.endswith(('.tar.xz','.tar.gz','.tgz','.zip')):
                target=build/'_deps'/p.relative_to(cached)
                target.parent.mkdir(parents=True,exist_ok=True)
                if not target.exists() or sha(target)!=sha(p):shutil.copy2(p,target)
                seeds[str(target.relative_to(ROOT))]=sha(target)
    (FOLDER/'download-cache.json').write_text(json.dumps(seeds,indent=2)+'\n')
    commands = [
        ['cmake','-S',str(source),'-B',str(build),'-DCMAKE_BUILD_TYPE=Release',
         '-DCMAKE_CUDA_ARCHITECTURES=89','-DCUDAToolkit_ROOT=/usr/local/cuda-13.2','-DUSE_RERUN=OFF'],
        ['cmake','--build',str(build),'--target','cuvslam','--parallel',str(args.jobs)],
        ['g++','-std=c++17','-O2','-DCUVSLAM_REPLAY_SDK_MAJOR=16',str(ROOT/'scripts/vio_sdk_covariance_replay.cpp'),
         f'-I{source}/libs','-I/usr/include/eigen3',f'-L{build}/bin',f'-Wl,-rpath,{build}/bin',
         '-lcuvslam','-o',str(FOLDER/'replay')]]
    for label,command in zip(('configure','build','replay'),commands):
        print('Default SDK16:',label,flush=True)
        run(command,FOLDER/(label+'.log'))
    artifacts = [build/'bin/libcuvslam.so',FOLDER/'replay',source/'libs/cuvslam/cuvslam2.h']
    receipt = dict(schema=1,source_commit=COMMIT,source_clean=False,source_modified=True,
        flight_authorized=False,protected_baseline_sha256=baseline,commands=commands,
        covariance_kind='pose conditional on nuisance states; corrected local IMU to world base; world small rotation, no Euler roundtrip',
        optimizer_changed=False,patch_inputs_sha256=expected_patch,
        source_tree_sha256={str(p.relative_to(source)):sha(p) for p in sorted(source.rglob('*')) if p.is_file() and '.git' not in p.relative_to(source).parts},
        artifacts_sha256={str(p.relative_to(ROOT)):sha(p) for p in artifacts})
    (FOLDER/'build-receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
    from build_vio_sdk16_ros_candidate import build_candidate
    build_candidate(FOLDER,PREFIX,CONTRACT,PROFILE,native_modified=True,
        native_inputs=[PATCH,*HEADERS,ROOT/'scripts/build_vio_sdk16_default.py'],wrapper_patch=WRAPPER_PATCH,
        wrapper_headers=[ROOT/'scripts/vio_sdk16_world_covariance.hpp'])
    if protected_baseline() != baseline: raise RuntimeError('SDK15 rollback drift')


if __name__ == '__main__': main()
