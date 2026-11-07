#!/usr/bin/env python3
"""Isolated stereo/IMU stationary initialization; retains dynamic initialization fallback."""

import sys as _sys
from pathlib import Path as _Path
_scripts = str(_Path(__file__).resolve().parents[1])
if _scripts not in _sys.path:
    _sys.path.insert(0, _scripts)
from support.paths import enable_imports as _enable_imports
_enable_imports()

import argparse
import json
import re
import shutil
import subprocess
from build_vio_sdk_candidate import ROOT, COMMIT, protected_baseline, sha, run, canonical_input_hashes
from build_vio_sdk16_schur64 import FOLDER as BASE
from build_vio_sdk16_default import WRAPPER_PATCH
from build_vio_sdk16_ros_candidate import build_candidate

FOLDER = ROOT/'.cache/vio-sdk-candidate/v16.0.0-conditional-v4-stationary-r7'
CONTRACT = 'cuvslam16_conditional_world_base_v4_stationary_disarmed'
PROFILE = ROOT/'simulation/px4/vio/profiles/cuvslam16_stationary.json'
PREFIX = FOLDER/'ros-install/isaac_ros_visual_slam'
HEADER = ROOT/'scripts/simulation/vio/sdk/vio_stationary_initialization.hpp'
ZERO_HEADER = ROOT/'scripts/simulation/vio/sdk/vio_zero_flow_seed.hpp'
PATCH = ROOT/'simulation/px4/vio/patches/sdk16_stationary_init.patch'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--jobs', type=int, default=3)
    args = parser.parse_args()
    if not 1 <= args.jobs <= 8: parser.error('Use 1 to 8 jobs')
    baseline = protected_baseline()
    base = json.loads((BASE/'build-receipt.json').read_text())
    if base['source_commit'] != COMMIT: raise RuntimeError('Pinned base required')
    for name,digest in base['source_tree_sha256'].items():
        if sha(BASE/'src'/name) != digest: raise RuntimeError('Base source drift: '+name)
    source,build = FOLDER/'src',FOLDER/'build'
    inputs = canonical_input_hashes({**base['patch_inputs_sha256'], str(PATCH.relative_to(ROOT)):sha(PATCH), str(HEADER.relative_to(ROOT)):sha(HEADER),str(ZERO_HEADER.relative_to(ROOT)):sha(ZERO_HEADER)})
    FOLDER.mkdir(parents=True,exist_ok=True)
    receipt_path = FOLDER/'source-receipt.json'
    if source.exists():
        receipt=json.loads(receipt_path.read_text())
        if canonical_input_hashes(receipt['patch_inputs_sha256']) != inputs:
            raise RuntimeError('Use a new diagnostic variant')
        for name,digest in receipt['source_tree_sha256'].items():
            if sha(source/name)!=digest: raise RuntimeError('Diagnostic source drift: '+name)
    else:
        shutil.copytree(BASE/'src',source)
        subprocess.run(['patch','--batch','--fuzz=0','-p1','-i',str(PATCH)],cwd=source,check=True)
        shutil.copy2(HEADER,source/'libs/imu'/HEADER.name)
        shutil.copy2(ZERO_HEADER,source/'libs/common'/ZERO_HEADER.name)
        receipt=dict(source_commit=COMMIT,patch_inputs_sha256=inputs,source_tree_sha256={
            str(p.relative_to(source)):sha(p) for p in sorted(source.rglob('*'))
            if p.is_file() and '.git' not in p.relative_to(source).parts})
        receipt_path.write_text(json.dumps(receipt,indent=2)+'\n')
    # Seed downloaded archives, retaining FetchContent's own URL hash checks.
    for p in sorted((BASE/'build/_deps').rglob('*')):
        if p.is_file() and p.name.endswith(('.tar.xz','.tar.gz','.tgz','.zip')):
            target=build/'_deps'/p.relative_to(BASE/'build/_deps')
            target.parent.mkdir(parents=True,exist_ok=True)
            if not target.exists(): shutil.copy2(p,target)
    dependency_flags=[]
    dependencies={}
    for name in ('dense_hash_map','cnpy','circularbuffer'):
        cmake=(source/'cmake/ext'/(name+'.cmake')).read_text()
        pin=re.search(r'set\('+name.upper()+r'_COMMIT "([0-9a-f]{40})"\)',cmake).group(1)
        cached=BASE/'dependency-sources'/name
        if (subprocess.check_output(['git','-C',str(cached),'rev-parse','HEAD'],text=True).strip()!=pin
                or subprocess.check_output(['git','-C',str(cached),'status','--porcelain'],text=True).strip()):
            raise RuntimeError('Cached dependency must match clean upstream pin: '+name)
        local=FOLDER/'dependency-sources'/name
        if not local.exists():shutil.copytree(cached,local)
        hashes={str(p.relative_to(cached)):sha(p) for p in sorted(cached.rglob('*'))
                if p.is_file() and '.git' not in p.relative_to(cached).parts}
        if any(not (local/n).is_file() or sha(local/n)!=digest for n,digest in hashes.items()):
            raise RuntimeError('Local dependency drift: '+name)
        dependencies[name]=dict(commit=pin,source_tree_sha256=hashes)
        dependency_flags.append('-DFETCHCONTENT_SOURCE_DIR_'+name.upper()+'='+str(local))
    commands=[['cmake','-S',str(source),'-B',str(build),'-DCMAKE_BUILD_TYPE=Release',
        '-DCMAKE_CUDA_ARCHITECTURES=89','-DCUDAToolkit_ROOT=/usr/local/cuda-13.2','-DUSE_RERUN=OFF'],
        ['cmake','--build',str(build),'--target','cuvslam','--parallel',str(args.jobs)]]
    commands[0]+=dependency_flags
    for label,command in zip(('configure','build'),commands):
        print('Double precision Schur candidate:',label,flush=True)
        run(command,FOLDER/(label+'.log'))
    receipt.update(schema=1,flight_authorized=False,optimizer_changed=True,patch_inputs_sha256=inputs,
        pinned_local_dependencies=dependencies,
        numeric_marginalization_precision="float64; original 1e-6 cutoff; float storage",protected_baseline_sha256=baseline,commands=commands,
        artifacts_sha256={str((build/'bin/libcuvslam.so').relative_to(ROOT)):sha(build/'bin/libcuvslam.so')})
    (FOLDER/'build-receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
    # Reuse the exact reviewed wrapper bytes: the SDK public header/ABI is
    # unchanged. Only the private native initializer was rebuilt.
    if sha(source/'libs/cuvslam/cuvslam2.h')!=sha(BASE/'src/libs/cuvslam/cuvslam2.h'):
        raise RuntimeError('Wrapper reuse requires identical public SDK header')
    wrapped=json.loads((BASE/'ros-contract.json').read_text())
    for name,digest in wrapped['expected_sha256'].items():
        if sha(ROOT/name)!=digest:raise RuntimeError('Reviewed wrapper/base drift: '+name)
    shutil.copytree(BASE/'ros-install',FOLDER/'ros-install',dirs_exist_ok=True)
    shutil.copy2(build/'bin/libcuvslam.so',PREFIX/'lib/libcuvslam.so')
    expected=dict(wrapped['expected_sha256'])
    files=[p for p in (FOLDER/'ros-install').rglob('*') if p.is_file()]
    files += [source/n for n in receipt['source_tree_sha256']]
    files += [PROFILE,PATCH,HEADER,ZERO_HEADER,_Path(__file__).resolve(),FOLDER/'build-receipt.json']
    expected.update({str(p.relative_to(ROOT)):sha(p) for p in files})
    wrapped.update(contract=CONTRACT,candidate_profile=json.loads(PROFILE.read_text()),
        expected_sha256=expected,wrapper_reused_from=str(BASE/'ros-contract.json'),
        scope='Observed-static initialization; original dynamic fallback; simulation candidate only')
    (FOLDER/'ros-contract.json').write_text(json.dumps(wrapped,indent=2)+'\n')
    print('Stationary native candidate built; identical reviewed ROS wrapper reused',flush=True)
    if protected_baseline()!=baseline: raise RuntimeError('Default SDK15 drift')


if __name__=='__main__': main()
