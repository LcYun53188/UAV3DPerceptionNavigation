#!/usr/bin/env python3
"""Isolated passive Hessian capture; never changes the default SDK or optimizer."""

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
from build_vio_sdk_candidate import ROOT, COMMIT, protected_baseline, sha, run
from build_vio_sdk16_default import FOLDER as BASE, CONTRACT, PROFILE, WRAPPER_PATCH
from build_vio_sdk16_ros_candidate import build_candidate

FOLDER = ROOT/'.cache/vio-sdk-candidate/v16.0.0-conditional-v2-schur-diagnostic-v2'
PREFIX = FOLDER/'ros-install/isaac_ros_visual_slam'
PATCH = ROOT/'simulation/px4/vio/patches/sdk16_schur_diagnostic.patch'


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
    inputs = {**base['patch_inputs_sha256'], str(PATCH.relative_to(ROOT)):sha(PATCH)}
    FOLDER.mkdir(parents=True,exist_ok=True)
    receipt_path = FOLDER/'source-receipt.json'
    if source.exists():
        receipt=json.loads(receipt_path.read_text())
        if receipt['patch_inputs_sha256'] != inputs: raise RuntimeError('Use a new diagnostic variant')
        for name,digest in receipt['source_tree_sha256'].items():
            if sha(source/name)!=digest: raise RuntimeError('Diagnostic source drift: '+name)
    else:
        shutil.copytree(BASE/'src',source)
        subprocess.run(['patch','--batch','--fuzz=0','-p1','-i',str(PATCH)],cwd=source,check=True)
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
    commands=[['cmake','-S',str(source),'-B',str(build),'-DCMAKE_BUILD_TYPE=Release',
        '-DCMAKE_CUDA_ARCHITECTURES=89','-DCUDAToolkit_ROOT=/usr/local/cuda-13.2','-DUSE_RERUN=OFF'],
        ['cmake','--build',str(build),'--target','cuvslam','--parallel',str(args.jobs)]]
    for label,command in zip(('configure','build'),commands):
        print('Passive Schur diagnostic:',label,flush=True)
        run(command,FOLDER/(label+'.log'))
    receipt.update(schema=1,flight_authorized=False,optimizer_changed=False,
        diagnostic_only=True,protected_baseline_sha256=baseline,commands=commands,
        artifacts_sha256={str((build/'bin/libcuvslam.so').relative_to(ROOT)):sha(build/'bin/libcuvslam.so')})
    (FOLDER/'build-receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
    build_candidate(FOLDER,PREFIX,CONTRACT,PROFILE,native_modified=True,
        native_inputs=[ROOT/name for name in inputs]+[ROOT/'scripts/build/build_vio_sdk16_schur_diagnostic.py'],
        wrapper_patch=WRAPPER_PATCH,wrapper_headers=[ROOT/'scripts/simulation/vio/sdk/vio_sdk16_world_covariance.hpp'])
    if protected_baseline()!=baseline: raise RuntimeError('Default SDK15 drift')


if __name__=='__main__': main()
