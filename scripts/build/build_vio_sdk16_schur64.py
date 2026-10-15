#!/usr/bin/env python3
"""Isolated double precision Schur candidate; never replaces the default SDK."""
import argparse
import json
import re
import shutil
import subprocess
from build_vio_sdk_candidate import ROOT, COMMIT, protected_baseline, sha, run
from build_vio_sdk16_default import FOLDER as BASE, WRAPPER_PATCH
from build_vio_sdk16_ros_candidate import build_candidate

FOLDER = ROOT/'.cache/vio-sdk-candidate/v16.0.0-conditional-v3-schur64'
CONTRACT = 'cuvslam16_conditional_world_base_v3_schur64_disarmed'
PROFILE = ROOT/'simulation/px4/vio/cuvslam16_schur64.json'
PREFIX = FOLDER/'ros-install/isaac_ros_visual_slam'
PATCH = ROOT/'simulation/px4/vio/sdk16_schur64.patch'


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
    dependency_flags=[]
    dependencies={}
    for name in ('dense_hash_map','cnpy','circularbuffer'):
        cmake=(source/'cmake/ext'/(name+'.cmake')).read_text()
        pin=re.search(r'set\('+name.upper()+r'_COMMIT "([0-9a-f]{40})"\)',cmake).group(1)
        cached=BASE/'build/_deps'/(name+'-src')
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
    receipt.update(schema=1,flight_authorized=False,optimizer_changed=True,
        pinned_local_dependencies=dependencies,
        numeric_marginalization_precision="float64; original 1e-6 cutoff; float storage",protected_baseline_sha256=baseline,commands=commands,
        artifacts_sha256={str((build/'bin/libcuvslam.so').relative_to(ROOT)):sha(build/'bin/libcuvslam.so')})
    (FOLDER/'build-receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
    build_candidate(FOLDER,PREFIX,CONTRACT,PROFILE,native_modified=True,
        native_inputs=[ROOT/name for name in inputs]+[ROOT/'scripts/build_vio_sdk16_schur64.py'],
        wrapper_patch=WRAPPER_PATCH,wrapper_headers=[ROOT/'scripts/vio_sdk16_world_covariance.hpp'])
    if protected_baseline()!=baseline: raise RuntimeError('Default SDK15 drift')


if __name__=='__main__': main()
