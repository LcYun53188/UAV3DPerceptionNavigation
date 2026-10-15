#!/usr/bin/env python3
"""Build a pinned native SDK candidate in cache, without installing ROS or PX4.

The diagnostic replay has its own RPATH. The SDK15 install and its contract
are never rewritten. Only the source-reviewed SDK16 tag is supported here.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
# Keep shared runtime modules available through canonical and compatibility paths.
sys.path.insert(0, str(ROOT / "scripts"))
COMMIT = '05588424d0440ac80af21080fd1650408fd9282a'
URL = 'https://github.com/nvidia-isaac/cuVSLAM.git'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def protected_baseline():
    files = [ROOT / 'install_uav/isaac_ros_visual_slam/lib/libcuvslam.so',
             ROOT / 'simulation/px4/vio/pose_contract.json']
    return {str(p.relative_to(ROOT)): sha(p) for p in files}


def run(command, logfile):
    with logfile.open('w') as log:
        subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--jobs', type=int, default=4)
    parser.add_argument('--cuda-root', type=Path, default=Path('/usr/local/cuda-13.2'))
    parser.add_argument('--cuda-architecture', default='89')
    args = parser.parse_args()
    profile_path = ROOT / 'simulation/px4/vio/cuvslam16_candidate.json'
    profile = json.loads(profile_path.read_text())
    if profile['source_commit'] != COMMIT or profile['source_url'] != URL or profile['sdk_major'] != 16:
        raise RuntimeError('Candidate profile differs from the source-reviewed build')
    if not 1 <= args.jobs <= 8 or not args.cuda_architecture.isdigit():
        parser.error('Use 1–8 jobs and a numeric CUDA architecture')
    if not (args.cuda_root / 'bin/nvcc').is_file():
        parser.error('CUDA compiler missing')
    baseline = protected_baseline()
    folder = ROOT / '.cache/vio-sdk-candidate/v16.0.0'
    folder.mkdir(parents=True, exist_ok=True)
    source, build = folder / 'src', folder / 'build'
    if not source.exists():
        env = dict(os.environ, GIT_LFS_SKIP_SMUDGE='1')
        subprocess.run(['git', 'clone', '--depth', '1', '--branch', 'v16.0.0', URL, str(source)],
                       env=env, check=True)
    actual = subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip()
    dirty = subprocess.check_output(['git', '-C', str(source), 'status', '--porcelain'], text=True)
    if actual != COMMIT or dirty.strip():
        raise RuntimeError('Candidate source must be clean and match the pinned commit')
    commands = [
        ['cmake', '-S', str(source), '-B', str(build), '-DCMAKE_BUILD_TYPE=Release',
         f'-DCMAKE_CUDA_ARCHITECTURES={args.cuda_architecture}', f'-DCUDAToolkit_ROOT={args.cuda_root}',
         '-DUSE_RERUN=OFF'],
        ['cmake', '--build', str(build), '--target', 'cuvslam', '--parallel', str(args.jobs)],
        ['g++', '-std=c++17', '-O2', '-DCUVSLAM_REPLAY_SDK_MAJOR=16',
         str(ROOT / 'scripts/vio_sdk_covariance_replay.cpp'), f'-I{source / "libs"}',
         '-I/usr/include/eigen3', f'-L{build / "bin"}', f'-Wl,-rpath,{build / "bin"}',
         '-lcuvslam', '-o', str(folder / 'replay')],
    ]
    for name, command in zip(('configure', 'build', 'replay-build'), commands):
        print(f'SDK16 {name}: {folder}', flush=True)
        run(command, folder / f'candidate-{name}.log')
    if protected_baseline() != baseline:
        raise RuntimeError('Protected SDK15 baseline changed during candidate build')
    tracked = subprocess.check_output(['git', '-C', str(source), 'ls-files'], text=True).splitlines()
    files = {p: sha(source / p) for p in tracked if (source / p).is_file()}
    artifacts = [build / 'bin/libcuvslam.so', folder / 'replay', source / 'libs/cuvslam/cuvslam2.h']
    receipt = dict(schema=1, source_url=URL, source_tag='v16.0.0', source_commit=actual,
                   source_clean=True, scope='isolated native diagnostic build; no ROS or PX4 installation',
                   flight_authorized=False, protected_baseline_sha256=baseline, commands=commands,
                   source_tree_sha256=files,
                   candidate_profile=profile, candidate_profile_sha256=sha(profile_path),
                   artifacts_sha256={str(p.relative_to(ROOT)): sha(p) for p in artifacts},
                   replay_source_sha256=sha(ROOT / 'scripts/vio_sdk_covariance_replay.cpp'))
    (folder / 'build-receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
    print('Isolated candidate build complete; SDK15 baseline hashes unchanged')


if __name__ == '__main__':
    main()
