#!/usr/bin/env python3
"""Prepare and check LFS working files in the workspace and its submodules."""
import argparse
import json
import os
from pathlib import Path
import platform
import subprocess
import sys

from vendor_patches import ROOT, git


def repositories(repo):
    if not repo.is_dir() or Path(os.fsdecode(git(repo, 'rev-parse', '--show-toplevel')).strip()).resolve() != repo.resolve():
        raise RuntimeError(f'{repo}: initialize this repository before preparing LFS assets')
    yield repo
    modules = repo / '.gitmodules'
    if modules.exists():
        result = subprocess.run(['git', 'config', '--file', str(modules), '--get-regexp',
                                 r'^submodule\..*\.path$'], capture_output=True, text=True)
        if result.returncode not in (0, 1):
            raise RuntimeError(result.stderr)
        for line in result.stdout.splitlines():
            yield from repositories(repo / line.split(None, 1)[1])


def verify_file(repo, entry):
    path = repo / entry['name']
    if not path.is_file() or path.stat().st_size != entry['size']:
        raise RuntimeError(f'Missing or incorrectly sized LFS asset: {path}')
    with path.open('rb') as stream:
        if stream.read(128).startswith(b'version https://git-lfs.github.com/spec/v1\n'):
            raise RuntimeError(f'LFS pointer was not hydrated: {path}')


def verify_elf(path, machine):
    if not path.is_file():
        raise RuntimeError(f'Missing GXF library: {path}')
    with path.open('rb') as stream:
        header = stream.read(20)
    if len(header) != 20 or header[:5] != b'\x7fELF\x02' or header[5] not in (1, 2):
        raise RuntimeError(f'Expected a 64-bit ELF library: {path}')
    endian = 'little' if header[5] == 1 else 'big'
    if int.from_bytes(header[18:20], endian) != machine:
        raise RuntimeError(f'GXF library architecture mismatch: {path}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='Validate without downloading or changing Git configuration')
    args = parser.parse_args()
    git(ROOT, 'lfs', 'version')
    repos = list(repositories(ROOT))  # Validate all roots before any mutations.
    for repo in repos:
        entries = json.loads(git(repo, 'lfs', 'ls-files', '--json'))['files']
        if not entries:
            continue
        if not args.check:
            git(repo, 'lfs', 'install', '--local')
            git(repo, 'lfs', 'pull')
        for entry in entries:
            verify_file(repo, entry)
        print(f'{repo.relative_to(ROOT)}: {len(entries)} LFS assets verified', flush=True)
    architecture = platform.machine()
    variants = {'x86_64': ('gxf_x86_64_cuda_13_0', 62),
                'aarch64': ('gxf_aarch64_cuda_13_0', 183)}
    if architecture not in variants:
        raise RuntimeError(f'Unsupported GXF host architecture: {architecture}')
    variant, machine = variants[architecture]
    variant = os.environ.get('GXF_LFS_VARIANT', variant)
    if variant not in ('gxf_x86_64_cuda_13_0', 'gxf_aarch64_cuda_13_0', 'gxf_jetpack70'):
        raise RuntimeError(f'Unknown GXF_LFS_VARIANT: {variant}')
    verify_elf(ROOT / 'src/isaac_ros_nitros/isaac_ros_gxf/gxf/core/lib' /
               variant / 'core/libgxf_core.so', machine)


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, OSError, ValueError, KeyError) as error:
        print(f'LFS preparation failed: {error}', file=sys.stderr)
        sys.exit(1)
