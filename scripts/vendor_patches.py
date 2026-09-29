#!/usr/bin/env python3
"""Validate vendor trees against pinned commits using disposable Git indexes."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def git(repo, *args, env=None):
    result = subprocess.run(['git', '-C', str(repo), *args], env=env,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if result.returncode:
        raise RuntimeError(result.stderr.decode(errors='replace').strip())
    return result.stdout


def paths(data):
    return [os.fsdecode(p) for p in data.split(b'\0') if p]


def inspect(root, entry):
    repo = root / entry['path']
    if not repo.is_dir() or Path(os.fsdecode(git(repo, 'rev-parse', '--show-toplevel')).strip()).resolve() != repo.resolve():
        raise RuntimeError(f'{repo}: missing repository or incorrect repository root')
    if git(repo, 'rev-parse', 'HEAD').decode().strip() != entry['commit']:
        raise RuntimeError(f'{repo}: HEAD differs from pinned commit; preserve local work before restoring it')
    if git(repo, 'diff', '--cached', '--name-only', '-z'):
        raise RuntimeError(f'{repo}: staged changes; unstage them before vendor operations')
    # A parent patch must never hide a changed or uninitialized nested gitlink.
    for line in git(repo, 'submodule', 'status', '--recursive').decode().splitlines():
        if line and line[0] != ' ':
            raise RuntimeError(f'{repo}: nested submodule mismatch: {line}')
    with tempfile.TemporaryDirectory(prefix='vendor-index-') as tmp:
        env = dict(os.environ, GIT_INDEX_FILE=str(Path(tmp) / 'index'))
        git(repo, 'read-tree', entry['commit'], env=env)
        base_diff = paths(git(repo, 'diff', '--name-only', '-z', '--ignore-submodules=all', env=env))
        base_entries = set(paths(git(repo, 'ls-files', '-z', env=env)))
        git(repo, 'apply', '--cached', '--whitespace=nowarn', str(root / entry['patch']), env=env)
        expected_entries = set(paths(git(repo, 'ls-files', '-z', env=env)))
        added = expected_entries - base_entries
        extras = paths(git(repo, 'ls-files', '--others', '--exclude-standard', '-z', env=env))
        if extras:
            raise RuntimeError(f'{repo}: unrecorded files: {extras}')
        changed = set(paths(git(repo, 'diff', '--name-only', '-z', '--ignore-submodules=all', env=env)))
        if not changed:
            return 'applied', []
        missing = {p for p in added if not os.path.lexists(repo / p)}
        if not base_diff and all(not os.path.lexists(repo / p) for p in added):
            return 'base', []
        if changed == missing:
            restore = []
            for p in sorted(missing):
                mode = git(repo, 'ls-files', '--stage', '--', p, env=env).split()[0]
                if mode not in (b'100644', b'100755'):
                    raise RuntimeError(f'{repo}: unsupported generated file mode for {p}')
                restore.append((p, git(repo, 'show', ':' + p, env=env), int(mode, 8) & 0o777))
            return 'missing', restore
        raise RuntimeError(f'{repo}: changes outside expected patch: {sorted(changed)}')


def validate_manifest(root, entries):
    by_path = {e['path']: e for e in entries}
    manifest = json.loads((root / 'src/uav_bringup/config/algorithm_versions.json').read_text())
    for dep in manifest['dependencies']:
        source = by_path[dep['path']]
        if any(dep[key] != source[key] for key in ('commit', 'patch')):
            raise RuntimeError(f"Manifest disagrees with vendor source: {dep['path']}")
        if hashlib.sha256((root / dep['patch']).read_bytes()).hexdigest() != dep['patch_sha256']:
            raise RuntimeError(f"Patch hash mismatch: {dep['patch']}")


def process(root, entries, mode):
    # Complete preflight before changing any repository.
    states = [(entry, inspect(root, entry)) for entry in entries]
    failures = False
    for entry, (state, restore) in states:
        print(f"{entry['path']}: {state}", flush=True)
        if mode == 'check':
            failures |= state != 'applied'
            continue
        if inspect(root, entry)[0] != state:
            raise RuntimeError('Repository changed during preflight; retry after other Git operations finish')
        repo = root / entry['path']
        if state == 'missing':
            for name, data, permissions in restore:
                destination = repo / name
                destination.parent.mkdir(parents=True, exist_ok=True)
                with destination.open('xb') as stream:
                    stream.write(data)
                destination.chmod(permissions)
            state = 'applied'
        if mode == 'apply' and state == 'base':
            git(repo, 'apply', '--whitespace=nowarn', str(root / entry['patch']))
        elif mode == 'reverse' and state == 'applied':
            git(repo, 'apply', '--reverse', '--whitespace=nowarn', str(root / entry['patch']))
        expected = 'applied' if mode == 'apply' else 'base'
        if inspect(root, entry)[0] != expected:
            raise RuntimeError(f'{repo}: post-operation verification failed')
    if failures:
        raise RuntimeError('Vendor trees are not fully applied; run the preparation script')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    for mode in ('apply', 'reverse', 'check'):
        modes.add_argument('--' + mode, dest='mode', action='store_const', const=mode)
    parser.set_defaults(mode='apply')
    parser.add_argument('--ego', action='store_true', help='Operate on the prepared EGO checkout only')
    args = parser.parse_args()
    entries = json.loads((ROOT / 'patches/vendor/sources.json').read_text())['dependencies']
    validate_manifest(ROOT, entries)
    if not args.ego:
        for line in git(ROOT, 'submodule', 'status', '--recursive').decode().splitlines():
            if line and line[0] != ' ':
                raise RuntimeError(f'Submodule does not match its recorded gitlink: {line}')
    selected = [e for e in entries if e['path'].startswith('.deps/') == args.ego]
    process(ROOT, selected, args.mode)


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, OSError, ValueError, KeyError) as error:
        print(f'Vendor check failed: {error}', file=sys.stderr)
        sys.exit(1)
