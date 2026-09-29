#!/usr/bin/env python3
"""Regression tests use disposable repositories, never workspace checkouts."""
import contextlib
import io
import os
from pathlib import Path
import tempfile
import unittest

from vendor_patches import git, inspect, process
from vendor_lfs import verify_file, verify_elf


class VendorGitTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.repo = self.root / 'repo'
        self.repo.mkdir()
        git(self.repo, 'init', '-q')
        git(self.repo, 'config', 'user.name', 'Fixture')
        git(self.repo, 'config', 'user.email', 'fixture@example.invalid')
        for name, data in [('source', 'original\n'), ('other', 'original\n'),
                           ('.gitignore', 'generated\n')]:
            (self.repo / name).write_text(data)
        git(self.repo, 'add', '.')
        git(self.repo, 'commit', '-qm', 'base')
        self.entry = dict(path='repo', patch='vendor.patch',
                          commit=git(self.repo, 'rev-parse', 'HEAD').decode().strip())
        (self.repo / 'source').write_text('patched\n')
        (self.repo / 'generated').write_text('manifest\n')
        (self.repo / 'empty').touch()
        git(self.repo, 'add', '-f', 'source', 'generated', 'empty')
        (self.root / 'vendor.patch').write_bytes(git(self.repo, 'diff', 'HEAD', '--binary', '--full-index'))
        git(self.repo, 'reset', '--hard', 'HEAD')

    def run_mode(self, mode):
        with contextlib.redirect_stdout(io.StringIO()):
            process(self.root, [self.entry], mode)

    def test_roundtrip_and_idempotence(self):
        self.assertEqual(inspect(self.root, self.entry)[0], 'base')
        self.run_mode('apply')
        self.run_mode('apply')
        self.run_mode('check')
        self.run_mode('reverse')
        self.run_mode('reverse')
        self.assertEqual(inspect(self.root, self.entry)[0], 'base')

    def test_missing_generated_files_recovered(self):
        self.run_mode('apply')
        (self.repo / 'generated').unlink()
        (self.repo / 'empty').unlink()
        with self.assertRaises(RuntimeError):
            self.run_mode('check')
        self.run_mode('apply')
        self.assertEqual((self.repo / 'generated').read_text(), 'manifest\n')
        self.assertTrue((self.repo / 'empty').is_file())

    def test_missing_files_can_be_reversed(self):
        self.run_mode('apply')
        (self.repo / 'generated').unlink()
        self.run_mode('reverse')
        self.assertEqual(inspect(self.root, self.entry)[0], 'base')

    def test_edited_generated_file_is_preserved(self):
        self.run_mode('apply')
        (self.repo / 'generated').write_text('user change')
        with self.assertRaises(RuntimeError):
            self.run_mode('apply')
        self.assertEqual((self.repo / 'generated').read_text(), 'user change')

    def test_untracked_and_staged_extras_rejected(self):
        self.run_mode('apply')
        extra = self.repo / 'extra'
        extra.touch()
        with self.assertRaises(RuntimeError):
            self.run_mode('check')
        extra.unlink()
        (self.repo / 'other').write_text('staged change\n')
        git(self.repo, 'add', 'other')
        with self.assertRaisesRegex(RuntimeError, 'staged'):
            self.run_mode('check')

    def test_unstaged_extra_rejected(self):
        self.run_mode('apply')
        (self.repo / 'other').write_text('user change\n')
        with self.assertRaises(RuntimeError):
            self.run_mode('reverse')

    def test_preflight_does_not_partially_apply(self):
        bad = dict(self.entry, path='absent')
        with self.assertRaises(RuntimeError):
            process(self.root, [self.entry, bad], 'apply')
        self.assertEqual((self.repo / 'source').read_text(), 'original\n')

    def test_parent_repository_is_not_accepted(self):
        (self.repo / 'child').mkdir()
        with self.assertRaisesRegex(RuntimeError, 'repository root'):
            inspect(self.root, dict(self.entry, path='repo/child'))

    def test_worktree_git_file_supported(self):
        linked = self.root / 'linked'
        git(self.repo, 'worktree', 'add', '--detach', str(linked), self.entry['commit'])
        self.assertTrue((linked / '.git').is_file())
        self.assertEqual(inspect(self.root, dict(self.entry, path='linked'))[0], 'base')

    def test_missing_lfs_and_invalid_elf_fail(self):
        with self.assertRaises(RuntimeError):
            verify_file(self.repo, {'name': 'missing', 'size': 12})
        with self.assertRaises(RuntimeError):
            verify_elf(self.repo / 'missing', 62)
        pointer = b'version https://git-lfs.github.com/spec/v1\n'
        (self.repo / 'pointer').write_bytes(pointer)
        with self.assertRaises(RuntimeError):
            verify_file(self.repo, {'name': 'pointer', 'size': len(pointer)})
        with self.assertRaises(RuntimeError):
            verify_elf(self.repo / 'pointer', 62)


if __name__ == '__main__':
    unittest.main()
