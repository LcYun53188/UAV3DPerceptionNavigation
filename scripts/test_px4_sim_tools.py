"""Regression for rejecting incompatible DDS interfaces and wrong source pins."""
import subprocess
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from check_px4_interfaces import compare
from prepare_px4_sim import ensure_source


def fixtures(tmp_path):
    px4, ros = tmp_path / 'px4', tmp_path / 'ros'
    (px4 / 'msg/versioned').mkdir(parents=True)
    (ros / 'msg').mkdir(parents=True)
    (px4 / 'src/modules/uxrce_dds_client').mkdir(parents=True)
    (px4 / 'src/modules/uxrce_dds_client/dds_topics.yaml').write_text(
        'publications:\n- topic: /fmu/out/test\n  type: px4_msgs::msg::Sample\nsubscriptions_multi:\n')
    (px4 / 'msg/versioned/Sample.msg').write_text('uint64 timestamp\nChild[2] items\n')
    (px4 / 'msg/versioned/Child.msg').write_text('float32 x\nfloat32 y\n')
    (ros / 'msg/Sample.msg').write_text('# comment\nuint64 timestamp # clock\nChild[2] items\n')
    (ros / 'msg/Child.msg').write_text('float32 x\nfloat32 y\n')
    return px4, ros


def test_comments_do_not_hide_missing_nested_definition(tmp_path):
    px4, ros = fixtures(tmp_path)
    assert compare(px4, ros)['status'] == 'PASS'
    (ros / 'msg/Child.msg').unlink()
    assert compare(px4, ros)['status'] == 'FAIL'


def test_field_reordering_is_incompatible(tmp_path):
    px4, ros = fixtures(tmp_path)
    (ros / 'msg/Child.msg').write_text('float32 y\nfloat32 x\n')
    assert compare(px4, ros)['status'] == 'FAIL'


def test_wrong_pin_never_switches_existing_checkout(tmp_path):
    repo = tmp_path / 'checkout'
    subprocess.run(['git', 'init', '-q', str(repo)], check=True)
    subprocess.run(['git', '-C', str(repo), '-c', 'user.name=Test', '-c', 'user.email=test@example.invalid',
                    'commit', '--allow-empty', '-qm', 'local work'], check=True)
    before = subprocess.check_output(['git', '-C', str(repo), 'rev-parse', 'HEAD'])
    with pytest.raises(RuntimeError, match='differs from lock'):
        ensure_source(dict(path=str(repo), commit='0' * 40))
    assert subprocess.check_output(['git', '-C', str(repo), 'rev-parse', 'HEAD']) == before
