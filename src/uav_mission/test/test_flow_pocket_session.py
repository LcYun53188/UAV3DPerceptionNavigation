"""A Pocket source change must preserve isolation and never imply auto-flight."""
import hashlib
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'scripts'))
from pocket_operator_session import operator_environment
from run_px4_mtf01 import main


def flow_manifest(session):
    receipt = session / 'flight-assets.json'
    receipt.write_text('{}')
    env = dict(UAV_REQUIRE_VIO='0', UAV_LOCALIZATION_PROFILE='FLOW_LOCAL',
        UAV_ENABLE_FUSION_SWITCH='0', UAV_FLIGHT_REGION_PROFILE='flow_local',
        UAV_ENABLE_EGO_NAV='0', UAV_NAVIGATION_BACKEND='DIRECT',
        UAV_FLIGHT_ASSET_RECEIPT=str(receipt), UAV_FLIGHT_EVIDENCE=str(session),
        UAV_FLIGHT_MODEL='x500_mtf01_7', UAV_FLIGHT_ASSET_SHA256=hashlib.sha256(receipt.read_bytes()).hexdigest())
    return dict(localization_profile='FLOW_LOCAL', operator_environment=env)


def test_flow_owner_inherits_exact_no_gnss_no_vio_asset_context(tmp_path):
    manifest = flow_manifest(tmp_path)
    env = operator_environment(tmp_path, manifest)
    assert env == manifest['operator_environment']
    assert env['UAV_REQUIRE_VIO'] == '0' and env['UAV_ENABLE_EGO_NAV'] == '0'


@pytest.mark.parametrize('key,value', [('UAV_REQUIRE_VIO', '1'),
    ('UAV_FLIGHT_REGION_PROFILE', 'W0'), ('UAV_FLIGHT_EVIDENCE', '/tmp/other'),
    ('UAV_FLIGHT_MODEL', 'x500_7'), ('UAV_FLIGHT_ASSET_SHA256', 'stale'),
    ('UAV_ENABLE_FUSION_SWITCH', '1'), ('UAV_ENABLE_EGO_NAV', '1')])
def test_mixed_source_or_receipt_identity_cannot_construct_operator(tmp_path, key, value):
    manifest = flow_manifest(tmp_path)
    manifest['operator_environment'][key] = value
    with pytest.raises(ValueError):
        operator_environment(tmp_path, manifest)


def test_rewritten_asset_receipt_rejected(tmp_path):
    manifest = flow_manifest(tmp_path)
    (tmp_path / 'flight-assets.json').write_text('{"changed":true}')
    with pytest.raises(ValueError, match='receipt changed'):
        operator_environment(tmp_path, manifest)


@pytest.mark.parametrize('flags', [[], ['--flight'], ['--ui', '--flight'],
    ['--ui', '--native-manual'], ['--ui', '--stop-sensors'], ['--ui', '--no-floor-texture']])
def test_persistent_pocket_rejects_automatic_flight_or_missing_observability(monkeypatch, flags):
    monkeypatch.setattr(sys, 'argv', ['run_px4_mtf01.py', '--usb-joystick', *flags])
    with pytest.raises(SystemExit) as error:
        main()
    assert error.value.code == 2
