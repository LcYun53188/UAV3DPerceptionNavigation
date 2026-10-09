#!/usr/bin/env python3
"""Verify archived bytes and independently recorded sensor/map outcomes."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parent
for line in (root/'SHA256SUMS').read_text().splitlines():
    digest, name = line.split('  ', 1)
    assert hashlib.sha256((root/name).read_bytes()).hexdigest() == digest, name
for name in ('mapping-pass', 'final-pass'):
    folder = root/name
    result = json.loads((folder/'observation.json').read_text())
    depth = json.loads((folder/'depth-camera.json').read_text())
    grid = json.loads((folder/'depth-map.json').read_text())
    manifest = json.loads((folder/'manifest.json').read_text())
    assert result['passed'] and result['depth_mapping_passed'] and result['depth_camera_passed']
    assert result['arming_states'] == [1] and result['landed']
    assert depth['passed'] and not depth['errors'] and depth['regressions'] == 0
    assert depth['valid_images'] >= 5 and depth['writers'] == {'/px4_depth/image':1,'/px4_depth/camera_info':1}
    assert grid['passed'] and grid['disarmed'] and grid['valid'] and grid['reason'] == 'READY'
    assert 0 < grid['observed_positive_distance_voxels'] <= grid['observed_voxels'] < grid['voxels']
    assert 0 <= grid['source_age_s'] <= 2.
    assert manifest['depth_reference_profile']['camera_pitch_deg'] == 5
    for name,digest in manifest['depth_reference_assets_sha256'].items():
        assert hashlib.sha256((folder/name).read_bytes()).hexdigest() == digest
    model=ET.parse(folder/'assets/x500_depth_ref/model.sdf').find('model')
    assert model.findtext('include/uri') == 'model://x500'
    assert len(model.findall('.//sensor')) == 1
failed=json.loads((root/'initialization-reset-fail/depth-map.json').read_text())
assert not failed['passed'] and not failed['valid'] and not failed['disarmed']
print('PASS: archive bytes and recorded disarmed depth/map outcomes; no flight qualification')
