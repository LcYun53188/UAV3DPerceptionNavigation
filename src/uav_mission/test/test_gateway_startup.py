"""Construct the actual gateway after asset admission, without spinning/control."""
import json
from pathlib import Path
import shutil
import uuid

import pytest
import rclpy
from rclpy.node import Node
from rclpy.context import Context

from uav_mission.flight_profiles import digest
from uav_mission.px4_flight import FlightServer


@pytest.fixture
def startup(tmp_path, monkeypatch):
    workspace = Path(__file__).resolve().parents[3]
    root = tmp_path
    regions = root/'simulation/safe_regions'
    regions.mkdir(parents=True)
    for name in ('vio_avoidance', 'obstacle_course', 'vio_chain'):
        shutil.copy2(workspace/f'simulation/safe_regions/{name}.json', regions/f'{name}.json')
    config = json.loads((regions/'obstacle_course.json').read_text())
    for name in config['scene_files']:
        source = workspace/'.deps/PX4-Autopilot/Tools/simulation/gz'/name
        target = root/'.deps/PX4-Autopilot/Tools/simulation/gz'/name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
    out = root/'.cache/simulation/vio-sensors/startup'
    files = {}
    for name in ('assets/default.sdf', 'assets/x500_vio_ref/model.sdf', 'assets/frames.json',
                 'assets/warehouse-layout.json', 'calibration.json', 'anchor.json'):
        path = out/name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(name)
        files[name] = digest(path)
    env = dict(UAV_WORKSPACE=str(root), UAV_SITL_AUTHORIZATION='test-owned', ROS_DOMAIN_ID='78',
        UAV_FLIGHT_COORDINATOR_INSTANCE=str(uuid.uuid4()), GZ_PARTITION='uav_warehouse_flight_test',
        UAV_REQUIRE_VIO='1', UAV_LOCALIZATION_PROFILE='DEFAULT',
        UAV_VIO_FUSION_PROFILE='aligned_pose_mtf_primary_sim_v1', UAV_VIO_CALIBRATION_ID='a'*64,
        UAV_EXPERIMENTAL_VIO_CHAIN='1', UAV_ENABLE_FUSION_SWITCH='0', UAV_ENABLE_EGO_NAV='1',
        UAV_FLIGHT_ASSET_RECEIPT=str(out/'flight-assets.json'))
    # Domain 78 is required by asset admission; isolate the actual ROS context
    # on domain 178 so this constructor test cannot join the running simulator.
    for key, value in env.items(): monkeypatch.setenv(key, value)
    nodes = []
    context = Context()
    original_init = Node.__init__
    def track_node(self, *args, **kwargs):
        original_init(self, *args, context=context, **kwargs)
        nodes.append(self)  # Also clean up partially constructed rejected nodes.
    monkeypatch.setattr(Node, '__init__', track_node)
    rclpy.init(context=context, domain_id=178)

    def configure(profile, fault=None):
        region = regions/f'{profile}.json'
        if fault == 'profile_mismatch':
            data = json.loads(region.read_text())
            data['profile'] = 'experimental_sdk16_static_avoidance'
            region.write_text(json.dumps(data))
        receipt = dict(schema=1, partition=env['GZ_PARTITION'], calibration_id=env['UAV_VIO_CALIBRATION_ID'],
                       region_sha256=digest(region), files=files)
        path = out/'flight-assets.json'
        path.write_text(json.dumps(receipt))
        monkeypatch.setenv('UAV_FLIGHT_ASSET_SHA256', digest(path))
        monkeypatch.setenv('UAV_FLIGHT_REGION_PROFILE', profile)
        if fault == 'fusion_switch': monkeypatch.setenv('UAV_ENABLE_FUSION_SWITCH', '1')
        elif fault == 'no_deferral': monkeypatch.setenv('UAV_EXPERIMENTAL_VIO_CHAIN', '0')
        elif fault == 'wrong_domain': monkeypatch.setenv('ROS_DOMAIN_ID', '79')
        elif fault == 'asset_drift': (out/'assets/default.sdf').write_text('drift')
        return FlightServer()
    try:
        yield configure
    finally:
        for node in reversed(nodes): node.destroy_node()
        context.shutdown()


@pytest.mark.parametrize('profile', ['vio_avoidance', 'obstacle_course'])
def test_owned_primary_gateway_constructs_without_arming_or_dispatch(startup, profile):
    node = startup(profile)
    assert node.vio_gate.primary_mtf
    assert {'flow_input', 'range_input'}.issubset(node.vio_topics)
    assert node.planned is not None
    assert node.phase == 'IDLE' and node.owner == 'NONE'
    assert not node.active_goal and not node.commands and node.output_count == 0
    assert not node.operator_arm_enabled and not node.operator_navigation_enabled
    assert node.config['body_radius_m'] == .5 and node.config['braking_margin_m'] == 1.2


@pytest.mark.parametrize('fault', ['profile_mismatch', 'fusion_switch', 'no_deferral', 'wrong_domain', 'asset_drift'])
def test_primary_course_startup_keeps_owned_profile_and_asset_guards(startup, fault):
    with pytest.raises(RuntimeError): startup('obstacle_course', fault)


def test_primary_fusion_is_still_rejected_for_other_vio_scenes(startup):
    with pytest.raises(RuntimeError): startup('vio_chain')
