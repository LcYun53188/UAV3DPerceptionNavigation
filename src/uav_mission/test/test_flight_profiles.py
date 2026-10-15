"""Scene authorization mismatches must fail before controls are constructed."""
import json
from pathlib import Path
import pytest
from uav_mission.flight_profiles import digest,load_region


@pytest.fixture
def owned(tmp_path):
    root=tmp_path;regions=root/'simulation/safe_regions';regions.mkdir(parents=True)
    upstream=root/'.deps/PX4-Autopilot/Tools/simulation/gz';upstream.mkdir(parents=True)
    (upstream/'scene.sdf').write_text('pinned upstream')
    for profile in ('W0','warehouse'):
        (regions/f'{profile}.json').write_text(json.dumps(dict(scene=profile,scene_files={'scene.sdf':digest(upstream/'scene.sdf')})))
    out=root/'.cache/simulation/vio-sensors/run';out.mkdir(parents=True)
    files={}
    for name in ('assets/default.sdf','assets/x500_vio_ref/model.sdf','assets/frames.json','assets/warehouse-layout.json','calibration.json','anchor.json'):
        p=out/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(name);files[name]=digest(p)
    env=dict(UAV_SITL_AUTHORIZATION='test-owned',ROS_DOMAIN_ID='78',GZ_PARTITION='uav_warehouse_flight_run',
        UAV_FLIGHT_REGION_PROFILE='warehouse',UAV_REQUIRE_VIO='1',UAV_VIO_FUSION_PROFILE='aligned_pose_v1',
        UAV_VIO_CALIBRATION_ID='a'*64,UAV_FLIGHT_ASSET_RECEIPT=str(out/'flight-assets.json'))
    data=dict(schema=1,partition=env['GZ_PARTITION'],calibration_id=env['UAV_VIO_CALIBRATION_ID'],
        region_sha256=digest(regions/'warehouse.json'),files=files)
    def write():
        Path(env['UAV_FLIGHT_ASSET_RECEIPT']).write_text(json.dumps(data))
        env['UAV_FLIGHT_ASSET_SHA256']=digest(Path(env['UAV_FLIGHT_ASSET_RECEIPT']))
    write()
    return root,env,out,data,write


def test_default_w0_keeps_pinned_scene_admission(owned):
    root,env,_,_,_=owned
    env.pop('UAV_FLIGHT_REGION_PROFILE');env['GZ_PARTITION']='uav_px4_s0_run'
    assert load_region(root,env)['scene']=='W0'
    (root/'.deps/PX4-Autopilot/Tools/simulation/gz/scene.sdf').write_text('drift')
    with pytest.raises(RuntimeError):load_region(root,env)


def test_owned_warehouse_receipt_passes(owned):
    root,env,_,_,_=owned
    assert load_region(root,env)['scene']=='warehouse'


@pytest.fixture
def flow_owned(tmp_path):
    import shutil,sys
    workspace=Path(__file__).resolve().parents[3]
    sys.path.insert(0,str(workspace/'scripts'))
    from prepare_mtf01_assets import assets
    regions=tmp_path/'simulation/safe_regions';regions.mkdir(parents=True)
    profile=workspace/'simulation/safe_regions/flow_local.json'
    shutil.copy2(profile,regions/profile.name)
    upstream=workspace/'.deps/PX4-Autopilot/Tools/simulation/gz'
    config=json.loads(profile.read_text())
    for name in config['scene_files']:
        target=tmp_path/'.deps/PX4-Autopilot/Tools/simulation/gz'/name
        target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(upstream/name,target)
    out=tmp_path/'.cache/simulation/mtf01/owned';out.mkdir(parents=True)
    assets(out/'assets',upstream/'worlds/default.sdf')
    data=dict(schema=1,partition='uav_mtf01_owned',region_sha256=digest(regions/profile.name),
        files={str(p.relative_to(out/'assets')):digest(p) for p in (out/'assets').rglob('*') if p.is_file()})
    receipt=out/'flight-assets.json';receipt.write_text(json.dumps(data))
    env=dict(UAV_SITL_AUTHORIZATION='owned',ROS_DOMAIN_ID='78',GZ_PARTITION=data['partition'],
        UAV_FLIGHT_REGION_PROFILE='flow_local',UAV_REQUIRE_VIO='0',UAV_LOCALIZATION_PROFILE='FLOW_LOCAL',
        UAV_FLIGHT_ASSET_RECEIPT=str(receipt),UAV_FLIGHT_ASSET_SHA256=digest(receipt))
    return tmp_path,env,out


def test_standalone_flow_boot_assets_pass_without_vio_calibration(flow_owned):
    root,env,_=flow_owned
    assert load_region(root,env)['profile']=='standalone_flow_local'


@pytest.mark.parametrize('fault',['asset','VIO','EGO','runtime_switch','partition'])
def test_standalone_flow_boot_rejects_mixed_ownership_and_asset_drift(flow_owned,fault):
    root,env,out=flow_owned
    if fault=='asset':(out/'assets/default.sdf').write_text('changed')
    elif fault=='VIO':env['UAV_REQUIRE_VIO']='1'
    elif fault=='EGO':env['UAV_ENABLE_EGO_NAV']='1'
    elif fault=='runtime_switch':env['UAV_ENABLE_FUSION_SWITCH']='1'
    else:env['GZ_PARTITION']='uav_mtf01_other'
    with pytest.raises(RuntimeError):load_region(root,env)


@pytest.mark.parametrize('key,value',[('UAV_SITL_AUTHORIZATION',''),('ROS_DOMAIN_ID','0'),
    ('GZ_PARTITION','uav_px4_s0_run'),('UAV_FLIGHT_REGION_PROFILE','arbitrary'),
    ('UAV_REQUIRE_VIO','0'),('UAV_VIO_FUSION_PROFILE','full_odometry'),
    ('UAV_VIO_CALIBRATION_ID','b'*64),('UAV_FLIGHT_ASSET_SHA256','old')])
def test_changed_ownership_or_calibration_fails_closed(owned,key,value):
    root,env,_,_,_=owned;env[key]=value
    with pytest.raises(RuntimeError):load_region(root,env)


@pytest.mark.parametrize('fault',['asset','region','missing','escape','partition'])
def test_scene_change_or_incomplete_receipt_is_rejected(owned,fault):
    root,env,out,data,write=owned
    if fault=='asset':(out/'assets/default.sdf').write_text('different world')
    if fault=='region':(root/'simulation/safe_regions/warehouse.json').write_text(json.dumps(dict(scene_files={})))
    if fault=='missing':data['files'].pop('assets/frames.json');write()
    if fault=='escape':
        (root/'outside').write_text('outside');data['files']['../../../../outside']=digest(root/'outside');write()
    if fault=='partition':data['partition']='old';write()
    with pytest.raises(RuntimeError):load_region(root,env)


@pytest.fixture
def depth_owned(tmp_path):
    import shutil
    import sys
    from pathlib import Path
    workspace=Path(__file__).resolve().parents[3]
    sys.path.insert(0,str(workspace/'scripts'))
    from prepare_px4_depth_assets import depth_assets
    regions=tmp_path/'simulation/safe_regions';regions.mkdir(parents=True)
    profile=workspace/'simulation/safe_regions/depth_reference.json'
    shutil.copy2(profile,regions/profile.name)
    upstream=workspace/'.deps/PX4-Autopilot/Tools/simulation/gz'
    config=json.loads(profile.read_text())
    for name in config['scene_files']:
        target=tmp_path/'.deps/PX4-Autopilot/Tools/simulation/gz'/name
        target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(upstream/name,target)
    out=tmp_path/'.cache/simulation/sitl/owned';out.mkdir(parents=True)
    depth_assets(out/'assets',upstream/'worlds/default.sdf')
    env=dict(UAV_SITL_AUTHORIZATION='owned',ROS_DOMAIN_ID='78',GZ_PARTITION='uav_px4_s0_owned',
             UAV_FLIGHT_REGION_PROFILE='depth_reference',UAV_REQUIRE_VIO='0',
             UAV_FLIGHT_COORDINATOR_INSTANCE='owned-session',UAV_FLIGHT_ASSET_RECEIPT=str(out/'receipt.json'))
    data=dict(schema=1,partition=env['GZ_PARTITION'],region_sha256=digest(regions/profile.name),
              localization_session='owned-session',files={str(p.relative_to(out)):digest(p) for p in (out/'assets').rglob('*') if p.is_file()})
    def write():
        (out/'receipt.json').write_text(json.dumps(data))
        env['UAV_FLIGHT_ASSET_SHA256']=digest(out/'receipt.json')
    write()
    return tmp_path,env,out,data,write


def test_generated_depth_flight_assets_match_frozen_profile(depth_owned):
    root,env,_,_,_=depth_owned
    region=load_region(root,env)
    assert region['observation_enabled'] and region['map_translation']==[0.,0.,0.]


@pytest.mark.parametrize('fault',['session','partition','rehashed_asset','escape','vio','missing','receipt_hash'])
def test_depth_asset_admission_fails_closed(depth_owned,fault):
    root,env,out,data,write=depth_owned
    if fault=='session':env['UAV_FLIGHT_COORDINATOR_INSTANCE']='other'
    elif fault=='partition':data['partition']='other';write()
    elif fault=='rehashed_asset':
        file=out/'assets/frames.json';file.write_text('{}')
        data['files']['assets/frames.json']=digest(file);write()
    elif fault=='escape':data['files']['../foreign']='bad';write()
    elif fault=='vio':env['UAV_REQUIRE_VIO']='1'
    elif fault=='missing':data['files'].pop('assets/frames.json');write()
    elif fault=='receipt_hash':env['UAV_FLIGHT_ASSET_SHA256']='bad'
    with pytest.raises(RuntimeError):load_region(root,env)


def test_execution_limits_are_shared_and_cannot_exceed_known_region_limits():
    from uav_mission.flight_profiles import navigation_limits
    config=dict(max_speed_mps=.6,max_acceleration_mps2=.5,max_jerk_mps3=.6)
    assert navigation_limits(config)==(.6,.5,.6)
    config['ego_limits']=[.18,.15,.2]
    assert navigation_limits(config)==(.18,.15,.2)
    for bad in ([.7,.15,.2],[.18,-1,.2],[True,.15,.2],[.18,float('nan'),.2],[.18,.15]):
        config['ego_limits']=bad
        with pytest.raises(ValueError,match='INVALID_EGO_LIMITS'):navigation_limits(config)


def test_experimental_joint_profile_needs_explicit_deferral_and_ownership(owned):
    root,env,out,data,write=owned
    region=root/'simulation/safe_regions/vio_chain.json'
    region.write_bytes((root/'simulation/safe_regions/warehouse.json').read_bytes())
    env.update(UAV_FLIGHT_REGION_PROFILE='vio_chain',UAV_VIO_FUSION_PROFILE='aligned_pose_mtf_sim_v1')
    data['region_sha256']=digest(region);write()
    with pytest.raises(RuntimeError,match='deferral'):load_region(root,env)
    env['UAV_EXPERIMENTAL_VIO_CHAIN']='1'
    assert load_region(root,env)
    env['ROS_DOMAIN_ID']='0'
    with pytest.raises(RuntimeError):load_region(root,env)


@pytest.mark.parametrize('profile',['warehouse','vio_chain','vio_avoidance'])
def test_vio_primary_is_admitted_only_in_owned_avoidance_scene(owned,profile):
    root,env,out,data,write=owned
    region=root/f'simulation/safe_regions/{profile}.json'
    if profile!='warehouse':
        region.write_bytes((root/'simulation/safe_regions/warehouse.json').read_bytes())
    env.update(UAV_FLIGHT_REGION_PROFILE=profile,
               UAV_VIO_FUSION_PROFILE='aligned_pose_mtf_primary_sim_v1',
               UAV_EXPERIMENTAL_VIO_CHAIN='1')
    data['region_sha256']=digest(region);write()
    if profile=='vio_avoidance':
        assert load_region(root,env)
        env['UAV_EXPERIMENTAL_VIO_CHAIN']='0'
        with pytest.raises(RuntimeError,match='deferral'):load_region(root,env)
    else:
        with pytest.raises(RuntimeError,match='aligned pose'):load_region(root,env)
