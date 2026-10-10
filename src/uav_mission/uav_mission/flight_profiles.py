"""Owned local SITL scene admission; default W0 remains the pinned baseline."""
import hashlib
import json
import math
from pathlib import Path


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def load_region(root,env):
    root=Path(root).resolve()
    if not env.get('UAV_SITL_AUTHORIZATION') or env.get('ROS_DOMAIN_ID')!='78':
        raise RuntimeError('Requires owned local SITL supervisor, domain 78 and nonce')
    partition=env.get('GZ_PARTITION','')
    profile=env.get('UAV_FLIGHT_REGION_PROFILE','W0')
    if profile not in ('W0','warehouse','depth_reference') or not partition.startswith(
            'uav_px4_s0_' if profile in ('W0','depth_reference') else 'uav_warehouse_flight_'):
        raise RuntimeError('Flight scene/partition mismatch')
    config=json.loads((root/f'simulation/safe_regions/{profile}.json').read_text())
    for name,expected in config['scene_files'].items():
        if digest(root/'.deps/PX4-Autopilot/Tools/simulation/gz'/name)!=expected:
            raise RuntimeError('Pinned scene/model hash mismatch: '+name)
    if profile=='depth_reference':
        if env.get('UAV_REQUIRE_VIO')=='1':raise RuntimeError('Depth observation requires official GNSS/inertial EKF')
        receipt=Path(env.get('UAV_FLIGHT_ASSET_RECEIPT','')).resolve()
        if not receipt.is_relative_to(root/'.cache/simulation/sitl'):
            raise RuntimeError('Depth receipt outside owned SITL cache')
        if digest(receipt)!=env.get('UAV_FLIGHT_ASSET_SHA256'):
            raise RuntimeError('Depth asset receipt hash mismatch')
        data=json.loads(receipt.read_text())
        if (data.get('schema')!=1 or data.get('partition')!=partition or
                data.get('region_sha256')!=digest(root/'simulation/safe_regions/depth_reference.json') or
                data.get('localization_session')!=env.get('UAV_FLIGHT_COORDINATOR_INSTANCE')):
            raise RuntimeError('Depth region/partition/session mismatch')
        if data.get('files')!=config['asset_files']:raise RuntimeError('Unreviewed depth assets')
        for name,expected in data['files'].items():
            path=(receipt.parent/name).resolve()
            if not path.is_relative_to(receipt.parent) or digest(path)!=expected:
                raise RuntimeError('Depth asset drift: '+name)
    if profile=='warehouse':
        if env.get('UAV_REQUIRE_VIO')!='1' or env.get('UAV_VIO_FUSION_PROFILE')!='aligned_pose_v1':
            raise RuntimeError('Warehouse requires actual aligned pose VIO fusion')
        receipt=Path(env.get('UAV_FLIGHT_ASSET_RECEIPT','')).resolve()
        if not receipt.is_relative_to(root/'.cache/simulation/vio-sensors'):
            raise RuntimeError('Warehouse receipt outside owned simulation cache')
        if digest(receipt)!=env.get('UAV_FLIGHT_ASSET_SHA256'):
            raise RuntimeError('Warehouse asset receipt hash mismatch')
        data=json.loads(receipt.read_text())
        if (data.get('schema')!=1 or data.get('region_sha256')!=digest(root/'simulation/safe_regions/warehouse.json')
                or data.get('calibration_id')!=env.get('UAV_VIO_CALIBRATION_ID')
                or data.get('partition')!=partition):
            raise RuntimeError('Warehouse region/calibration/partition mismatch')
        required={'assets/default.sdf','assets/x500_vio_ref/model.sdf','assets/frames.json','assets/warehouse-layout.json','calibration.json','anchor.json'}
        files=data.get('files',{})
        if not required.issubset(files):raise RuntimeError('Incomplete warehouse asset receipt')
        for name,expected in files.items():
            path=(receipt.parent/name).resolve()
            if not path.is_relative_to(receipt.parent) or digest(path)!=expected:
                raise RuntimeError('Warehouse asset drift: '+name)
    return config


def navigation_limits(config):
    upper=tuple(config[k] for k in ('max_speed_mps','max_acceleration_mps2','max_jerk_mps3'))
    values=config.get('ego_limits',upper)
    if (not isinstance(values,(list,tuple)) or len(values)!=3 or
            any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or
                not 0<v<=bound for v,bound in zip(values,upper))):
        raise ValueError('INVALID_EGO_LIMITS')
    return tuple(map(float,values))
