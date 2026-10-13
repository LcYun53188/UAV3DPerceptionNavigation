#!/usr/bin/env python3
"""Generate OpenVINS calibration from physical mount TF and camera intrinsics."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import yaml
from vio_sensor_transforms import quaternion_from_rpy

ROOT=Path(__file__).resolve().parents[1]
COMMIT='69488123ed9362dd44b6f28e7f4680abbff1442b'
class OpenCvDumper(yaml.SafeDumper):
    def ignore_aliases(self,data):return True
    def increase_indent(self,flow=False,indentless=False):
        return super().increase_indent(flow,False)

CV_FROM_FLU=np.array([[0.,-1.,0.],[0.,0.,-1.],[1.,0.,0.]])

def mount(frame):
    x,y,z,w=quaternion_from_rpy(frame['rpy'])
    r=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    t=np.eye(4);t[:3,:3]=r;t[:3,3]=frame['position'];return t

def generate(calibration,intrinsics,output,*,imu_axes='flu'):
    output.mkdir(parents=True,exist_ok=True)
    base=ROOT/'.cache/vio-openvins/src/config/rpng_sim'
    config=yaml.safe_load('\n'.join((base/'estimator_config.yaml').read_text().splitlines()[1:]))
    config.update(verbosity='WARNING',max_cameras=2,use_stereo=True,use_fej=True,
        calib_cam_extrinsics=False,calib_cam_intrinsics=False,calib_cam_timeoffset=False,
        calib_imu_intrinsics=False,calib_imu_g_sensitivity=False,init_dyn_use=False,gravity_mag=9.8,
        try_zupt=True,zupt_only_at_beginning=True,zupt_max_velocity=.1,
        track_frequency=25.,num_opencv_threads=2,init_max_features=100,grid_x=5,grid_y=4,init_max_disparity=1.5,record_timing_information=False,save_total_state=False)
    config.update(relative_config_imu='imu.yaml',relative_config_imucam='cameras.yaml')
    imu=yaml.safe_load('\n'.join((base/'kalibr_imu_chain.yaml').read_text().splitlines()[1:]))
    params=calibration['parameters'];noise=calibration.get('profile',{}).get('imu_noise_model',params);sensor=imu['imu0']
    sensor.update(gyroscope_noise_density=noise['gyro_noise_density'],
        gyroscope_random_walk=noise['gyro_random_walk'],accelerometer_noise_density=noise['accel_noise_density'],
        accelerometer_random_walk=noise['accel_random_walk'],update_rate=params['calibration_frequency'],rostopic='/vio/imu')
    tf=calibration['frames'];base_from_imu=mount(tf['vio_imu'])
    if imu_axes=='opencv':base_from_imu[:3,:3]=base_from_imu[:3,:3]@CV_FROM_FLU.T
    elif imu_axes!='flu':raise ValueError('Explicit IMU axes required')
    imu_from_base=np.linalg.inv(base_from_imu);cameras={}
    for i,name in enumerate(('vio_left_optical','vio_right_optical')):
        cam=intrinsics[i];cameras['cam'+str(i)]=dict(T_imu_cam=(imu_from_base@mount(tf[name])).tolist(),
            cam_overlaps=[1-i],camera_model='pinhole',distortion_model='radtan',distortion_coeffs=[0.,0.,0.,0.],
            intrinsics=[*cam['focal'],*cam['principal']],resolution=cam['size'],rostopic='/vio/'+('left' if i==0 else 'right')+'/image')
    for name,value in [('estimator.yaml',config),('imu.yaml',imu),('cameras.yaml',cameras)]:
        (output/name).write_text('%YAML:1.0\n'+yaml.dump(value,Dumper=OpenCvDumper,sort_keys=False,default_flow_style=None))
    (output/'imu_from_base.json').write_text(json.dumps(imu_from_base.tolist(),indent=2)+'\n')
    receipt=dict(schema=1,source_commit=COMMIT,imu_axes=imu_axes,flight_authorized=False,
        imu_noise_authority=calibration.get('profile',{}).get('imu_noise_model','legacy SDK configuration; simulator ideal IMU'),
        gauge='gravity aligned local world, launch yaw/position supplied independently',
        initialization='static IMU and visual disparity windows; independently checked disarmed landed launch alignment',
        covariance='full posterior pose marginal; IMU local rotation -> world base point, full cross blocks',
        mount_authority='physical TF, not ambiguous SDK debug-dump translation',
        files_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(output.iterdir()) if p.is_file()})
    (output/'config-receipt.json').write_text(json.dumps(receipt,indent=2)+'\n');return receipt

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('recording',type=Path)
    parser.add_argument('--out',type=Path,required=True);parser.add_argument('--imu-axes',choices=('flu','opencv'),required=True)
    args=parser.parse_args();calib=json.loads((args.recording/'calibration.json').read_text())
    cameras=json.loads((args.recording/'sdk-input/stereo.edex').read_text())[0]['cameras']
    generate(calib,[c['intrinsics'] for c in cameras],args.out,imu_axes=args.imu_axes)

if __name__=='__main__':main()
