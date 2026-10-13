#!/usr/bin/env python3
"""Resolve SDK15 dump body-reference ambiguity using physical mount calibration."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from prepare_openvins_config import mount, CV_FROM_FLU

def generate(recording, output):
    if output.exists():
        raise ValueError('Refuse calibration overwrite')
    calibration=json.loads((recording/'calibration.json').read_text())
    dump=json.loads((recording/'sdk-input/stereo.edex').read_text())[0]
    tf=calibration['frames']
    cv_from_base=np.eye(4);cv_from_base[:3,:3]=CV_FROM_FLU
    flu_from_cv=np.eye(4);flu_from_cv[:3,:3]=CV_FROM_FLU.T
    imu=cv_from_base@mount(tf['vio_imu'])@flu_from_cv
    cameras=[cv_from_base@mount(tf[name]) for name in ('vio_left_optical','vio_right_optical')]
    def homogeneous(value):
        result=np.eye(4);result[:3]=value;return result
    dumped_imu=homogeneous(dump['imu']['transform'])
    for i,camera in enumerate(cameras):
        relative=np.linalg.inv(imu)@camera
        dumped_relative=np.linalg.inv(dumped_imu)@homogeneous(dump['cameras'][i]['transform'])
        if not np.allclose(relative,dumped_relative,atol=3e-7,rtol=0):
            raise ValueError('Physical calibration changes relative sensor extrinsics')
    receipt=dict(schema=1,contract='physical_base_sdk_opencv_imu_v1',
        calibration_sha256=hashlib.sha256((recording/'calibration.json').read_bytes()).hexdigest(),
        dump_sha256=hashlib.sha256((recording/'sdk-input/stereo.edex').read_bytes()).hexdigest(),
        rig_from_cameras=[c[:3].tolist() for c in cameras],rig_from_imu=imu[:3].tolist(),
        relative_extrinsics_preserved=True,scope='base reference correction only, pixels and IMU unchanged')
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(receipt,indent=2)+'\n');return receipt

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording',type=Path);parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args();generate(args.recording,args.out)

if __name__=='__main__':main()
