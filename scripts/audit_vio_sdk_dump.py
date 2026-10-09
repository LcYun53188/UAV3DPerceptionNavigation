#!/usr/bin/env python3
"""Read actual SDK-consumed input dump; diagnostic receipt, never flight evidence."""
import argparse
import hashlib
import json
from pathlib import Path


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def timeline(stamps):
    gaps=[b-a for a,b in zip(stamps,stamps[1:])]
    return dict(count=len(stamps),first_ns=stamps[0] if stamps else None,
        last_ns=stamps[-1] if stamps else None,strictly_ordered=bool(stamps) and all(g>0 for g in gaps),
        max_gap_ns=max(gaps,default=None))


def audit(directory):
    config=json.loads((directory/'stereo.edex').read_text())[0]
    frames=rows(directory/'frame_metadata.jsonl');imu=rows(directory/'IMU.jsonl')
    stamps={0:[],1:[]};images={};paired=True
    for frame in frames:
        cams=frame['cams']
        paired &= len(cams)==2 and {c['id'] for c in cams}=={0,1} and len({c['timestamp'] for c in cams})==1
        for camera in cams:
            index=camera['id']
            if index not in stamps: raise ValueError('Unexpected camera index')
            name=camera['filename'];path=(directory/name).resolve()
            if not path.is_relative_to(directory.resolve()) or name in images:
                raise ValueError('Unsafe or duplicate image path')
            raw=path.read_bytes()
            images[name]=dict(bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest())
            stamps[index].append(camera['timestamp'])
    receipts={}
    for name in ('stereo.edex','frame_metadata.jsonl','IMU.jsonl'):
        raw=(directory/name).read_bytes();receipts[name]=dict(bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest())
    return dict(schema=1,flight_authorized=False,scope='SDK consumed input only; no latency, bias, calibration or qualification proof',
        stereo_paired=bool(frames) and paired,frames=len(frames),camera_timeline={str(k):timeline(v) for k,v in stamps.items()},
        imu_timeline=timeline([r['timestamp'] for r in imu]),
        native_camera_transforms=[c['transform'] for c in config['cameras']],
        native_imu_transform=config['imu']['transform'],sdk_configuration=config['configuration'],
        metadata_receipts=receipts,image_receipts=images)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();result=audit(args.directory)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('metadata_receipts','image_receipts')},indent=2))
