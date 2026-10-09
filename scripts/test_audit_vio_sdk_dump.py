"""SDK receipt rejects ambiguous image inputs and preserves consumed timestamps."""
import json
from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parent))
from audit_vio_sdk_dump import audit


def fixture(folder):
    config=dict(cameras=[dict(transform=[[1,0,0,0]])]*2,imu=dict(transform=[[1,0,0,0]]),configuration=dict(odometry_mode=1))
    (folder/'stereo.edex').write_text(json.dumps([config,{}]))
    (folder/'IMU.jsonl').write_text('\n'.join(json.dumps(dict(timestamp=t)) for t in [100,104,108]))
    frames=[]
    for i,t in enumerate([100,140]):
        cams=[]
        for index in [0,1]:
            name=f'cam{index}.{i}.tga';(folder/name).write_bytes(bytes([i,index]))
            cams.append(dict(id=index,filename=name,timestamp=t))
        frames.append(dict(frame_id=i,cams=cams))
    (folder/'frame_metadata.jsonl').write_text('\n'.join(map(json.dumps,frames)))
    return frames


def test_original_stamps_and_image_bytes_are_receipted(tmp_path):
    fixture(tmp_path);r=audit(tmp_path)
    assert r['stereo_paired'] and r['frames']==2 and not r['flight_authorized']
    assert r['camera_timeline']['0']==dict(count=2,first_ns=100,last_ns=140,strictly_ordered=True,max_gap_ns=40)
    assert r['imu_timeline']['max_gap_ns']==4
    assert len(r['image_receipts'])==4 and all(v['bytes']==2 for v in r['image_receipts'].values())


def test_unpaired_consumed_images_are_reported(tmp_path):
    frames=fixture(tmp_path);frames[0]['cams'][1]['timestamp']=101
    (tmp_path/'frame_metadata.jsonl').write_text('\n'.join(map(json.dumps,frames)))
    assert not audit(tmp_path)['stereo_paired']


def test_duplicate_image_path_is_rejected(tmp_path):
    frames=fixture(tmp_path);frames[1]['cams'][0]['filename']=frames[0]['cams'][0]['filename']
    (tmp_path/'frame_metadata.jsonl').write_text('\n'.join(map(json.dumps,frames)))
    with pytest.raises(ValueError,match='duplicate'):audit(tmp_path)
