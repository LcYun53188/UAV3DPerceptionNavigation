"""Verify byte receipts and replay source/motion/fusion assessments and live TF."""
import gzip
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from diagnose_vio_motion import diagnose
from assess_vio_warehouse import assess,read
from assess_vio_motion_evidence import replay,rotation
from assess_px4_real_vio_fusion import assess as fusion
from audit_vio_sdk_dump import timeline


def raw(path):
    return path.read_bytes() if path.is_file() else gzip.decompress(Path(str(path)+'.gz').read_bytes())


def main():
    count=0
    for name,expected in read(HERE,'summary.json').items():
        folder=HERE/name;m=read(folder,'manifest.json');r=read(folder,'result.json')
        assert m['run_id']==expected['run_id'] and r['passed']==expected['passed']
        assert r['cleanup_confirmed'] and r['arming_states']==[1]
        for receipt in read(folder,'origin-files.json').values():
            path=folder/receipt['archive'];data=path.read_bytes()
            if path.suffix=='.gz':data=gzip.decompress(data)
            assert len(data)==receipt['bytes'] and hashlib.sha256(data).hexdigest()==receipt['sha256']
            count+=1
        assert assess(folder)==read(folder,'warehouse-assessment.json')
        assert not read(folder,'warehouse-assessment.json')['flight_authorized']
        if m['motion']:
            a=replay(folder);reported=read(folder,'motion-independent-assessment.json')
            for key,value in a.items():
                if key in ('position_rmse_m','position_max_m'):
                    assert math.isclose(value,reported[key],rel_tol=0,abs_tol=1e-12)
                else:assert value==reported[key]
            assert a['reported_metrics_agree'] and a['fmu_inputs_absent']
            diagnosis=diagnose(folder)
            assert diagnosis==read(folder,'motion-diagnosis.json') and diagnosis['metrics_match']
        else:assert fusion(folder)==read(folder,'independent-assessment.json')
        if 'runtime_mounts' in r:
            theta=math.radians(m['profile']['camera_pitch_deg'])
            ry=np.array([[math.cos(theta),0,math.sin(theta)],[0,1,0],[-math.sin(theta),0,math.cos(theta)]])
            optical=np.array([[0,0,1],[-1,0,0],[0,-1,0]])
            frames=read(folder,'assets/frames.json')
            assert set(r['runtime_mounts'])==set(frames)
            for child,mount in r['runtime_mounts'].items():
                assert r['checks']['runtime_tf:'+child] and mount['parent']=='base_link'
                assert np.allclose(mount['position'],frames[child]['position'],atol=1e-10,rtol=0)
                assert np.allclose(rotation(mount['quaternion']),np.eye(3) if child=='vio_imu' else ry@optical,atol=1e-10,rtol=0)
        if m.get('sdk_debug_dump'):
            receipt=read(folder,'sdk-input-receipt.json');sdk=folder/'sdk-input'
            config=json.loads(raw(sdk/'stereo.edex'))[0]
            metadata=[json.loads(line) for line in raw(sdk/'frame_metadata.jsonl').splitlines()]
            imu=[json.loads(line) for line in raw(sdk/'IMU.jsonl').splitlines()]
            assert receipt['frames']==len(metadata) and not receipt['flight_authorized']
            assert receipt['native_camera_transforms']==[c['transform'] for c in config['cameras']]
            assert receipt['native_imu_transform']==config['imu']['transform']
            if 'runtime_mounts' in r:
                imu_rotation=np.asarray(config['imu']['transform'])[:,:3]
                for camera in config['cameras']:
                    relative=imu_rotation.T@np.asarray(camera['transform'])[:,:3]
                    angle=math.degrees(math.acos(float(np.clip((np.trace(relative)-1)/2,-1,1))))
                    assert math.isclose(angle,m['profile']['camera_pitch_deg'],abs_tol=1e-4)
            assert receipt['sdk_configuration']==config['configuration']
            assert receipt['imu_timeline']==timeline([v['timestamp'] for v in imu])
            stamps={0:[],1:[]};paths=set()
            for f in metadata:
                assert {c['id'] for c in f['cams']}=={0,1} and len({c['timestamp'] for c in f['cams']})==1
                for c in f['cams']:
                    assert c['filename'] not in paths;paths.add(c['filename']);stamps[c['id']].append(c['timestamp'])
            assert paths==set(receipt['image_receipts']) and receipt['stereo_paired']
            for k,v in stamps.items():assert receipt['camera_timeline'][str(k)]==timeline(v)
            for n,v in receipt['metadata_receipts'].items():assert hashlib.sha256(raw(sdk/n)).hexdigest()==v['sha256']
            for side in (0,1):
                n=f'images/cam{side}.00000.tga';v=receipt['image_receipts'][n]
                assert hashlib.sha256(raw(sdk/n)).hexdigest()==v['sha256']
    print(f'PASS: {count} original byte receipts; live mounts and motion/fusion replay agree; SDK metadata and archived image samples agree (full SDK image replay not performed)')


if __name__=='__main__':main()
