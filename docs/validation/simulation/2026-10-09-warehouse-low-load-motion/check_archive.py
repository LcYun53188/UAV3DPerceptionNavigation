"""Replay archived bytes, motion metrics and warehouse acceptance; no live graph."""
import gzip
import hashlib
import math
from pathlib import Path
import sys
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parents[3]/'scripts'))
from assess_vio_warehouse import assess,read
from assess_vio_motion_evidence import replay
from diagnose_vio_motion import diagnose
from px4_vio_motion_audit import assess_motion


def equal_numeric(a,b):
    # BLAS implementations can differ in last-bit accumulation only. Raw receipt
    # bytes, categorical outcomes and integer sample counts remain exact.
    if isinstance(a,float) or isinstance(b,float):
        return math.isclose(a,b,rel_tol=1e-12,abs_tol=1e-12)
    if isinstance(a,dict):return a.keys()==b.keys() and all(equal_numeric(a[k],b[k]) for k in a)
    if isinstance(a,list):return len(a)==len(b) and all(equal_numeric(x,y) for x,y in zip(a,b))
    return a==b


def main():
    folder=HERE/'motion';r=read(folder,'result.json');m=read(folder,'manifest.json')
    receipts=read(folder,'origin-files.json')
    for receipt in receipts.values():
        p=folder/receipt['archive'];raw=p.read_bytes()
        if p.suffix=='.gz':raw=gzip.decompress(raw)
        assert len(raw)==receipt['bytes'] and hashlib.sha256(raw).hexdigest()==receipt['sha256']
    assert m['run_id']=='ad713ed2-73b9-41bf-9771-ee06dde79db4'
    assert r['passed'] and r['cleanup_confirmed'] and r['arming_states']==[1]
    assert not any(r['control_publishers'].values()) and not m['fuse_pose']
    assert m['requested_sdk_image_depth']==1 and m['requested_real_time_factor']==.8
    assert m['headless_rendering'] and m['render_device']=='nvidia'
    runtime=read(folder,'sdk-runtime-receipt.json')
    assert runtime==r['sdk_runtime_receipt'] and runtime['passed']
    assert runtime['actual']==runtime['expected'] and runtime['actual']['image_qos_depth']==1
    renderer=read(folder,'renderer-info.json')
    assert renderer==r['renderer'] and renderer['passed'] and 'NVIDIA' in renderer['vendor']
    assert hashlib.sha256(gzip.decompress((folder/'renderer.log.gz').read_bytes())).hexdigest()==renderer['sha256']
    assessment=assess(folder)
    assert assessment==read(folder,'warehouse-assessment.json') and assessment['passed']
    assert not assessment['flight_authorized']
    assert replay(folder)==read(folder,'motion-independent-assessment.json')
    assert equal_numeric(diagnose(folder),read(folder,'motion-diagnosis.json'))
    motion=assess_motion(read(folder,'truth.json'),read(folder,'normalized-poses.json'),read(folder,'motion-imu.json'))
    assert motion['passed']
    for key,value in motion.items():
        if key=='checks':assert all(r['motion']['checks'][k]==v for k,v in value.items())
        else:assert equal_numeric(value,r['motion'][key])
    print(f'PASS: {len(receipts)} original receipts; normalized/raw motion, renderer, SDK and warehouse replay')


if __name__=='__main__':main()
