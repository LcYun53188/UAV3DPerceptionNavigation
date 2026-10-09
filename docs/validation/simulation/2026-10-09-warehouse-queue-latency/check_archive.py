"""Replay original bytes, timing windows, fusion and all ULog parameter records."""
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from assess_vio_warehouse import assess as warehouse,read
from assess_px4_real_vio_fusion import assess as fusion
from assess_vio_timing import assess as timing
from diagnose_vio_deadline import diagnose


def main():
    count=0
    spec=importlib.util.spec_from_file_location('ulog',HERE.parent/'2026-10-09-real-vio-fusion/extract_parameters.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    for name,expected in read(HERE,'summary.json').items():
        folder=HERE/name;m=read(folder,'manifest.json');r=read(folder,'result.json')
        assert m['run_id']==expected['run_id'] and r['passed']==expected['passed']
        assert r['cleanup_confirmed'] and r['arming_states']==[1]
        for receipt in read(folder,'origin-files.json').values():
            p=folder/receipt['archive'];data=p.read_bytes()
            if p.suffix=='.gz':data=gzip.decompress(data)
            assert len(data)==receipt['bytes'] and hashlib.sha256(data).hexdigest()==receipt['sha256'];count+=1
        assert warehouse(folder)==read(folder,'warehouse-assessment.json')
        assert not read(folder,'warehouse-assessment.json')['flight_authorized']
        assert fusion(folder)==read(folder,'independent-assessment.json')
        assert timing(folder)==read(folder,'timing-assessment.json')
        assert diagnose(folder)==read(folder,'deadline-diagnosis.json')
        if 'sdk_runtime_receipt' in r:
            receipt=read(folder,'sdk-runtime-receipt.json')
            assert receipt==r['sdk_runtime_receipt'] and receipt['passed']
            assert receipt['expected']==receipt['actual']
            assert receipt['actual']['image_qos_depth']==(m.get('requested_sdk_image_depth') or 10)
            assert receipt['actual']['tracking_mode']==1 and receipt['actual']['num_cameras']==2
        raw=gzip.decompress((folder/'px4.ulg.gz').read_bytes());origin=read(folder,'ulog-origin.json')
        assert len(raw)==origin['bytes'] and hashlib.sha256(raw).hexdigest()==origin['sha256']
        expected_params={k.removeprefix('PX4_PARAM_'):float(v) for k,v in m['px4_parameter_overrides'].items()}
        expected_params.update(EKF2_DELAY_MAX=float(m.get('requested_ekf_delay_max_ms') or 200),EKF2_EV_DELAY=0.)
        assert origin['expected']==expected_params
        with tempfile.TemporaryDirectory() as temp:
            p=Path(temp)/'px4.ulg';p.write_bytes(raw);parameters,_=module.extract(p,expected_params)
            original=read(folder,'effective-parameters.json')
            for key in ('passed','checks','parameters','history','source_sha256','initial_header_bytes'):
                assert parameters[key]==original[key]
            assert parameters['passed'] or (name=='nvidia-depth10' and not r['passed'] and
                {k for k,v in parameters['checks'].items() if not v}=={'EKF2_DELAY_MAX','EKF2_EV_DELAY'})
    print(f'PASS: {count} original byte receipts; fusion, warehouse, clock-window and deadline replay; full ULog replay agrees (one incomplete historical parameter receipt preserved)')


if __name__=='__main__':main()
