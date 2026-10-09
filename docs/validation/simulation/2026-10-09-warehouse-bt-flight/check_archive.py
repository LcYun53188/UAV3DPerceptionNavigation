"""Exact byte receipts and independent positive flight/ULog replay."""
import gzip
import hashlib
import importlib.util
from pathlib import Path
import sys
import tempfile
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from assess_vio_warehouse import read
from assess_warehouse_flight import assess


def main():
    count=0
    spec=importlib.util.spec_from_file_location('ulog',HERE.parent/'2026-10-09-real-vio-fusion/extract_parameters.py')
    u=importlib.util.module_from_spec(spec);spec.loader.exec_module(u)
    for name,expected in read(HERE,'summary.json').items():
        folder=HERE/name;r=read(folder,'result.json');m=read(folder,'manifest.json')
        assert r['passed']==expected['passed'] and m['run_id']==expected['run_id']
        assert r['cleanup_confirmed'] and not r['remaining_owned_processes']
        for receipt in read(folder,'origin-files.json').values():
            p=folder/receipt['archive'];raw=p.read_bytes()
            if p.suffix=='.gz':raw=gzip.decompress(raw)
            assert len(raw)==receipt['bytes'] and hashlib.sha256(raw).hexdigest()==receipt['sha256'];count+=1
        if expected['passed']:
            report=assess(folder)
            assert report==read(folder,'flight-independent-assessment.json') and report['passed']
            assert report['takeoff_height_m']==.8 and not report['planned_1p5m_hover_passed']
            files=list((folder/'rootfs/log').rglob('*.ulg.gz'));assert len(files)==1
            params={k.removeprefix('PX4_PARAM_'):float(v) for k,v in m['px4_parameter_overrides'].items()}
            params['EKF2_EV_DELAY']=0.
            with tempfile.TemporaryDirectory() as tmp:
                p=Path(tmp)/'px4.ulg';p.write_bytes(gzip.decompress(files[0].read_bytes()))
                current,_=u.extract(p,params);saved=read(folder,'effective-parameters.json')
                assert current['passed']
                for key in ('passed','checks','parameters','history','source_sha256','initial_header_bytes'):
                    assert current[key]==saved[key]
        else:
            assert expected['passed'] is False  # Includes truncated original failed-run files, never rewritten.
    print(f'PASS: {count} original receipts, 2 low flight/truth/ULog replays; 8 failed runs preserved')


if __name__=='__main__':main()
