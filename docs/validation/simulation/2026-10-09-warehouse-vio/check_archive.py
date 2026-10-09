"""Replay warehouse geometry, original bytes and independent motion/fusion evidence."""
import gzip
import hashlib
import json
import math
from pathlib import Path
import sys
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from assess_vio_warehouse import assess,read
from assess_vio_motion_evidence import replay
from assess_px4_real_vio_fusion import assess as fusion


def main():
    count=0
    for name,expected in read(HERE,'summary.json').items():
        folder=HERE/name;manifest=read(folder,'manifest.json');result=read(folder,'result.json')
        assert manifest['run_id']==expected['run_id']
        assert result['passed']==expected['passed'] and result['cleanup_confirmed']
        for receipt in read(folder,'origin-files.json').values():
            p=folder/receipt['archive'];raw=p.read_bytes()
            if p.suffix=='.gz':raw=gzip.decompress(raw)
            assert len(raw)==receipt['bytes'] and hashlib.sha256(raw).hexdigest()==receipt['sha256']
            count+=1
        assert assess(folder)==read(folder,'warehouse-assessment.json')
        assert not read(folder,'warehouse-assessment.json')['flight_authorized']
        if manifest['motion']:
            actual=replay(folder);reported=read(folder,'motion-independent-assessment.json')
            for key,value in actual.items():
                if key in ('position_rmse_m','position_max_m'):
                    assert math.isclose(value,reported[key],rel_tol=0,abs_tol=1e-12)
                else:assert value==reported[key]
            assert actual['reported_metrics_agree'] and actual['fmu_inputs_absent']
        else:assert fusion(folder)==read(folder,'independent-assessment.json')
    print(f'PASS: {count} original byte receipts; all six failed outcomes preserved; warehouse and motion/fusion replay agree')


if __name__=='__main__':main()
