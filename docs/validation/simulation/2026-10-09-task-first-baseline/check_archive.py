#!/usr/bin/env python3
import gzip
import hashlib
import json
import math
from pathlib import Path
import sys
BASE=Path(__file__).resolve().parent
sys.path.insert(0,str(BASE.parents[3]/'scripts'))
from assess_px4_bt_suite import assess_case


def read(p):return p.read_bytes() if p.exists() else gzip.decompress(p.with_name(p.name+'.gz').read_bytes())


def main():
    for line in (BASE/'SHA256SUMS').read_text().splitlines():
        digest,name=line.split('  ',1)
        assert hashlib.sha256((BASE/name).read_bytes()).hexdigest()==digest,name
    p=BASE/'74fa88e8-dbd4-4165-98c7-f833746627d3'
    observation=json.loads(read(p/'observation.json'));flight=json.loads(read(p/'flight-observation.json'));truth=json.loads(read(p/'flight-truth.json'))
    assert assess_case('full',observation,flight,read(p/'bt-runner.log').decode(),truth)['passed']
    segments=[]
    for sample in truth:
        if sample['phase']=='HOVER':
            if not segments or previous!='HOVER':segments.append([])
            segments[-1].append(sample)
        previous=sample['phase']
    assert len(segments)==2
    for segment,duration in zip(segments,[30.,3.]):
        assert segment[-1]['mono']-segment[0]['mono']>=duration
        assert max(math.dist(s['position'],segment[0]['position']) for s in segment)<=.15
    failed=json.loads(read(BASE/'c0061261-8e89-45c9-824d-c14025766279/flight-observation.json'))
    assert failed['passed'] is False and failed['result']['reason']=='STALE_OR_INVALID_AIRCRAFT_STATE'
    assert failed['final_land'] and failed['final_arming']==1 and 'ARM_REQUEST' not in failed['phases']
    print('Full BT baseline and independent hover truth verified; UI failure retained. No VIO/avoidance acceptance.')

if __name__=='__main__':main()
