#!/usr/bin/env python3
import gzip
import hashlib
import json
from pathlib import Path
BASE=Path(__file__).resolve().parent


def read(p):return json.loads(p.read_bytes() if p.exists() else gzip.decompress(p.with_name(p.name+'.gz').read_bytes()))


def main():
    for line in (BASE/'SHA256SUMS').read_text().splitlines():
        digest,name=line.split('  ',1)
        assert hashlib.sha256((BASE/name).read_bytes()).hexdigest()==digest,name
    r=read(BASE/'planning/result.json')
    assert r['passed'] and r['execution_admission']['passed'] and not r['flight_control_topics']
    assert r['normal']['detour_y_m']>.9 and r['bound_count']==1 and all(x==0 for x in r['cleanup_exit_codes'])
    p=BASE/'missing-planning-sitl'
    f=read(p/'flight-observation.json');d=read(p/'flight-diagnostics.json')
    assert not f['passed'] and f['navigation_backend']=='EGO' and f['final_land'] and f['final_arming']==1
    assert any(x.get('admission',{}).get('reason')=='INVALID_REQUEST:NON_UNIQUE_PLANNING_SOURCE' for x in d)
    assert read(p/'flight-commands.json')==[]
    print('Archived planner admission and real SITL pre-arm rejection verified; no curve flight claim.')

if __name__=='__main__':main()
