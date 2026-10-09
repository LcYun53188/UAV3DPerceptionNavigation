"""Archived flight proof cannot hide timestamp rewrites or truth drift."""
import copy
from pathlib import Path
import pytest
import assess_warehouse_flight as audit
from assess_vio_warehouse import read

FOLDER=Path(__file__).resolve().parents[1]/'docs/validation/simulation/2026-10-09-warehouse-bt-flight/hover08-pass'


def test_actual_low_flight_is_not_planned_height_acceptance():
    r=audit.assess(FOLDER)
    assert r['passed'] and r['takeoff_height_m']==.8 and not r['planned_1p5m_hover_passed']


@pytest.mark.parametrize('fault',['timestamp','truth','bt','source'])
def test_failed_provenance_or_truth_cannot_be_hidden_by_reported_pass(monkeypatch,fault):
    def altered(folder,name):
        value=copy.deepcopy(read(folder,name))
        if fault=='timestamp' and name=='fusion-outputs.json':value[0]['ev']['timestamp_sample']+=1
        if fault=='truth' and name=='flight-truth.json':
            events=read(folder,'flight-events.json');hover=next(e for e in events if e['phase']=='HOVER')
            next(t for t in value if t['mono']>hover['mono']+.5)['position'][0]+=1.
        if fault=='bt' and name=='flight-observation.json':value['bt_dispatch_count']=2
        if fault=='source' and name=='result.json':value['flight_pose_session']['source_fault']='VIO_RECEIVE_GAP'
        return value
    monkeypatch.setattr(audit,'read',altered)
    assert not audit.assess(FOLDER)['passed']
