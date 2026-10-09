"""Separate SDK source health from a missed bridge deadline; never allow recovery."""
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from diagnose_vio_deadline import diagnose
from test_assess_vio_timing import record


def save(folder,**streams):
    for name,records in streams.items():
        (folder/(name+'-timing.json')).write_text(json.dumps(dict(records=records,total=len(records),retained=len(records))))


def test_healthy_next_source_arriving_after_fault_is_not_accepted(tmp_path):
    last=record('accepted',.1,.09)
    fault=dict(record('watchdog_fault',.204,.203),reason='VIO_SAMPLE_STALE')
    next_emit=record('pose_emit',.22,.21);next_emit['sample']=10.08
    receive=dict(next_emit,stage='pose_rx',mono=100.221)
    save(tmp_path,fusion=[last,fault,receive],normalizer=[next_emit],sensor=[])
    r=diagnose(tmp_path)
    assert r['first_normalizer_retirement'] is None and not r['flight_authorized']
    assert abs(r['next_emit_after_fault_s']-.016)<1e-9
    assert r['next_fusion_receive']['sample']==10.08
    assert r['last_accepted']['sample']==10.


def test_no_fusion_fault_is_reported_without_fabricated_failure(tmp_path):
    save(tmp_path,fusion=[record('accepted',.1,.09)],normalizer=[],sensor=[])
    r=diagnose(tmp_path)
    assert r['first_fusion_fault'] is None and 'fault_window_timing' not in r and not r['flight_authorized']
