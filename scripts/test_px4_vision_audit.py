"""Negative assessment checks: topic presence alone must never pass fusion audit."""
from collections import Counter
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import pytest
from px4_msgs.msg import EstimatorAidSource1d, EstimatorSelectorStatus

sys.path.insert(0, str(Path(__file__).resolve().parent))
from px4_vision_audit import VisionFusionAudit


def completed_audit():
    audit = object.__new__(VisionFusionAudit)
    audit.node = SimpleNamespace(get_publishers_info_by_topic=lambda _: [])
    audit.topics = {}
    audit.gate = SimpleNamespace(reason='VIO_TELEMETRY_STALE:source')
    audit.counts = Counter()
    audit.fused_counts = Counter({n:10 for n in ('ev_pos','ev_hgt','ev_vel','ev_yaw')})
    audit.reasons = Counter()
    audit.ready_before_stop, audit.ready_count, audit.input_count = True,10,20
    audit.aiding_before_stop = {n:False for n in ('cs_gnss_pos','cs_gnss_vel','cs_gnss_yaw','cs_gps_hgt','cs_mag_hdg','cs_mag_3d','cs_mag','cs_opt_flow')}
    audit.last = {n:EstimatorAidSource1d(time_last_fuse=100) for n in audit.fused_counts}
    audit.stop_snapshot = {n:90 for n in audit.fused_counts}  # queued samples may still fuse
    audit.drained_snapshot = {n:100 for n in audit.fused_counts}
    return audit


@pytest.mark.parametrize('fault', ['no_fusion','never_ready','still_ready','continued_fusion','hidden_gnss','control_writer'])
def test_incomplete_or_contaminated_audit_fails(fault):
    audit = completed_audit()
    if fault == 'no_fusion': audit.fused_counts['ev_vel'] = 0
    if fault == 'never_ready': audit.ready_before_stop = False
    if fault == 'still_ready': audit.gate.reason = 'READY'
    if fault == 'continued_fusion': audit.last['ev_pos'].time_last_fuse = 101
    if fault == 'hidden_gnss': audit.aiding_before_stop['cs_gnss_pos'] = True
    if fault == 'control_writer': audit.node.get_publishers_info_by_topic = lambda _: [object()]
    assert not audit.result()['passed']


def test_drained_fusion_can_stop_and_unavailable_selector_slots_are_json_safe():
    audit = completed_audit()
    selector = EstimatorSelectorStatus()
    selector.combined_test_ratio = [0.] + [float('nan')]*8
    audit.last['selector'] = selector
    result = audit.result()
    assert result['passed']
    assert result['last_messages']['selector']['combined_test_ratio'][1] is None
    json.dumps(result, allow_nan=False)
