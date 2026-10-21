from types import SimpleNamespace as NS
from uav_mission.manual_input_evidence import ManualInputEvidence


def test_native_invalid_and_reception_vs_source_gaps_remain_distinct():
    evidence=ManualInputEvidence()
    evidence.receive(NS(timestamp=1000000,valid=True,data_source=2),1.)
    evidence.receive(NS(timestamp=1040000,valid=True,data_source=2),1.04)
    evidence.receive(NS(timestamp=1080000,valid=False,data_source=2),1.5)
    snapshot=evidence.snapshot(1.6)
    assert snapshot['samples']==3 and snapshot['invalid_samples']==1
    assert .45 < snapshot['max_receive_gap_s'] < .47
    assert snapshot['max_source_gap_s']==.04
    assert len(snapshot['events'])==2 and not snapshot['events'][-1]['valid']


def test_evidence_is_bounded_and_duplicates_are_not_interpreted_as_fresh_input():
    evidence=ManualInputEvidence()
    assert evidence.snapshot(1.)['receive_age_s'] is None
    for i in range(100):evidence.receive(NS(timestamp=1000000,valid=True,data_source=2),i*.02)
    snapshot=evidence.snapshot(2.)
    assert snapshot['non_increasing_stamps']==99 and len(snapshot['events'])==64
    assert snapshot['samples']==100
