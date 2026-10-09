import json
from px4_comm_bridge.source_timing import SourceTiming


def test_timing_is_bounded_and_preserves_host_metadata_without_mutating_info(tmp_path):
    timing=SourceTiming(2);metadata={'source_timestamp':123,'received_timestamp':456,'publisher_gid':bytes([1])}
    timing.record('pose',10.,9.95,metadata)
    timing.record('tracking',10.04,10.,metadata)
    timing.record('retire',10.24,10.,reason='STALE')
    target=tmp_path/'trace.json';timing.write(target);data=json.loads(target.read_text())
    assert data['total']==3 and data['retained']==2
    assert data['records'][0]['rmw']=={'source_timestamp':123,'received_timestamp':456}
    assert data['records'][1]['reason']=='STALE'
    assert metadata['publisher_gid']==bytes([1])
