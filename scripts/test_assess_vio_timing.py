import gzip,json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from assess_vio_timing import assess


def record(stage,delay,published):
    start=1790000000000000000
    return dict(stage=stage,mono=100.+delay,system_ns=start+round(delay*1e9),
        ros=10.+delay,sample=10.,age_s=delay,rmw={'source_timestamp':start+round(published*1e9)})


def write(folder,sensor,normalizer):
    for name,records in (('sensor-timing',sensor),('normalizer-timing',normalizer)):
        (folder/(name+'.json')).write_text(json.dumps(dict(records=records,total=len(records),retained=len(records))))


def test_original_sample_matching_separates_pipeline_and_callback_transport(tmp_path):
    sensor=[record('left',.002,0.),record('right',.003,.001),record('pose_cov',.042,.041)]
    norm=[record('pose_rx',.043,.041)]
    write(tmp_path,sensor,norm);r=assess(tmp_path)
    assert r['host_clock_stable'] and r['trace_complete']
    assert abs(r['stereo_to_sdk_publish']['max_s']-.04)<1e-9
    assert abs(r['metrics']['normalizer:pose_rx']['publisher_to_callback']['max_s']-.002)<1e-9


def test_system_clock_jump_prevents_pipeline_interpretation(tmp_path):
    sensor=[record('left',.002,0.),record('right',.003,.001),record('pose_cov',.042,.041)]
    norm=[record('pose_rx',.043,.041)];norm[0]['system_ns']+=100000000
    write(tmp_path,sensor,norm);r=assess(tmp_path)
    assert not r['host_clock_stable'] and r['stereo_to_sdk_publish'] is None and not r['worst_pipeline']


def test_gzip_archive_retains_timing_assessment(tmp_path):
    sensor=[record('left',.002,0.),record('right',.003,.001),record('pose_cov',.042,.041)]
    write(tmp_path,sensor,[record('pose_rx',.043,.041)])
    expected=assess(tmp_path)
    for path in tmp_path.glob('*.json'):
        Path(str(path)+'.gz').write_bytes(gzip.compress(path.read_bytes(),mtime=0))
        path.unlink()
    assert assess(tmp_path)==expected


def test_sdk_callback_subtraction_is_only_outside_lower_bound(tmp_path):
    sensor=[record('left',.002,0.),record('right',.003,.001),record('pose_cov',.042,.041),
        dict(record('tracking',.044,.043),sdk_track_s=.010,sdk_callback_s=.015)]
    write(tmp_path,sensor,[record('pose_rx',.043,.041)])
    sdk=assess(tmp_path)['sdk_execution']
    assert sdk['valid_count']==1 and sdk['invalid_count']==0
    assert abs(sdk['outside_callback_lower_bound']['max_s']-.025)<1e-9
    assert abs(sdk['callback_without_track']['max_s']-.005)<1e-9
    sensor[-1]['sdk_callback_s']=.050
    write(tmp_path,sensor,[])
    assert assess(tmp_path)['sdk_execution']['outside_callback_lower_bound']['max_s']==0


def test_invalid_sdk_duration_does_not_become_latency_evidence(tmp_path):
    sensor=[dict(record('tracking',.044,.043),sdk_track_s=float('nan'),sdk_callback_s=.01),
        dict(record('tracking',.044,.043),sdk_track_s=.02,sdk_callback_s=.01)]
    write(tmp_path,sensor,[])
    sdk=assess(tmp_path)['sdk_execution']
    assert sdk['valid_count']==0 and sdk['invalid_count']==2
    assert sdk['track']['count']==0 and sdk['outside_callback_lower_bound']['count']==0


def test_later_clock_jump_does_not_invalidate_explicit_earlier_window(tmp_path):
    sensor=[record('left',.002,0.),record('right',.003,.001),record('pose_cov',.042,.041)]
    late=record('pose_rx',3.,2.99);late['system_ns']+=500000000
    write(tmp_path,sensor,[record('pose_rx',.043,.041),late])
    assert not assess(tmp_path)['host_clock_stable']
    early=assess(tmp_path,after_mono=100.,before_mono=100.1)
    assert early['host_clock_stable'] and early['trace_complete']
    assert abs(early['stereo_to_sdk_publish']['max_s']-.04)<1e-9
    assert not assess(tmp_path)['host_clock_stable']  # Input and full interpretation unchanged.


def test_window_spanning_clock_jump_still_rejects_pipeline(tmp_path):
    sensor=[record('left',.002,0.),record('right',.003,.001),record('pose_cov',.042,.041)]
    late=record('pose_rx',.043,.041);late['system_ns']+=500000000
    write(tmp_path,sensor,[late])
    result=assess(tmp_path,after_mono=100.,before_mono=100.1)
    assert not result['host_clock_stable'] and result['stereo_to_sdk_publish'] is None
