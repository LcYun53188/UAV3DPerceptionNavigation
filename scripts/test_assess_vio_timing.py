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
