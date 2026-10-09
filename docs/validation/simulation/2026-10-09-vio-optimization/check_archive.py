"""Replay byte integrity, passive timing, fusion and actual ULog parameters."""
import gzip,hashlib,importlib.util,json,math,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from assess_vio_timing import assess as timing_assess
from assess_px4_real_vio_fusion import assess as fusion_assess
from assess_vio_motion_evidence import replay as motion_replay


def read(folder,name):
    path=folder/name
    return json.loads(path.read_text()) if path.is_file() else json.loads(gzip.decompress(Path(str(path)+'.gz').read_bytes()))


def main():
    receipts=0
    for name,expected in read(HERE,'summary.json').items():
        folder=HERE/name
        result=read(folder,'result.json')
        assert result['passed']==expected['passed'] and result['cleanup_confirmed']
        assert read(folder,'manifest.json')['run_id']==expected['run_id']
        for receipt in read(folder,'origin-files.json').values():
            path=folder/receipt['archive'];raw=path.read_bytes()
            if path.suffix=='.gz':raw=gzip.decompress(raw)
            assert len(raw)==receipt['bytes'] and hashlib.sha256(raw).hexdigest()==receipt['sha256']
            receipts+=1
        if 'timing' in expected:
            assert timing_assess(folder)==read(folder,'timing-assessment.json')
        if 'fusion' in expected:
            assert fusion_assess(folder)==read(folder,'independent-assessment.json')
        if name=='motion-current':
            replay=motion_replay(folder)
            reported=read(folder,'motion-independent-assessment.json')
            for field,value in replay.items():
                if field in ('position_rmse_m','position_max_m'):
                    assert math.isclose(value,reported[field],rel_tol=0,abs_tol=1e-12)
                else:assert value==reported[field]
            assert replay['error_limits_met'] and replay['reported_metrics_agree'] and replay['fmu_inputs_absent']
    folder=HERE/'paced-current';raw=gzip.decompress((folder/'px4.ulg.gz').read_bytes())
    origin=read(folder,'ulog-origin.json')
    assert len(raw)==origin['bytes'] and hashlib.sha256(raw).hexdigest()==origin['sha256']
    # Full archive scan, including any later parameter changes.
    spec=importlib.util.spec_from_file_location('params',HERE.parent/'2026-10-09-real-vio-fusion/extract_parameters.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    import tempfile
    with tempfile.TemporaryDirectory() as temp:
        path=Path(temp)/'px4.ulg';path.write_bytes(raw)
        manifest=read(folder,'manifest.json')
        parameters,header=module.extract(path,{k.removeprefix('PX4_PARAM_'):float(v) for k,v in manifest['px4_parameter_overrides'].items()})
        original=read(folder,'effective-parameters.json')
        for field in ('passed','checks','parameters','history','source_sha256','initial_header_bytes'):
            assert parameters[field]==original[field]
        assert parameters['passed']
    for name in ('missing-source-before-build-fix','missing-source-current'):
        folder=HERE/name
        for receipt in read(folder,'origin-files.json').values():
            path=folder/receipt['archive'];raw=path.read_bytes()
            if path.suffix=='.gz':raw=gzip.decompress(raw)
            assert len(raw)==receipt['bytes'] and hashlib.sha256(raw).hexdigest()==receipt['sha256']
            receipts+=1
        observation=read(folder,'observation.json')
        assert not observation['passed'] and not observation['xy_valid']
        assert observation['arming_states']==[1] and observation['landed']
        assert 'flight_passed' not in observation
        assert read(folder,'cleanup.json')['root_exit_codes']
    manifest=read(HERE/'missing-source-current','manifest.json')
    assert manifest['require_vio'] and manifest['vio_fusion_profile']=='aligned_pose_v1'
    assert manifest['vio_build']['added_publications']['estimator_aid_src_ev_pos']
    writers=read(HERE/'missing-source-current','observation.json')['writers']
    assert all(writers.get('/px4_7/fmu/out/estimator_aid_src_ev_'+kind)==1 for kind in ('pos','hgt','yaw'))
    print(f'PASS: {receipts} original files, all original outcomes, timing/fusion replay, 14 ULog parameters and missing-source preflight evidence')


if __name__=='__main__':main()
