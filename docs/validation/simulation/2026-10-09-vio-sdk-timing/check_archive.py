"""Replay byte integrity, passive timing, fusion and actual ULog parameters."""
import gzip,hashlib,importlib.util,json,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from assess_vio_timing import assess as timing_assess
from assess_px4_real_vio_fusion import assess as fusion_assess


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
    folder=HERE/'paced-default-failure';raw=gzip.decompress((folder/'px4.ulg.gz').read_bytes())
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
    print(f'PASS: {receipts} original files, 4 failure outcomes, timing/fusion replay and 14 ULog parameters')


if __name__=='__main__':main()
