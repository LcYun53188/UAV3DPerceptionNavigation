#!/usr/bin/env python3
"""Freeze completed experimental chain using verified consumed-input snapshots."""

# Support canonical direct execution and historical flat imports.
import sys as _sys
from pathlib import Path as _Path
_scripts = str(_Path(__file__).resolve().parents[3])
if _scripts not in _sys.path:
    _sys.path.insert(0, _scripts)
from support.paths import enable_imports as _enable_imports
_enable_imports()

import argparse
import hashlib
import json
from pathlib import Path
import tarfile
from sim_validation import ROOT
from freeze_vio_run import verify

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def freeze(run, archive):
    result=json.loads((run/'result.json').read_text())
    if not result.get('cleanup_confirmed'):
        raise RuntimeError('Only completed owned runs may be frozen')
    manifest=json.loads((run/'manifest.json').read_text())
    files={}
    for name,digest in manifest['input_sha256'].items():
        snapshot=run/'consumed-inputs'/name
        path=snapshot if snapshot.exists() else ROOT/name
        if sha(path)!=digest:raise RuntimeError('Consumed source unavailable: '+name)
        files['inputs/'+name]=path
    for path in sorted(run.rglob('*')):
        relative=path.relative_to(run)
        if path.is_file() and relative.parts[0]!='consumed-inputs' and path.suffix not in ('.ppm','.tga'):
            files['runs/'+run.name+'/'+str(relative)]=path
    if archive.exists():raise RuntimeError('Refuse overwrite')
    archive.parent.mkdir(parents=True,exist_ok=True)
    with tarfile.open(archive,'w:gz',dereference=True) as tar:
        for name,path in sorted(files.items()):tar.add(path,arcname=name,recursive=False)
    index=archive.with_suffix('.index.json')
    index.write_text(json.dumps(dict(schema=1,run_id=run.name,archive_sha256=sha(archive),
        archive_bytes=archive.stat().st_size,members={name:sha(path) for name,path in files.items()},
        scope='completed owned simulation; source snapshots verified against prelaunch hashes; loose preview pixels excluded; optional verified preview archives retained'),indent=2)+'\n')
    verify(archive,index)
    print('Frozen and verified',run.name,len(files))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('--archive',type=Path,required=True)
    a=p.parse_args();freeze(a.run,a.archive)
