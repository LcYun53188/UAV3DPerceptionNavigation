#!/usr/bin/env python3
"""Freeze a completed run and its exact consumed inputs before further edits."""

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

ROOT=Path(__file__).resolve().parents[4]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def verify(archive,index):
    receipt=json.loads(index.read_text())
    if sha(archive)!=receipt['archive_sha256'] or archive.stat().st_size!=receipt['archive_bytes']:
        raise ValueError('Frozen archive identity mismatch')
    observed={}
    with tarfile.open(archive,'r|gz') as tar:
        for member in tar:
            path=Path(member.name)
            if not member.isfile() or path.is_absolute() or '..' in path.parts or member.name in observed:
                raise ValueError('Unsafe or duplicate archive member')
            value=hashlib.sha256();stream=tar.extractfile(member)
            for block in iter(lambda:stream.read(1024*1024),b''):value.update(block)
            observed[member.name]=value.hexdigest()
    if observed!=receipt['members']:raise ValueError('Frozen member identity mismatch')
    return receipt
def freeze(folder,archive):
    manifest=json.loads((folder/'manifest.json').read_text())
    files={'inputs/'+n:ROOT/n for n in manifest['input_sha256']}
    for n,p in files.items():
        if sha(p)!=manifest['input_sha256'][n.removeprefix('inputs/')]:
            raise RuntimeError('Input drift before freeze: '+n)
    if archive.exists():raise RuntimeError('Refuse archive overwrite')
    archive.parent.mkdir(parents=True,exist_ok=True)
    for p in sorted(folder.rglob('*')):
        if p.is_file() and p.suffix not in ('.ppm','.tga'):
            files['runs/'+folder.name+'/'+str(p.relative_to(folder))]=p
    with tarfile.open(archive,'w:gz',dereference=True) as tar:
        for name,p in sorted(files.items()):tar.add(p,arcname=name,recursive=False)
    receipt=dict(schema=1,run_id=folder.name,archive_sha256=sha(archive),archive_bytes=archive.stat().st_size,
        excluded_suffixes=['.ppm','.tga'],members={n:sha(p) for n,p in files.items()})
    archive.with_suffix('.index.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print('Frozen',len(files),'files:',archive,flush=True);return receipt
def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('folder',type=Path)
    parser.add_argument('--archive',required=True,type=Path);args=parser.parse_args();freeze(args.folder,args.archive)
if __name__=='__main__':main()
