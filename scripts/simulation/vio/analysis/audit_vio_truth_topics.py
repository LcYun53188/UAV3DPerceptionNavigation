#!/usr/bin/env python3
"""Read-only Gazebo topic graph proof for an active owned dynamic audit."""

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
import os
from pathlib import Path
import subprocess
import time
import threading


def observe(folder):
    manifest=json.loads((folder/'manifest.json').read_text())
    if not manifest.get('same_aircraft_motion'):raise ValueError('Owned same-aircraft fixture required')
    os.kill(manifest['processes']['gazebo']['pid'],0)
    env=dict(os.environ,GZ_PARTITION=manifest['partition'],GZ_IP='127.0.0.1');rows={}
    default='/model/'+manifest['model']+'/odometry_with_covariance'
    observer='/vio/truth_covariance_observer_only'
    for topic in (default,observer):
        r=subprocess.run(['gz','topic','-i','-t',topic],env=env,text=True,capture_output=True,timeout=3)
        rows[topic]=dict(command=r.args,exit_code=r.returncode,stdout=r.stdout,stderr=r.stderr)
    passed=all(r['exit_code']==0 for r in rows.values()) and 'No publishers on topic' in rows[default]['stdout'] and 'No subscribers on topic' in rows[observer]['stdout']
    receipt=dict(schema=1,run_id=folder.name,partition=manifest['partition'],
        captured_mono=time.monotonic(),passed=passed,topics=rows,
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (folder/'truth-topic-isolation.json').write_text(json.dumps(receipt,indent=2)+'\n')
    return receipt


class TruthTopicAudit:
    """Run CLI probes in an owned worker, away from the live ROS watchdog."""
    def __init__(self,folder):
        self.folder=folder;self.receipt=None;self.error=''
        self.thread=threading.Thread(target=self.run,name='truth-topic-probe',daemon=False)
        self.thread.start()

    def run(self):
        try:self.receipt=observe(self.folder)
        except Exception as exc:self.error=str(exc)

    @property
    def passed(self):return bool(self.receipt and self.receipt['passed'] and not self.error)

    def stop(self):
        self.thread.join(timeout=7)
        if self.thread.is_alive():raise RuntimeError('Truth topic probe did not stop')


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('folder',type=Path)
    args=parser.parse_args();receipt=observe(args.folder)
    print('Gazebo truth isolation:', 'PASS' if receipt['passed'] else 'FAIL')
    return 0 if receipt['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
