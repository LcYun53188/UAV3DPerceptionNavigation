#!/usr/bin/env python3
"""Passive Gazebo truth recorder for course audits; publishes no control/TF data."""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--model', required=True)
    args = parser.parse_args()
    process = subprocess.Popen(['gz', 'topic', '-e', '-t', '/world/default/pose/info', '--json-output'],
                               stdout=subprocess.PIPE, text=True)
    def stop(_signum, _frame):
        process.terminate()
    signal.signal(signal.SIGTERM, stop); signal.signal(signal.SIGINT, stop)
    previous = -1.
    try:
        with (args.directory/'course-truth.jsonl').open('x', buffering=1) as output:
            for line in process.stdout:
                try:
                    message = json.loads(line); stamp = message['header']['stamp']
                    source = float(stamp.get('sec', 0))+float(stamp.get('nsec', 0))/1e9
                    if 0 <= source-previous < .045: continue  # ~20 Hz at original simulation time.
                    for pose in message.get('pose', []):
                        if pose.get('name') != args.model: continue
                        p = pose['position']
                        output.write(json.dumps(dict(mono=time.monotonic(), source_sim_s=source,
                            model=args.model, position=[p.get(k, 0.) for k in ('x','y','z')]))+'\n')
                        previous = source
                except (ValueError, KeyError, TypeError):
                    continue
        return process.wait(timeout=5)
    finally:
        if process.poll() is None:
            process.terminate()
            try: process.wait(timeout=3)
            except subprocess.TimeoutExpired: process.kill(); process.wait(timeout=3)


if __name__ == '__main__':
    raise SystemExit(main())
