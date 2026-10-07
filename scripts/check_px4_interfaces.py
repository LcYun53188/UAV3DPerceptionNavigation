#!/usr/bin/env python3
"""Compare every DDS-exported message and its nested types with pinned px4_msgs."""
import argparse
import re
from pathlib import Path

from sim_validation import ROOT, file_hash, read, write_json


def declarations(path):
    return [''.join(line.split('#', 1)[0].split()) for line in Path(path).read_text().splitlines()
            if line.split('#', 1)[0].strip()]


def compare(px4, messages):
    topic_path = px4 / 'src/modules/uxrce_dds_client/dds_topics.yaml'
    topics = read(topic_path)
    exported = [entry for entries in topics.values() for entry in (entries or [])]
    pending = {e['type'].rsplit('::', 1)[-1] for e in exported}
    checked = {}
    while pending:
        name = pending.pop()
        if name in checked:
            continue
        source = next((p for p in (px4 / 'msg/versioned' / f'{name}.msg', px4 / 'msg' / f'{name}.msg')
                       if p.is_file()), None)
        target = messages / 'msg' / f'{name}.msg'
        equal = source is not None and target.is_file() and declarations(source) == declarations(target)
        checked[name] = dict(status='PASS' if equal else 'FAIL',
                             px4_sha256=file_hash(source) if source else None,
                             ros_sha256=file_hash(target) if target.is_file() else None)
        if source:
            for line in source.read_text().splitlines():
                fields = line.split('#', 1)[0].split()
                if len(fields) < 2:
                    continue
                nested = fields[0].split('[')[0]
                if re.fullmatch(r'[A-Z][A-Za-z0-9_]*', nested):
                    pending.add(nested)
    return dict(status='PASS' if all(v['status'] == 'PASS' for v in checked.values()) else 'FAIL',
                scope='ordered message declarations/constants and nested types; comments/whitespace excluded',
                dds_topics_sha256=file_hash(topic_path), topics=exported, messages=checked)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    lock = read(ROOT / 'simulation/px4/versions.lock.yaml')
    result = compare(ROOT / lock['sitl']['path'], ROOT / lock['px4_msgs']['path'])
    if args.output:
        write_json(args.output, result)
    print(f"{result['status']}: {len(result['messages'])} messages, {len(result['topics'])} DDS topic declarations")
    for name, value in result['messages'].items():
        if value['status'] != 'PASS':
            print(f'FAIL {name}')
    return 0 if result['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
