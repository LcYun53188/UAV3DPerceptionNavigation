#!/usr/bin/env python3
"""Separate SITL telemetry build using the pinned upstream DDS generator.

No upstream source edits; no change to the locked W0 build. Always generate the
expanded header after initializing the upstream topic target, then verify that
CMake/Ninja did not replace it during compilation.
"""
import argparse
import os
from pathlib import Path
import subprocess
from sim_validation import ROOT, read, file_hash, write_json
from prepare_px4_sim import ensure_source, check_external

BUILD = ROOT / '.deps/px4-vio-build'
BUILD_ENV = dict(os.environ)
BUILD_ENV.pop('PYTHONPATH', None)
BUILD_ENV.pop('VIRTUAL_ENV', None)
BUILD_ENV['PATH'] = str(ROOT/'.deps/px4-venv/bin') + ':' + BUILD_ENV.get('PATH', '')
CONFIG = ROOT / 'simulation/px4/vio/dds_topics.yaml'
ADDED = dict(estimator_selector_status='EstimatorSelectorStatus',
             estimator_aid_src_ev_pos='EstimatorAidSource2d',
             estimator_aid_src_ev_hgt='EstimatorAidSource1d',
             estimator_aid_src_ev_vel='EstimatorAidSource3d',
             estimator_aid_src_ev_yaw='EstimatorAidSource1d')


def validate_topics(base, extended):
    if set(base) != set(extended):
        raise ValueError('DDS sections changed')
    for key in base:
        if key != 'publications' and base[key] != extended[key]:
            raise ValueError('DDS subscriptions changed')
    pubs = extended['publications']
    original = base['publications']
    expected = [dict(topic='/fmu/out/'+name,type='px4_msgs::msg::'+kind) for name,kind in ADDED.items()]
    if pubs != original+expected or len({p['topic'] for p in pubs}) != len(pubs):
        raise ValueError('Expected baseline publications plus exactly five read-only topics')


def inputs(lock):
    source = ROOT / lock['sitl']['path']
    files = [ROOT/'simulation/px4/versions.lock.yaml', CONFIG, Path(__file__),
             ROOT/'requirements/px4-sim.txt', source/'src/modules/uxrce_dds_client/generate_dds_topics.py',
             source/'src/modules/uxrce_dds_client/dds_topics.h.em',
             source/'src/modules/uxrce_dds_client/dds_topics.yaml']
    return {str(p.relative_to(ROOT)):file_hash(p) for p in files}


def generate(source, output):
    subprocess.run([ROOT/'.deps/px4-venv/bin/python',
        source/'src/modules/uxrce_dds_client/generate_dds_topics.py',
        '--dds-topics-file', CONFIG, '--template_file',
        source/'src/modules/uxrce_dds_client/dds_topics.h.em', '--client-outdir',output],check=True,cwd=ROOT,env=BUILD_ENV)


def verify(lock):
    manifest = read(BUILD/'vio-build.json')
    if manifest['inputs'] != inputs(lock):
        raise RuntimeError('VIO build inputs changed; rebuild explicitly')
    for name,digest in manifest['artifacts'].items():
        if file_hash(ROOT/name) != digest:
            raise RuntimeError('VIO build artifact drift: '+name)
    return manifest


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--jobs',type=int,default=4)
    parser.add_argument('--check',action='store_true')
    args=parser.parse_args()
    if not 1 <= args.jobs <= 16: parser.error('jobs must be within [1,16]')
    lock=read(ROOT/'simulation/px4/versions.lock.yaml')
    for name in ('sitl','agent','px4_msgs'): ensure_source(lock[name])
    check_external(lock)
    source=ROOT/lock['sitl']['path']
    validate_topics(read(source/'src/modules/uxrce_dds_client/dds_topics.yaml'),read(CONFIG))
    baseline={p['path']:file_hash(ROOT/p['path']) for p in lock['build_artifacts']}
    if any(baseline[p['path']] != p['sha256'] for p in lock['build_artifacts']):
        raise RuntimeError('Original W0/Agent artifact drift')
    if args.check:
        verify(lock)
        print('Separate VIO telemetry build inputs/artifacts verified.')
        return
    subprocess.run(['cmake','-S',source,'-B',BUILD,'-G','Ninja','-DCONFIG=px4_sitl_default',
                    '-DPYTHON_EXECUTABLE='+str(ROOT/'.deps/px4-venv/bin/python')],check=True,cwd=ROOT,env=BUILD_ENV)
    subprocess.run(['cmake','--build',BUILD,'--target','topic_bridge_files','--parallel',str(args.jobs)],check=True,env=BUILD_ENV)
    client=BUILD/'src/modules/uxrce_dds_client'
    generate(source,client)
    expected=file_hash(client/'dds_topics.h')
    subprocess.run(['cmake','--build',BUILD,'--parallel',str(args.jobs)],check=True,env=BUILD_ENV)
    if file_hash(client/'dds_topics.h') != expected:
        raise RuntimeError('Build replaced expanded DDS header; do not use artifact')
    if any(file_hash(ROOT/name) != digest for name,digest in baseline.items()):
        raise RuntimeError('Original W0/Agent artifacts changed during separate build')
    artifacts=[BUILD/'bin/px4',client/'dds_topics.h']
    artifacts.extend(sorted((BUILD/'src/modules/simulation/gz_plugins').glob('*.so')))
    write_json(BUILD/'vio-build.json',dict(scope='separate read-only telemetry SITL build; not VIO flight acceptance',
        px4_commit=lock['sitl']['commit'],inputs=inputs(lock),
        artifacts={str(p.relative_to(ROOT)):file_hash(p) for p in artifacts},
        added_publications=ADDED,baseline_artifacts=baseline))
    verify(lock)
    print('Separate VIO telemetry build complete:',BUILD)


if __name__=='__main__': main()
