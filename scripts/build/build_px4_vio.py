#!/usr/bin/env python3
"""Separate SITL telemetry build using the pinned upstream DDS generator.

Managed EV yaw noise unit correction in an isolated source; locked W0 preserved. Always generate the
expanded header after initializing the upstream topic target, then verify that
CMake/Ninja did not replace it during compilation.
"""
import argparse
import os
from pathlib import Path
import subprocess
import shutil
import sys

# Canonical execution also needs the shared scripts directory on the import path.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sim_validation import ROOT, read, file_hash, write_json
from prepare_px4_sim import ensure_source, check_external

BUILD = ROOT / '.deps/px4-vio-sdk16-build'
SOURCE = ROOT / '.deps/PX4-VIO-SDK16'
PATCH = ROOT / 'simulation/px4/vio/ev_yaw_noise_units.patch'
STATUS_PATCH = ROOT / 'simulation/px4/vio/flight_status_heartbeat.patch'
COMMANDER = Path('src/modules/commander/Commander.cpp')
LAND = Path('src/modules/land_detector/LandDetector.cpp')
YAW = Path('src/modules/ekf2/EKF/aid_sources/external_vision/ev_yaw_control.cpp')
BUILD_ENV = dict(os.environ)
BUILD_ENV.pop('PYTHONPATH', None)
BUILD_ENV.pop('VIRTUAL_ENV', None)
BUILD_ENV['PATH'] = str(ROOT/'.deps/px4-venv/bin') + ':' + BUILD_ENV.get('PATH', '')
CONFIG = ROOT / 'simulation/px4/vio/dds_topics.yaml'
ADDED = dict(estimator_selector_status='EstimatorSelectorStatus',
             estimator_aid_src_ev_pos='EstimatorAidSource2d',
             estimator_aid_src_ev_hgt='EstimatorAidSource1d',
             estimator_aid_src_ev_vel='EstimatorAidSource3d',
             estimator_aid_src_ev_yaw='EstimatorAidSource1d',
             estimator_aid_src_optical_flow='EstimatorAidSource2d',
             estimator_aid_src_rng_hgt='EstimatorAidSource1d')


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
        raise ValueError('Expected baseline publications plus exactly the declared read-only topics')


def inputs(lock):
    source = ROOT / lock['sitl']['path']
    files = [ROOT/'simulation/px4/versions.lock.yaml', CONFIG, Path(__file__).resolve(),
             ROOT/'requirements/px4-sim.txt', PATCH, STATUS_PATCH, source/YAW, source/COMMANDER, source/LAND, source/'src/modules/uxrce_dds_client/generate_dds_topics.py',
             source/'src/modules/uxrce_dds_client/dds_topics.h.em',
             source/'src/modules/uxrce_dds_client/dds_topics.yaml']
    return {str(p.relative_to(ROOT)):file_hash(p) for p in files}


def generate(source, output):
    subprocess.run([ROOT/'.deps/px4-venv/bin/python',
        source/'src/modules/uxrce_dds_client/generate_dds_topics.py',
        '--dds-topics-file', CONFIG, '--template_file',
        source/'src/modules/uxrce_dds_client/dds_topics.h.em', '--client-outdir',output],check=True,cwd=ROOT,env=BUILD_ENV)


def patched_source(lock):
    original = ROOT / lock['sitl']['path']
    if not SOURCE.exists():
        # Independent files, including Git/module metadata; never hardlink the
        # editable source or Git index to the locked checkout.
        shutil.copytree(original, SOURCE, symlinks=True,
            ignore=lambda directory, names: ['build'] if Path(directory) == original else [])
    for patch in (PATCH,STATUS_PATCH):
        applied=subprocess.run(['git','apply','--reverse','--check',str(patch)],cwd=SOURCE,
                               stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode==0
        if not applied:
            subprocess.run(['git','apply','--check',str(patch)],cwd=SOURCE,check=True)
            subprocess.run(['git','apply',str(patch)],cwd=SOURCE,check=True)
    verify_source(lock)
    return SOURCE


def verify_source(lock):
    head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=SOURCE, text=True).strip()
    if head != lock['sitl']['commit']:
        raise RuntimeError('Patched PX4 source base commit drift')
    changed = subprocess.check_output(['git', 'diff', '--name-only', 'HEAD'], cwd=SOURCE, text=True).splitlines()
    if changed != sorted(map(str,(YAW,COMMANDER,LAND))):
        raise RuntimeError('Unexpected patched PX4 source edits: '+str(changed))
    original = ROOT / lock['sitl']['path']
    upstream = (original/YAW).read_text()
    expected = upstream.replace('ev_sample.orientation_var(2), _params.ev_att_noise,',
                                'ev_sample.orientation_var(2), sq(_params.ev_att_noise),')
    if expected == upstream or (SOURCE/YAW).read_text() != expected:
        raise RuntimeError('PX4 EV yaw patch content drift')
    commander=(original/COMMANDER).read_text()
    expected=commander.replace('\twhile (!should_exit()) {',
        '\thrt_abstime last_status_heartbeat = 0;\n\n\twhile (!should_exit()) {')
    heartbeat='\t\t// Independent simulation telemetry heartbeat: preserve the 2 Hz\n\t\t// armed/control-mode publication schedule and its original timestamp.\n\t\tif (now >= last_status_heartbeat + 100_ms) {\n\t\t\tvehicle_status_s status_heartbeat = _vehicle_status;\n\t\t\tstatus_heartbeat.timestamp = now;\n\t\t\t_vehicle_status_pub.publish(status_heartbeat);\n\t\t\tlast_status_heartbeat = now;\n\t\t}\n\n'
    expected=expected.replace('\t\tcheckWorkerThread();',heartbeat+'\t\tcheckWorkerThread();')
    if expected==commander or (SOURCE/COMMANDER).read_text()!=expected:
        raise RuntimeError('PX4 status heartbeat patch content drift')
    land=(original/LAND).read_text()
    expected=land.replace('// publish at 1 Hz, very first time, or when the result has changed',
        '// publish simulation ground telemetry at 4 Hz, first time, or when changed').replace(
        'hrt_elapsed_time(&_land_detected.timestamp) >= 1_s',
        'hrt_elapsed_time(&_land_detected.timestamp) >= 250_ms')
    if expected==land or (SOURCE/LAND).read_text()!=expected:
        raise RuntimeError('PX4 land heartbeat patch content drift')
    for patch in (PATCH,STATUS_PATCH):
        subprocess.run(['git','apply','--reverse','--check',str(patch)],cwd=SOURCE,check=True)



def verify(lock):
    verify_source(lock)
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
    original=ROOT/lock['sitl']['path']
    validate_topics(read(original/'src/modules/uxrce_dds_client/dds_topics.yaml'),read(CONFIG))
    baseline={p['path']:file_hash(ROOT/p['path']) for p in lock['build_artifacts']}
    if any(baseline[p['path']] != p['sha256'] for p in lock['build_artifacts']):
        raise RuntimeError('Original W0/Agent artifact drift')
    if args.check:
        verify(lock)
        print('Separate VIO telemetry build inputs/artifacts verified.')
        return
    source=patched_source(lock)
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
    write_json(BUILD/'vio-build.json',dict(scope='isolated SDK16 SITL EV yaw noise units patch and read-only telemetry; not flight acceptance',
        patched_source=str(SOURCE.relative_to(ROOT)),patch_sha256=file_hash(PATCH),status_patch_sha256=file_hash(STATUS_PATCH),yaw_source_sha256=file_hash(SOURCE/YAW),
        px4_commit=lock['sitl']['commit'],inputs=inputs(lock),
        artifacts={str(p.relative_to(ROOT)):file_hash(p) for p in artifacts},
        added_publications=ADDED,baseline_artifacts=baseline))
    verify(lock)
    print('Separate VIO telemetry build complete:',BUILD)


if __name__=='__main__': main()
