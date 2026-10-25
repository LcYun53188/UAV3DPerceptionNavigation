#!/usr/bin/env python3
"""Check ordered waypoints, directed apertures and swept native Gazebo truth."""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from simulation.vio.runtime.obstacle_course import (
    clearance, obstacles, segment_points, waypoints,
)


def directed_crossing(a, b, zone, radius):
    center, normal = zone['center'], zone['normal']
    da = sum((x-y)*n for x, y, n in zip(a, center, normal))
    db = sum((x-y)*n for x, y, n in zip(b, center, normal))
    if not da < 0 <= db: return False
    t = -da/(db-da); point = [x+(y-x)*t for x,y in zip(a,b)]
    if zone['id'] == 'rectangle':
        return (abs(point[1]-center[1])+radius < zone['opening_m'][0]/2 and
                abs(point[2]-center[2])+radius < zone['opening_m'][1]/2)
    return math.hypot(point[0]-center[0], point[2]-center[2])+radius < zone['inner_radius_m']


def evaluate(layout, task, rows, *, origin=(0.,0.,0.), waypoint_tolerance=.3):
    """Geometry audit only. A caller must separately require actual mission success."""
    rows = list(rows); checks = dict(samples_present=len(rows) >= 2)
    if not checks['samples_present']:
        return dict(passed=False, checks=checks, reason='MISSING_NATIVE_TRUTH')
    try:
        if any(len(r['position']) != 3 or not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in
               [*r['position'], r['source_sim_s'], r['mono']]) for r in rows):
            raise ValueError('Invalid truth sample')
        checks['continuous_native_time'] = all(0 < b['source_sim_s']-a['source_sim_s'] <= .3 and
            0 < b['mono']-a['mono'] <= .5 for a,b in zip(rows,rows[1:]))
        checks['no_pose_jump'] = all(math.dist(a['position'],b['position']) <=
            .9*(b['source_sim_s']-a['source_sim_s'])+.02 for a,b in zip(rows,rows[1:]))
    except (KeyError, TypeError, ValueError):
        return dict(passed=False, checks=checks, reason='INVALID_NATIVE_TRUTH')
    points = [r['position'] for r in rows]
    targets = [[a+b for a,b in zip(p,origin)] for p in waypoints(layout,task)]
    cursor = 0; hits = []
    for target in targets:
        hit = next((i for i in range(cursor,len(points)) if math.dist(target,points[i]) <= waypoint_tolerance), None)
        if hit is None: break
        hits.append(hit); cursor = hit+1
    checks['ordered_waypoints'] = len(hits) == len(targets)
    radius = layout['body_radius_m']+layout['tracking_margin_m']
    solids = obstacles(layout)
    swept = (p for a,b in zip(points,points[1:]) for p in segment_points(a,b))
    minimum = min(clearance(p,o) for p in swept for o in solids)
    checks['swept_body_clear'] = minimum >= radius
    gates = {}; offset = 0
    zones = layout['zones'] if task == 'all' else [z for z in layout['zones'] if z['id'] == task]
    for zone in zones:
        if zone['id'] in ('rectangle','ring'):
            # Bound this crossing by the entrance/exit waypoint hits; a previous
            # manual pass or a later RETURN cannot satisfy the gate task.
            if len(hits) > offset+2:
                start,end = hits[offset],hits[offset+2]
                gates[zone['id']] = any(directed_crossing(points[i],points[i+1],zone,radius)
                                       for i in range(start,end))
            else: gates[zone['id']] = False
        offset += len(zone['waypoints_enu'])
    checks['directed_apertures'] = all(gates.values())
    return dict(passed=all(checks.values()), checks=checks, gates=gates,
                waypoint_hits=len(hits), expected_waypoints=len(targets), min_clearance_m=minimum,
                scope='native Gazebo geometry evidence; mission/source success checked separately')


def assess(directory):
    directory = Path(directory)
    evidence = json.loads((directory/'course-flight.json').read_text())
    manifest = json.loads((directory/'manifest.json').read_text())
    receipt = json.loads((directory/'assets/warehouse-layout.json').read_text())
    layout = receipt['layout']; rows = []
    with (directory/'course-truth.jsonl').open() as stream:
        for line in stream:
            try: row = json.loads(line)
            except ValueError: continue  # A still-running recorder may have one incomplete last line.
            if evidence['start_mono'] <= row['mono'] <= evidence['end_mono']:
                rows.append(row)
    result = evaluate(layout, manifest['obstacle_course'], rows, origin=evidence['home_truth'])
    mission_ok = (evidence['exit_code'] == 0 and evidence['result'] == ['SUCCEEDED','LANDED_AND_DISARMED',True]
                  and evidence['landed'] is True and evidence['arming'] == 1
                  and evidence['navigation_backend'] == 'EGO')
    binding = (evidence['run_id'] == manifest['run_id'] and layout['scene_id'] == 'uav_obstacle_course_v2'
               and all(r.get('model') == manifest['model'] for r in rows))
    coverage = bool(rows and rows[0]['mono']-evidence['start_mono'] <= .5 and
                    evidence['end_mono']-rows[-1]['mono'] <= .5)
    frozen = json.loads((directory/'flight-assets.json').read_text())
    for name in ('assets/default.sdf','assets/warehouse-layout.json'):
        binding &= frozen['files'].get(name) == hashlib.sha256((directory/name).read_bytes()).hexdigest()
    result['checks'].update(mission_completed=mission_ok, owned_assets=binding, whole_task_recorded=coverage)
    result['passed'] = all(result['checks'].values())
    result['task'] = manifest['obstacle_course']
    result['scope'] = 'owned simulated course only; VIO qualification and hardware flight remain separate'
    return result


def save_pocket_result(directory, node, manifest, exit_code):
    """Called after root termination; never modifies the flight result or controller."""
    directory = Path(directory)
    try:
        # Geometry work must not block the sole gateway during an airborne
        # cancel/HOLD fallback; those roots fail immediately without replay.
        if not (node.samples['vehicle_land_detected'].landed and node.samples['vehicle_status'].arming_state == 1):
            raise ValueError('COURSE_AUDIT_REQUIRES_LANDED_DISARMED')
        events = node.events
        started = next(e['mono'] for e in events if e['phase'] == 'TAKEOFF')
        ended = events[-1]['mono']
        first = None
        with (directory/'course-truth.jsonl').open() as stream:
            for line in stream:
                row = json.loads(line)
                if row['mono'] >= started:
                    first = row; break
        if first is None: raise ValueError('Missing launch truth')
        # At TAKEOFF entry the aircraft is still at its launch support plane.
        evidence = dict(run_id=manifest['run_id'], start_mono=started, end_mono=ended,
            home_truth=first['position'], exit_code=exit_code, result=node.result,
            landed=node.samples['vehicle_land_detected'].landed,
            arming=node.samples['vehicle_status'].arming_state, navigation_backend=node.navigation_backend)
        (directory/'course-flight.json').write_text(json.dumps(evidence,indent=2)+'\n')
        result = assess(directory)
    except (OSError, KeyError, TypeError, ValueError, StopIteration) as exc:
        result = dict(passed=False, reason=str(exc), scope='missing or invalid course evidence')
    try:
        (directory/'course-assessment.json').write_text(json.dumps(result,indent=2)+'\n')
    except OSError as exc:
        result = dict(passed=False, reason='COURSE_EVIDENCE_WRITE_FAILED:'+str(exc))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument('directory',type=Path)
    args = parser.parse_args(); result = assess(args.directory)
    (args.directory/'course-assessment.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2)); return 0 if result['passed'] else 1


def save_automated_result(directory, manifest):
    directory = Path(directory)
    try:
        flight = json.loads((directory/'flight-observation.json').read_text())
        events = json.loads((directory/'flight-events.json').read_text())
        rows = json.loads((directory/'flight-truth.json').read_text())
        with (directory/'course-truth.jsonl').open('w') as stream:
            for row in rows:
                stream.write(json.dumps(dict(row, model=manifest['model']))+'\n')
        native = flight['result']
        evidence = dict(run_id=manifest['run_id'],
            start_mono=next(e['mono'] for e in events if e['phase']=='TAKEOFF'), end_mono=events[-1]['mono'],
            home_truth=flight['home_truth'], exit_code=0 if flight.get('passed') else 1,
            result=['SUCCEEDED' if flight['action_status']==4 else 'ABORTED',native['reason'],native['cleanup_confirmed']],
            landed=flight['final_land'], arming=flight['final_arming'], navigation_backend=flight['navigation_backend'])
        (directory/'course-flight.json').write_text(json.dumps(evidence,indent=2)+'\n')
        result = assess(directory)
    except (OSError, KeyError, TypeError, ValueError, StopIteration) as exc:
        result = dict(passed=False, reason=str(exc))
    (directory/'course-assessment.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


if __name__ == '__main__':
    raise SystemExit(main())
