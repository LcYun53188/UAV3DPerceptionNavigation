#!/usr/bin/env python3
"""Read-only collision screening of an actual flight's frozen depth map."""

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
import numpy as np
from analyze_vio_chain_flight import read
from uav_nav_sim.core import Grid
from uav_mission.ego_execution import BrakingGrid


def analyze(folder):
    metadata, metadata_sha = read(folder, 'observation-grid-returned.json')
    path = folder / metadata['grid_file']
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if digest != metadata['grid_sha256']:
        raise ValueError('Frozen grid hash mismatch')
    with np.load(path, allow_pickle=False) as data:
        grid = Grid(data['origin'], float(data['resolution']),
                    data['distance'], data['observed'])
    gate = BrakingGrid(grid, metadata['braking_margin_m'])
    radius = metadata['radius_m']
    home = metadata['reference_map']
    occupied = np.argwhere(grid.observed & np.isfinite(grid.distance) & (grid.distance <= 0))
    unknown = np.argwhere(~grid.observed)
    def evidence(indices):
        if not len(indices):
            return None
        # Select an actual voxel near flight altitude, not a synthetic obstacle.
        centres = grid.origin + (indices + .5) * grid.resolution
        point = centres[np.argmin(np.abs(centres[:, 2] - home[2]))]
        return dict(point_map=point.tolist(), rejected=bool(gate.collision(point, radius)))
    trace, trace_sha = read(folder, 'flight-trace.json')
    navigation = [r for r in trace if r['phase'] in ('NAVIGATE', 'RETURN')]
    rejected = sum(gate.collision(r['position_enu'], radius) for r in navigation)
    report = dict(schema=1, run_id=folder.name,
        scope='offline actual ESDF collision screening; returned-map snapshot only; does not prove obstacle injection, live replanning or a flown detour',
        map_id=metadata['map_id'], map_epoch=metadata['map_epoch'],
        map_version=metadata['map_version'], localization_session=metadata['localization_session'],
        resolution_m=grid.resolution, body_tracking_voxel_radius_m=radius,
        horizontal_braking_margin_m=metadata['braking_margin_m'],
        observed_voxels=int(np.count_nonzero(grid.observed)),
        occupied_voxels=len(occupied), unknown_voxels=len(unknown),
        home_clear=not gate.collision(home, radius),
        real_occupied_voxel=evidence(occupied), real_unknown_voxel=evidence(unknown),
        flown_navigation_samples=len(navigation),
        flown_navigation_rejected_by_returned_snapshot=int(rejected),
        active_obstacle_detour_tested=False,
        input_sha256={'grid':digest,'metadata':metadata_sha,'trace':trace_sha},
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (folder / 'map-collision-screening.json').write_text(json.dumps(report, indent=2)+'\n')
    snapshots = folder / 'analysis-inputs'
    snapshots.mkdir(exist_ok=True)
    (snapshots / Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    analyze(parser.parse_args().run)
