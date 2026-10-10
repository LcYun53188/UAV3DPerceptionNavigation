#!/usr/bin/env python3
"""Offline ESDF evidence and ideal camera visibility; never produces a flight map."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def volume_indices(grid,point,radius,braking):
    extent=np.array([radius+braking,radius+braking,radius])
    lo=np.floor((np.asarray(point)-extent-grid['origin'])/float(grid['resolution'])).astype(int)
    hi=np.floor((np.asarray(point)+extent-grid['origin'])/float(grid['resolution'])).astype(int)
    shape=np.array(grid['distance'].shape)
    lo=np.maximum(0,lo);hi=np.minimum(shape-1,hi)
    if np.any(lo>hi):return np.empty((0,3),int)
    return np.indices(hi-lo+1).reshape(3,-1).T+lo


def visible(points,body,profile):
    """Ideal pinhole sweep upper bound: ignores occlusion and actual tracking."""
    result=np.zeros(len(points),bool)
    optical=Rotation.from_euler('xyz',[-math.pi/2-math.radians(profile['camera_pitch_deg']),0,-math.pi/2]).as_matrix()
    tangent=math.tan(profile['horizontal_fov_rad']/2)
    vertical=tangent*profile['height']/profile['width']
    mount=np.array(profile['rig_position_flu_m'])
    for yaw in np.linspace(0,math.tau,180,endpoint=False):
        body_rotation=Rotation.from_euler('z',yaw).as_matrix()
        camera=np.asarray(body)+body_rotation@mount
        p=(points-camera)@(body_rotation@optical)
        z=p[:,2]
        result|=((z>=profile['near_m']) & (z<=min(10.,profile['far_m'])) &
                 (np.abs(p[:,0])<=z*tangent) & (np.abs(p[:,1])<=z*vertical))
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('metadata',type=Path)
    parser.add_argument('--profile',type=Path,required=True)
    parser.add_argument('--flight-profile',type=Path,required=True)
    parser.add_argument('--flight-result',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    metadata=json.loads(args.metadata.read_text());grid_file=args.metadata.parent/metadata['grid_file']
    if hashlib.sha256(grid_file.read_bytes()).hexdigest()!=metadata['grid_sha256']:raise ValueError('GRID_HASH_MISMATCH')
    with np.load(grid_file,allow_pickle=False) as data:grid={k:data[k] for k in data.files}
    indices=volume_indices(grid,metadata['point_map'],metadata['radius_m'],metadata['braking_margin_m'])
    values=grid['distance'][tuple(indices.T)];observed=grid['observed'][tuple(indices.T)].astype(bool)
    points=grid['origin']+(indices+.5)*float(grid['resolution'])
    masks=dict(unknown=~observed,occupied=observed & np.isfinite(values) & (values<=0))
    report=dict(scope='offline diagnostic; ideal visibility is an upper bound, not map clearance',
        metadata=metadata,counts={k:int(np.count_nonzero(v)) for k,v in masks.items()},
        occupied_cell_centres_map_m=points[masks['occupied']].tolist())
    args.out.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(args.out/'envelope-gaps.npz',unknown=points[masks['unknown']],occupied=points[masks['occupied']])
    profile=json.loads(args.profile.read_text());flight=json.loads(args.flight_profile.read_text())
    result=json.loads(args.flight_result.read_text());home=np.array(result['home_enu'])
    unknown=points[masks['unknown']];covered=np.zeros(len(unknown),bool)
    for offset in flight['observation_offsets_enu']:covered|=visible(unknown,home+offset,profile)
    report['existing_recipe_ideal_visible_unknown']=int(covered.sum())
    candidates=[]
    from uav_mission.flight_geometry import in_region
    margin=flight['body_radius_m']+flight['tracking_margin_m']+flight['braking_margin_m']
    for x in (-.9,0.,.9):
        for y in (-.9,0.,.9):
            for z in (1.2,1.6,2.,2.4,2.8):
                point=home+np.array([x,y,z])
                if not in_region(point,flight['bounds_min'],flight['bounds_max'],margin):continue
                mask=visible(unknown,point,profile)
                candidates.append(dict(offset_enu=[x,y,z],ideal_visible_unknown=int(mask.sum()),
                    additional_geometry_visible=int(np.count_nonzero(mask & ~covered))))
    report['candidates']=sorted(candidates,key=lambda c:(c['additional_geometry_visible'],c['ideal_visible_unknown']),reverse=True)
    (args.out/'analysis.json').write_text(json.dumps(report,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,3,figsize=(13,4),constrained_layout=True)
    for ax,z in zip(axes,(1.3,2.1,2.7)):
        layer=int(round((z-float(grid['origin'][2]))/float(grid['resolution'])))
        near=indices[:,2]==layer
        actual_z=float(grid['origin'][2])+(layer+.5)*float(grid['resolution'])
        for key,colour in [('unknown','orange'),('occupied','red')]:
            selected=points[near & masks[key]]
            ax.scatter(selected[:,0],selected[:,1],s=6,c=colour,label=key)
        ax.scatter(metadata['point_map'][0],metadata['point_map'][1],marker='+',c='black',label='returned pose')
        ax.set(title=f'map voxel centres z = {actual_z:.2f} m',xlabel='map x (m)',ylabel='map y (m)',aspect='equal')
        ax.set_xlim(metadata['point_map'][0]-2.1,metadata['point_map'][0]+2.1)
        ax.set_ylim(metadata['point_map'][1]-2.1,metadata['point_map'][1]+2.1)
    axes[0].legend(loc='upper left');fig.savefig(args.out/'gap-slices.png',dpi=160);plt.close(fig)
    print(json.dumps({k:v for k,v in report.items() if k not in ('metadata','candidates')},indent=2))
    print('Top geometry candidates:',report['candidates'][:3])


if __name__=='__main__':main()
