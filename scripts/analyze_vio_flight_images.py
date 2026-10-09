#!/usr/bin/env python3
"""Offline image structure diagnostics. Candidate corners are NOT cuVSLAM inliers."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np


def analyze(folder):
    def read(name):return json.loads((folder/name).read_text())
    poses=read('normalized-poses.json');truth=read('flight-truth.json')
    result=read('result.json') if (folder/'result.json').exists() else read('ui-session.json')['result_snapshot']
    initial=result.get('flight_pose_session',{}).get('alignment',{}).get('initial_covariance')
    records=[]
    for path in sorted((folder/'flight-images').glob('*.json')):
        m=json.loads(path.read_text());stamp=m['image_stamp']
        pose=min(poses,key=lambda p:abs(p['stamp']-stamp)) if poses else None
        mono=m.get('mono',pose['mono'] if pose else None)
        state=min(truth,key=lambda p:abs(p['mono']-mono)) if mono is not None and truth else None
        metrics={}
        for side in ('left','right'):
            image=cv2.imread(str(path.with_name(path.stem+'-'+side+'.ppm')),cv2.IMREAD_GRAYSCALE)
            if image is None:continue
            points=cv2.goodFeaturesToTrack(image,maxCorners=500,qualityLevel=.01,minDistance=8,blockSize=3)
            grid=np.zeros((5,8),dtype=int)
            if points is not None:
                for x,y in points[:,0,:]:grid[min(4,int(y*5/image.shape[0])),min(7,int(x*8/image.shape[1]))]+=1
            metrics[side]=dict(gray_std=float(image.std()),laplacian_variance=float(cv2.Laplacian(image,cv2.CV_64F).var()),candidate_corners=0 if points is None else len(points),occupied_grid_cells=int(np.count_nonzero(grid)),grid=grid.tolist())
        valid_pose=pose is not None and abs(pose['stamp']-stamp)<=.08
        z=pose['covariance'][14] if valid_pose else None
        records.append(dict(image=path.stem,image_stamp=stamp,pose_stamp_difference=None if pose is None else pose['stamp']-stamp,phase=None if state is None else state['phase'],truth_height=None if state is None else state['position'][2],truth_receive_difference=None if state is None else state['mono']-mono,normalized_z_variance=z,aligned_z_bound_axis_diagnostic=None if initial is None or z is None else 2*(z+initial[2][2]),metrics=metrics))
    return dict(scope='offline image structure; no SDK tracked-feature/inlier measurements; axis bound is diagnostic, actual full Jacobian gate is in fusion evidence',detector=dict(name='Shi-Tomasi',max_corners=500,quality_level=.01,min_distance_px=8,grid=[8,5]),records=records)


def main():
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    a.output.write_text(json.dumps(analyze(a.folder),indent=2)+'\n')

if __name__=='__main__':main()
