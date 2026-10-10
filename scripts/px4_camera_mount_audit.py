"""Read-only Gazebo Scene pose audit against the advertised body optical TF."""
import hashlib
import math
import re
import numpy as np
from scipy.spatial.transform import Rotation


def block(text,start):
    opening=text.index('{',start);depth=1;end=opening+1
    while depth and end<len(text):
        if text[end]=='{':depth+=1
        elif text[end]=='}':depth-=1
        end+=1
    if depth:raise ValueError('TRUNCATED_SCENE')
    return text[opening+1:end-1]


def named(text,kind,name):
    found=[]
    for match in re.finditer(r'\b'+kind+r'\s*\{',text):
        item=block(text,match.start());value=re.search(r'\bname:\s*"([^"]+)"',item)
        if value and value[1]==name:found.append(item)
    if len(found)!=1:raise ValueError('NON_UNIQUE_SCENE_'+kind.upper())
    return found[0]


def pose(text):
    match=re.search(r'\bpose\s*\{',text)
    if not match:raise ValueError('MISSING_SCENE_POSE')
    content=block(text,match.start())
    def values(kind,axes,defaults):
        match=re.search(r'\b'+kind+r'\s*\{',content)
        if not match:raise ValueError('MISSING_SCENE_'+kind.upper())
        part=block(content,match.start());answer=[]
        for axis,default in zip(axes,defaults):
            match=re.search(r'\b'+axis+r':\s*([-+\d.eE]+)',part)
            answer.append(float(match[1]) if match else default)
        if not np.isfinite(answer).all():raise ValueError('NONFINITE_SCENE_POSE')
        return answer
    return np.array(values('position','xyz',[0,0,0])),Rotation.from_quat(values('orientation','xyzw',[0,0,0,1]))


def audit(scene,model,frames):
    try:
        m=named(scene,'model',model)
        base_position,base_rotation=pose(named(m,'link','base_link'))
        rig=named(m,'link','vio_rig_link');rig_position,rig_rotation=pose(rig)
        sensor_position,sensor_rotation=pose(named(rig,'sensor','mapping_rgbd'))
        actual_position=base_rotation.inv().apply(rig_position+rig_rotation.apply(sensor_position)-base_position)
        optical_basis=Rotation.from_matrix([[0,0,1],[-1,0,0],[0,-1,0]])
        actual_rotation=base_rotation.inv()*rig_rotation*sensor_rotation*optical_basis
        expected=frames['oakd_camera_optical_frame']
        translation_error=float(np.linalg.norm(actual_position-np.array(expected['position'])))
        rotation_error=float((Rotation.from_euler('xyz',expected['rpy']).inv()*actual_rotation).magnitude())
        return dict(passed=translation_error<=1e-6 and rotation_error<=1e-6,
            actual_mount_body_flu_m=actual_position.tolist(),advertised_mount_body_flu_m=expected['position'],
            translation_error_m=translation_error,rotation_error_rad=rotation_error,
            scene_sha256=hashlib.sha256(scene.encode()).hexdigest())
    except (ValueError,KeyError) as error:return dict(passed=False,error=str(error))
