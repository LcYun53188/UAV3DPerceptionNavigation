"""Independent chronology must retain real drift after one initial alignment."""
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from diagnose_vio_motion import diagnose


def test_error_is_not_removed_by_trajectory_fitting(tmp_path):
    truth=[dict(stamp=float(i),position=[i,0,0],quaternion=[0,0,0,1]) for i in range(21)]
    raw=[dict(stamp=float(i),position=[i,.01*i,0],quaternion=[0,0,0,1],
              covariance=[.1 if k%7==0 else 0 for k in range(36)]) for i in range(21)]
    # Start at the original anchor; offset is zero, later transverse drift remains.
    norm=[dict(stamp=0.)]
    expected=(sum((i*.01)**2 for i in range(21))/21)**.5
    for name,value in [('truth',truth),('sdk-poses',raw),('normalized-poses',norm),
                       ('result',dict(motion_raw_sdk_diagnostic=dict(position_rmse_m=expected)))]:
        (tmp_path/(name+'.json')).write_text(json.dumps(value))
    report=diagnose(tmp_path)
    assert report['metrics_match'] and abs(report['per_axis_rmse_m'][1]-expected)<1e-12
    assert report['samples'][-1]['error_enu_m']==[0.,.2,0.]
    assert report['samples'][0]['normalized_present'] and not report['samples'][-1]['normalized_present']
    assert report['samples'][-1]['world_variances']==[.1]*6


def test_imu_residual_keeps_axis_sign_error_visible():
    from diagnose_vio_motion import imu_residual
    truth=[dict(stamp=i*.1,position=[.5*(i*.1)**2,0,0],quaternion=[0,0,0,1]) for i in range(401)]
    imu=[dict(stamp=i*.1,values=[1,0,9.8,0,0,0]) for i in range(401)]
    good=imu_residual(truth,imu,[.12,0,.242])
    assert max(good['rms_mps2'])<1e-9
    for sample in imu:sample['values'][0]=-1
    bad=imu_residual(truth,imu,[.12,0,.242])
    assert abs(bad['rms_mps2'][0]-2)<1e-9
