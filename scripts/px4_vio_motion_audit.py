"""Independent timestamp-matched VIO error audit; truth is never fed to VIO."""
import math
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data
from px4_vio_sensor_audit import stamp


def rotation(q):
    q = np.asarray(q,dtype=float)
    q /= np.linalg.norm(q)
    x,y,z,w = q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def assess_motion(truth,poses,imu):
    checks = dict(truth_present=len(truth)>100,pose_present=len(poses)>100,imu_present=len(imu)>100)
    if not all(checks.values()): return dict(passed=False,checks=checks)
    ts = np.array([r['stamp'] for r in truth])
    positions = np.array([r['position'] for r in truth])
    checks['ordered_truth'] = bool(np.all(np.diff(ts)>0))
    if not checks['ordered_truth']: return dict(passed=False,checks=checks)
    # Match only bracketed samples, no extrapolation or trajectory fitting.
    matched = [p for p in poses if ts[0] <= p['stamp'] <= ts[-1]]
    checks['paired_samples'] = len(matched)>=100
    if not matched: return dict(passed=False,checks=checks)
    first = max(0,int(np.searchsorted(ts,matched[0]['stamp'],side='right')-1))
    last = min(len(ts)-1,int(np.searchsorted(ts,matched[-1]['stamp'],side='right')))
    checks['truth_rate'] = bool(np.max(np.diff(ts[first:last+1])) <= .032)
    observed = positions[first:last+1]
    def reference(p):
        t = p['stamp']
        i = int(np.clip(np.searchsorted(ts,t,side='right')-1,0,len(ts)-2))
        f = (t-ts[i])/(ts[i+1]-ts[i])
        q0,q1 = np.array(truth[i]['quaternion']),np.array(truth[i+1]['quaternion'])
        if np.dot(q0,q1)<0: q1 = -q1
        return (1-f)*positions[i]+f*positions[i+1],rotation((1-f)*q0+f*q1)
    # One initial rigid alignment, fixed throughout. No scale or best-fit correction.
    initial = matched[0]
    p0,r0 = reference(initial)
    align = r0@rotation(initial['quaternion']).T
    offset = p0-align@np.array(initial['position'])
    errors,angles = [],[]
    for p in matched:
        xyz,r = reference(p)
        errors.append(float(np.linalg.norm(align@np.array(p['position'])+offset-xyz)))
        relative = r.T@align@rotation(p['quaternion'])
        angles.append(math.acos(float(np.clip((np.trace(relative)-1)/2,-1,1))))
    extent = np.ptp(observed,axis=0)
    yaws = np.unwrap([math.atan2(rotation(r['quaternion'])[1,0],rotation(r['quaternion'])[0,0]) for r in truth[first:last+1]])
    checks['three_axis_excitation'] = bool(np.all(extent>=[.8,.6,.4]))
    checks['yaw_excitation'] = float(np.ptp(yaws))>=.6
    checks['motion_duration'] = matched[-1]['stamp']-max(15.,matched[0]['stamp']) >= 30.
    checks['source_covers_end'] = 0 <= ts[-1]-matched[-1]['stamp'] <= .2
    checks['source_continuous'] = bool(np.max(np.diff([p['stamp'] for p in matched])) <= .081)
    rmse = float(np.sqrt(np.mean(np.square(errors))))
    checks['position_rmse'] = rmse <= .15
    checks['position_max'] = max(errors) <= .30
    checks['orientation_max'] = max(angles) <= math.radians(10.)
    values = np.asarray([r['values'] for r in imu if r['stamp']>=15.])
    deviations = np.std(values,axis=0) if len(values) else np.zeros(6)
    checks['imu_acceleration_response'] = bool(np.all(deviations[:3]>.03))
    checks['imu_rotation_response'] = deviations[5]>.03
    return dict(passed=all(checks.values()),checks={k:bool(v) for k,v in checks.items()},
        paired_samples=len(matched),alignment_stamp=initial['stamp'],alignment_rotation=align.tolist(),
        alignment_translation=offset.tolist(),position_rmse_m=rmse,position_max_m=max(errors),
        orientation_max_deg=math.degrees(max(angles)),truth_extent_m=extent.tolist(),
        yaw_extent_rad=float(np.ptp(yaws)),imu_std=deviations.tolist(),
        truth_audit_interval=[float(ts[first]),float(ts[last])],
        startup_truth_max_gap_s=float(np.max(np.diff(ts[:first+1]))) if first else None,
        last_truth_stamp=float(ts[-1]),last_pose_stamp=matched[-1]['stamp'],
        scope='reference fixture motion only; one initial rigid alignment; no PX4 EV or flight acceptance')


class MotionAudit:
    def __init__(self,node):
        self.truth,self.imu = [],[]
        self.node = node
        node.create_subscription(Odometry,'/vio/truth',self.on_truth,qos_profile_sensor_data)
        node.create_subscription(Imu,'/vio/imu',self.on_imu,qos_profile_sensor_data)

    def on_truth(self,m):
        p,q = m.pose.pose.position,m.pose.pose.orientation
        self.truth.append(dict(stamp=stamp(m),frame=m.header.frame_id,child=m.child_frame_id,
            position=[p.x,p.y,p.z],quaternion=[q.x,q.y,q.z,q.w]))

    def on_imu(self,m):
        a,w = m.linear_acceleration,m.angular_velocity
        self.imu.append(dict(stamp=stamp(m),values=[a.x,a.y,a.z,w.x,w.y,w.z]))

    def result(self,poses):
        result = assess_motion(self.truth,poses,self.imu)
        result['checks']['unique_truth_writer'] = len(self.node.get_publishers_info_by_topic('/vio/truth'))==1
        result['checks']['truth_frames'] = bool(self.truth) and all(
            r['frame']=='world' and r['child']=='base_link' for r in self.truth)
        result['checks']['truth_observer_only'] = all(
            endpoint.node_name=='vio_sensor_observer'
            for endpoint in self.node.get_subscriptions_info_by_topic('/vio/truth'))
        result['passed'] = all(result['checks'].values())
        return result
