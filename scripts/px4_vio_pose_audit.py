"""Observe normalized SDK poses and the reset retirement boundary, without FMU writes."""
from collections import deque
import math
import time
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from uav_nav_interfaces.msg import VioStatus
from isaac_ros_visual_slam_interfaces.srv import Reset
from px4_vio_sensor_audit import stamp
from vio_pose_window import source_window


class PoseAudit:
    def __init__(self,node):
        self.node = node
        self.poses,self.statuses = deque(maxlen=5000),deque(maxlen=12000)
        self.raw = {}
        self.reset_time = None
        self.reset_ros = None
        self.future = None
        self.reset_client = node.create_client(Reset,'/uav/vio/reset')
        node.create_subscription(PoseWithCovarianceStamped,'/uav/vio/pose',self.on_pose,10)
        node.create_subscription(PoseWithCovarianceStamped,'/visual_slam/tracking/vo_pose_covariance',self.on_raw,10)
        node.create_subscription(VioStatus,'/uav/vio/pose_status',self.on_status,10)

    def on_raw(self,m):
        self.raw[stamp(m)] = m

    def on_pose(self,m):
        q,p = m.pose.pose.orientation,m.pose.pose.position
        self.poses.append(dict(mono=time.monotonic(),stamp=stamp(m),frame=m.header.frame_id,
            position=[p.x,p.y,p.z],quaternion=[q.x,q.y,q.z,q.w],covariance=list(m.pose.covariance)))

    def on_status(self,m):
        self.statuses.append(dict(mono=time.monotonic(),stamp=stamp(m),sample=m.sample_stamp.sec+m.sample_stamp.nanosec/1e9,
            valid=m.valid,reason=m.reason,session=m.localization_session,reset=m.reset_counter,calibration=m.calibration_id))

    def request_reset(self):
        if not self.reset_client.service_is_ready(): raise RuntimeError('Reset proxy not discovered')
        self.reset_time = time.monotonic()
        self.reset_ros = self.node.get_clock().now().nanoseconds/1e9
        self.future = self.reset_client.call_async(Reset.Request())

    def result(self,calibration):
        boundary = self.reset_time or time.monotonic()
        steady = [s for s in self.statuses if boundary-2 <= s['mono'] < boundary]
        ros = self.reset_ros if self.reset_ros is not None else self.node.get_clock().now().nanoseconds/1e9
        poses = source_window(self.poses,ros,boundary)
        checks = dict(steady_valid=len(steady)>=30 and all(s['valid'] for s in steady),
            calibration=bool(steady) and all(s['calibration']==calibration for s in steady),
            source_samples=bool(steady) and all(0 <= s['stamp']-s['sample'] <= .2 for s in steady),
            one_session=len({s['session'] for s in self.statuses})==1,
            original_timestamps=bool(poses) and all(p['stamp'] in self.raw for p in poses),
            pose_window=len(poses)>=100,
            writer_pose=len(self.node.get_publishers_info_by_topic('/uav/vio/pose'))==1,
            writer_status=len(self.node.get_publishers_info_by_topic('/uav/vio/pose_status'))==1)
        covariance_ok = bool(poses)
        matching_pose = bool(poses)
        for p in poses:
            c = np.asarray(p['covariance']).reshape(6,6)
            source = self.raw.get(p['stamp'])
            covariance_ok &= bool(np.isfinite(c).all() and np.allclose(c,c.T,atol=1e-8)
                and np.linalg.eigvalsh(c).min() >= -1e-8 and np.all(np.diag(c)>0) and np.all(np.diag(c)<=.25))
            if source is None:
                matching_pose = False
                continue
            raw = np.asarray(source.pose.covariance).reshape(6,6)
            covariance_ok &= abs(np.trace(c)-np.trace(raw)) < 1e-6
            pos,q = source.pose.pose.position,source.pose.pose.orientation
            matching_pose &= p['frame']=='odom' and math.dist(p['position'],[pos.x,pos.y,pos.z]) < 1e-8
            matching_pose &= abs(abs(np.dot(p['quaternion'],[q.x,q.y,q.z,q.w]))-1) < .01
        checks['covariance'] = bool(covariance_ok)
        checks['unchanged_pose'] = bool(matching_pose)
        if self.reset_time is not None:
            after = [s for s in self.statuses if s['mono'] > boundary+.5]
            checks['sdk_reset_success'] = self.future.done() and bool(self.future.result().success)
            checks['retired'] = len(after)>=20 and all(not s['valid'] and s['reset']==1
                and s['reason']=='VIO_RESET_REQUESTED' for s in after)
            checks['no_pose_after_drain'] = not any(p['mono'] > boundary+.5 for p in self.poses)
            checks['sdk_output_resumed'] = bool(after) and max(self.raw,default=0) > after[0]['stamp']+1.
        return dict(passed=all(checks.values()),checks=checks,reset_request_mono=self.reset_time,
            poses=len(self.poses),statuses=len(self.statuses),
            pose_window=dict(clock='original_sample_ros',end_ros=ros,duration_s=5.,count=len(poses),minimum_count=100),
            scope='standard SDK pose covariance and source retirement only; no velocity, EV or flight')
