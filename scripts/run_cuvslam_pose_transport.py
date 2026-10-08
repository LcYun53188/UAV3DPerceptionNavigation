#!/usr/bin/env python3
"""Synthetic DDS tests of pose warmup and reset retirement, never real VIO/flight."""
import argparse
import json
import os
from pathlib import Path
import time
import uuid
from collections import deque
os.environ['ROS_DOMAIN_ID'] = '93'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseWithCovarianceStamped
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from isaac_ros_visual_slam_interfaces.srv import Reset
from uav_nav_interfaces.msg import VioStatus
from px4_comm_bridge.cuvslam_pose import SOURCE_PARAMETERS, CONTRACT
from px4_comm_bridge.cuvslam_pose_node import CuvslamPose


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',choices=['success','failure','missing','timeout'],default='success')
    case = parser.parse_args().case
    out = Path('.cache/simulation/vio-pose-transport')/str(uuid.uuid4())
    out.mkdir(parents=True)
    rclpy.init(args=['--ros-args','-p','calibration_id:='+'b'*64,'-p','source_contract:='+CONTRACT])
    fixture = Node('visual_slam')
    for key,value in SOURCE_PARAMETERS.items(): fixture.declare_parameter(key,value)
    if any(n in ('/uav/vio/pose','/visual_slam/tracking/vo_pose_covariance') for n,_ in fixture.get_topic_names_and_types()):
        raise RuntimeError('Domain 93 is occupied')
    adapter = CuvslamPose()
    executor = SingleThreadedExecutor()
    executor.add_node(fixture);executor.add_node(adapter)
    source = fixture.create_publisher(PoseWithCovarianceStamped,adapter.pose_topic,10)
    tracking = fixture.create_publisher(VisualSlamStatus,adapter.tracking_topic,10)
    poses,statuses,upstream = [],[],[]
    fixture.create_subscription(PoseWithCovarianceStamped,'/uav/vio/pose',lambda m:poses.append(time.monotonic()),10)
    fixture.create_subscription(VioStatus,'/uav/vio/pose_status',lambda m:statuses.append(
        dict(mono=time.monotonic(),valid=m.valid,reason=m.reason,session=m.localization_session,reset=m.reset_counter)),10)
    delayed = Future()
    async def sdk_reset(request,response):
        upstream.append(dict(retired=adapter.fault=='VIO_RESET_REQUESTED',reset=adapter.counter))
        if case == 'timeout': await delayed
        response.success = case != 'failure'
        return response
    if case != 'missing': fixture.create_service(Reset,'/visual_slam/internal/reset',sdk_reset,
                                                callback_group=ReentrantCallbackGroup())
    client = fixture.create_client(Reset,'/uav/vio/reset')
    next_publish = 0.
    delayed_tracking = deque()
    def observe(duration,variance=.01):
        nonlocal next_publish
        until = time.monotonic()+duration
        while time.monotonic()<until:
            if time.monotonic() >= next_publish:
                stamp = fixture.get_clock().now().to_msg()
                status = VisualSlamStatus(vo_state=1);status.header.stamp = stamp
                delayed_tracking.append(status)
                # Three callbacks can be reordered across DDS topics. Preserve exact stamps.
                if len(delayed_tracking)>2: tracking.publish(delayed_tracking.popleft())
                m = PoseWithCovarianceStamped();m.header.stamp = stamp;m.header.frame_id = 'odom'
                m.pose.pose.orientation.w = 1.
                for i in range(6): m.pose.covariance[i*7] = variance
                source.publish(m)
                next_publish = time.monotonic()+.04
            executor.spin_once(timeout_sec=.002)
    try:
        observe(.7)
        observe(.3,1.)  # SDK initialization fallback: reject, restart warmup.
        warmup_end = time.monotonic()
        observe(1.)
        no_early_output = not poses
        observe(2.)
        ready = bool(poses) and len(statuses)>=5 and all(s['valid'] for s in statuses[-5:])
        start = time.monotonic()
        if not client.service_is_ready(): raise RuntimeError('Proxy discovery failed')
        future = client.call_async(Reset.Request())
        observe(6. if case=='timeout' else 1.)
        response = future.result() if future.done() else None
        late = [s for s in statuses if s['mono'] > start+.5]
        controls = [n for n,_ in fixture.get_topic_names_and_types() if '/fmu/in/' in n
                    and fixture.get_publishers_info_by_topic(n)]
        checks = dict(no_early_output=no_early_output,
            warmup_span=bool(poses) and poses[0]-warmup_end >= 1.9,
            ready=ready,retired_before_forward=all(s['retired'] and s['reset']==1 for s in upstream),
            response=response is not None and response.success==(case=='success'),
            forwarded=(len(upstream)==(0 if case=='missing' else 1)),
            latched=len(late)>=5 and all(not s['valid'] and s['reset']==1 and s['reason']=='VIO_RESET_REQUESTED' for s in late),
            no_late_pose=not any(t>start+.5 for t in poses),
            one_session=len({s['session'] for s in statuses})==1,no_fmu=not controls)
        result=dict(passed=all(checks.values()),checks=checks,case=case,upstream=upstream,
                    scope='synthetic DDS pose/reset contracts only, no sensor/EKF/flight')
        (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(out,'PASS' if result['passed'] else 'FAIL',flush=True)
        return 0 if result['passed'] else 1
    finally:
        if not delayed.done(): delayed.set_result(None)
        for _ in range(10): executor.spin_once(timeout_sec=.01)
        executor.shutdown(timeout_sec=1.)
        adapter.destroy_node();fixture.destroy_node();rclpy.try_shutdown()


if __name__=='__main__': raise SystemExit(main())
