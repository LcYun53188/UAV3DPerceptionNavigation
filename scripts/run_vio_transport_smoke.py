#!/usr/bin/env python3
"""Isolated DDS contract test with synthetic odometry/status, never real VIO."""
import json
import os
from pathlib import Path
import time
import uuid

os.environ['ROS_DOMAIN_ID'] = '92'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import Odometry
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from uav_nav_interfaces.msg import VioStatus
from px4_comm_bridge.vio_input_node import VioInput


def main():
    out = Path('.cache/simulation/vio-transport') / str(uuid.uuid4())
    out.mkdir(parents=True)
    rclpy.init(args=['--ros-args','-p','calibration_id:='+'a'*64])
    observer = Node('vio_transport_fixture')
    if any('/visual_slam/' in n or '/uav/vio/' in n for n, _ in observer.get_topic_names_and_types()):
        raise RuntimeError('Domain 92 already contains VIO endpoints')
    adapter = VioInput()
    executor = SingleThreadedExecutor()
    executor.add_node(observer)
    executor.add_node(adapter)
    publisher = observer.create_publisher(Odometry, '/visual_slam/tracking/odometry', 10)
    tracking = observer.create_publisher(VisualSlamStatus, '/visual_slam/status', 10)
    states = []
    phase = 'discovery'
    observer.create_subscription(VioStatus, '/uav/vio/status',
        lambda m: states.append(dict(phase=phase, valid=m.valid,reason=m.reason,
                                     session=m.localization_session)),10)

    def run_phase(name, duration, vo_state=1):
        nonlocal phase
        phase = name
        until = time.monotonic()+duration
        next_publish = 0.
        while time.monotonic()<until:
            if time.monotonic()>=next_publish:
                stamp = observer.get_clock().now().to_msg()
                status = VisualSlamStatus(vo_state=vo_state)
                status.header.stamp = stamp
                status.header.frame_id = 'map'
                tracking.publish(status)
                m = Odometry()
                m.header.frame_id,m.child_frame_id = 'odom','base_link'
                m.header.stamp = stamp
                m.pose.pose.orientation.w = 1.
                for i in range(6):
                    m.pose.covariance[7*i] = .01
                    m.twist.covariance[7*i] = .01
                publisher.publish(m)
                next_publish=time.monotonic()+.03
            executor.spin_once(timeout_sec=.002)

    try:
        run_phase('fresh',2.)
        run_phase('tracking_lost',1.,2)
        run_phase('recovered',1.)
        old=publisher
        publisher=observer.create_publisher(Odometry,'/visual_slam/tracking/odometry',10)
        observer.destroy_publisher(old)
        run_phase('publisher_replaced',1.5)
        run_phase('replacement_latched',.7)
        check = lambda name: [s for s in states if s['phase']==name][-5:]
        controls = {n:len(observer.get_publishers_info_by_topic(n))
                    for n,_ in observer.get_topic_names_and_types()
                    if n.startswith('/px4_7/fmu/in/')}
        passed=(len(check('fresh'))==5 and all(s['valid'] for s in check('fresh'))
                and all(not s['valid'] for s in check('tracking_lost'))
                and all(s['valid'] for s in check('recovered'))
                and all(not s['valid'] and s['reason']=='VIO_PUBLISHER_CHANGED'
                        for s in check('replacement_latched'))
                and not any(controls.values()) and len({s['session'] for s in states})==1)
        (out/'result.json').write_text(json.dumps(dict(passed=passed,domain=92,
            scope='synthetic DDS contracts only, no camera or actual EKF fusion',
            states=len(states),checks={n:check(n) for n in ('fresh','tracking_lost','recovered','replacement_latched')},
            fmu_input_publishers=controls),indent=2)+'\n')
        print(out, 'PASS' if passed else 'FAIL',flush=True)
        return 0 if passed else 1
    finally:
        executor.shutdown()
        adapter.destroy_node()
        observer.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    raise SystemExit(main())
