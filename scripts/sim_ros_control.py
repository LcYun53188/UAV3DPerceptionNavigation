#!/usr/bin/env python3
"""Bounded ROS operations for sim.sh; run via with_venv.sh."""
import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger
from uav_nav_interfaces.msg import MapSnapshot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['status', 'cancel', 'ready', 'goal'])
    parser.add_argument('xyz', nargs='*', type=float)
    args = parser.parse_args()
    if args.command == 'goal' and (len(args.xyz) != 3 or not all(math.isfinite(v) for v in args.xyz)):
        parser.error('goal requires three finite map-frame coordinates')
    rclpy.init()
    node = rclpy.create_node('sim_control', parameter_overrides=[Parameter('use_sim_time', value=True)])
    latest, received = {}, {}

    def record(key, value):
        latest[key] = value
        received[key] = time.monotonic()

    def on_map(msg):
        record('map', dict(valid=msg.valid, version=msg.version, static=msg.static_map))
        latest['map_stamp'] = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9

    def on_odom(msg):
        p = msg.pose.pose.position
        record('position', [p.x, p.y, p.z])
        latest['odom_stamp'] = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9

    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/uav/navigation/state', lambda m: record('navigation', m.data), durable)
    node.create_subscription(String, '/uav/executor/state', lambda m: record('executor', m.data), 10)
    node.create_subscription(MapSnapshot, '/uav/map/snapshot', on_map, durable)
    node.create_subscription(Odometry, '/uav/localization/odometry', on_odom, qos_profile_sensor_data)

    def wait(predicate, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
            if predicate():
                return True
        return False

    def ready():
        sim_now = node.get_clock().now().nanoseconds / 1e9
        return (latest.get('map', {}).get('valid', False) and
                0 <= sim_now-latest.get('map_stamp', -1e9) <= 2.0 and
                time.monotonic()-received.get('map', 0) <= 2.0 and
                0 <= sim_now-latest.get('odom_stamp', -1e9) <= .5 and
                time.monotonic()-received.get('position', 0) <= .5)

    try:
        if args.command == 'status':
            wait(lambda: False, 4)
            latest['ready'] = ready()
            latest['receive_age_seconds'] = {k: round(time.monotonic()-v, 2) for k, v in received.items()}
            print(json.dumps(latest, indent=2, ensure_ascii=False))
        elif args.command == 'cancel':
            client = node.create_client(Trigger, '/uav/cancel')
            if not client.wait_for_service(timeout_sec=10):
                raise RuntimeError('取消服务不可用；检查 launch 日志。')
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            if not future.done() or not future.result().success:
                raise RuntimeError('取消目标失败或超时。')
            received.pop('executor', None)
            if not wait(lambda: latest.get('executor') == 'HOLD' and 'executor' in received, 5):
                raise RuntimeError('未收到取消后的 HOLD 状态。')
            print('目标已取消；执行器 HOLD。')
        else:
            if not wait(ready, 25 if args.command == 'ready' else 15):
                raise RuntimeError('未收到有效且新鲜的地图/里程计；先 init 或 load，再检查 status 和 logs。')
            if args.command == 'ready':
                print('地图和里程计已就绪。')
                return
            pub = node.create_publisher(PoseStamped, '/uav/goal', 10)
            if not wait(lambda: pub.get_subscription_count() > 0, 3):
                raise RuntimeError('没有目标订阅者。')
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = args.xyz
            msg.pose.orientation.w = 1.0
            pub.publish(msg)
            if not pub.wait_for_all_acked(Duration(seconds=2)):
                raise RuntimeError('目标已发送，但未在时限内收到传输确认；先查看 status，勿假定发送失败。')
            print(f'已发布目标 {args.xyz}；用 status 查看结果，发布不代表到达。')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from None
