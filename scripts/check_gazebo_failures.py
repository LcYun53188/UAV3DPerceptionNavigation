#!/usr/bin/env python3
"""Negative goals for the fixed uav_ego_lab scene; run only while stopped."""
import argparse
import json
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger
from uav_nav_interfaces.msg import TimedTrajectory


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='/tmp/uav_negative_goals.json')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('negative_goal_probe')
    seen = {'planner': [], 'trajectories': 0, 'motion': False}
    node.create_subscription(String, '/uav/planner/state', lambda m: seen['planner'].append(m.data), 10)
    node.create_subscription(TimedTrajectory, '/uav/trajectory',
                             lambda m: seen.__setitem__('trajectories', seen['trajectories'] + 1), 10)
    node.create_subscription(Twist, '/cmd_vel', lambda m: seen.__setitem__('motion', seen['motion'] or
                             sum(abs(v) for v in (m.linear.x, m.linear.y, m.linear.z, m.angular.z)) > 1e-5), 10)
    publisher = node.create_publisher(PoseStamped, '/uav/goal', 10)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)

    results = {}
    try:
        spin(5)
        for name, position in [('obstacle', (0., 0., 1.2)), ('outside', (20., 0., 1.2))]:
            seen.update(planner=[], trajectories=0, motion=False)
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = position
            goal.pose.orientation.w = 1.
            publisher.publish(goal)
            spin(3)
            results[name] = dict(seen)
            results[name]['passed'] = ('BLOCKED_START_OR_GOAL' in seen['planner'] and
                                       not seen['motion'] and seen['trajectories'] == 0)
    finally:
        cancel = node.create_client(Trigger, '/uav/cancel')
        if cancel.wait_for_service(timeout_sec=2):
            rclpy.spin_until_future_complete(node, cancel.call_async(Trigger.Request()), timeout_sec=3)
        node.destroy_node()
        rclpy.shutdown()
    Path(args.output).write_text(json.dumps(results, indent=2) + '\n')
    print(json.dumps(results, indent=2))
    raise SystemExit(0 if results and all(item['passed'] for item in results.values()) else 1)


if __name__ == '__main__':
    main()
