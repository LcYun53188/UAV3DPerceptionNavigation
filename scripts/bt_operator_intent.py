#!/usr/bin/env python3
"""Send one operator intent to the current isolated SITL BT, without a USB radio."""
import argparse
from copy import deepcopy
import json
import os
import time
import uuid


COMMANDS = ('START', 'PAUSE', 'RESUME', 'RETURN', 'RESUME_MISSION', 'END_MISSION', 'CANCEL')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=COMMANDS)
    parser.add_argument('--enable-sitl-commands', action='store_true')
    args = parser.parse_args()
    if not args.enable_sitl_commands or os.environ.get('ROS_DOMAIN_ID') != '78':
        parser.error('Requires --enable-sitl-commands and isolated SITL ROS_DOMAIN_ID=78')
    import rclpy
    from uav_nav_interfaces.msg import TaskStatus
    from uav_nav_interfaces.srv import SetMissionIntent
    rclpy.init()
    node = rclpy.create_node('bt_operator_intent')
    client = node.create_client(SetMissionIntent, '/uav/bt/set_intent')
    current, received = None, 0.

    def receive(message):
        nonlocal current, received
        current, received = message, time.monotonic()

    subscription = node.create_subscription(TaskStatus, '/uav/bt/task_status', receive, 10)
    try:
        until = time.monotonic()+5.
        while (current is None or not client.service_is_ready()) and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
        if (current is None or time.monotonic()-received > .5 or not client.service_is_ready() or
                len(node.get_publishers_info_by_topic('/uav/bt/task_status')) != 1):
            raise RuntimeError('Current unique runner status/service unavailable')
        snapshot = deepcopy(current)
        if ((args.command == 'START' and snapshot.phase != 'WAIT_START') or
                (args.command != 'START' and not any(snapshot.mission_uuid.uuid))):
            raise RuntimeError('Intent does not match the current root state')
        request = SetMissionIntent.Request(mission_uuid=snapshot.mission_uuid,
            coordinator_instance=snapshot.coordinator_instance,
            request_id=uuid_message(uuid.uuid4().bytes), command=args.command)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3.)
        if not future.done():
            raise RuntimeError('Admission response timeout; do not infer physical completion')
        response = future.result()
        print(json.dumps(dict(command=args.command, accepted=response.accepted,
            reason=response.reason, phase=response.phase, root=str(uuid.UUID(bytes=bytes(snapshot.mission_uuid.uuid))),
            scope='Admission only; inspect root feedback and final result for physical completion')))
        return 0 if response.accepted else 2
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.try_shutdown()


def uuid_message(value):
    from unique_identifier_msgs.msg import UUID
    return UUID(uuid=list(value))


if __name__ == '__main__':
    raise SystemExit(main())
