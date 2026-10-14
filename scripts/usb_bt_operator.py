#!/usr/bin/env python3
"""Explicit USB switch mapping to the bound BT; sticks remain QGC's input."""
import argparse
from copy import deepcopy
import json
import os
import time
import uuid

from usb_joystick import Joystick, SwitchEdges


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', required=True)
    parser.add_argument('--mapping', required=True, help='Reviewed JSON mapping from usb_joystick.py inspection')
    parser.add_argument('--enable-sitl-commands', action='store_true')
    args = parser.parse_args()
    if not args.enable_sitl_commands or os.environ.get('ROS_DOMAIN_ID') != '78':
        parser.error('Task commands require --enable-sitl-commands and the owned SITL domain 78')
    # Import ROS only after the explicit simulation guard.
    import rclpy
    from uav_nav_interfaces.msg import TaskStatus
    from uav_nav_interfaces.srv import SetMissionIntent
    with open(args.mapping) as stream:
        mapping = SwitchEdges(json.load(stream))
    joystick = Joystick(args.device)
    for item in mapping.mapping:
        if item['index'] >= (joystick.axes if item['kind'] == 'axis' else joystick.buttons):
            joystick.close()
            raise ValueError('Mapping index is absent from this joystick')
    rclpy.init()
    node = rclpy.create_node('usb_bt_operator')
    client = node.create_client(SetMissionIntent, '/uav/bt/set_intent')
    current, received, pending = None, 0., None

    def status(message):
        nonlocal current, received
        current, received = message, time.monotonic()

    node.create_subscription(TaskStatus, '/uav/bt/task_status', status, 10)

    def send(command, snapshot):
        request = SetMissionIntent.Request(mission_uuid=snapshot.mission_uuid,
            coordinator_instance=snapshot.coordinator_instance, command=command)
        request.request_id.uuid = list(uuid.uuid4().bytes)
        return client.call_async(request)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.01)
            now = time.monotonic()
            for _ in range(64):
                event = joystick.read()
                if event is None:
                    break
                mapping.receive(event, now)
            if pending is not None and pending.done():
                result = pending.result()
                print(json.dumps(dict(accepted=result.accepted, reason=result.reason, phase=result.phase)), flush=True)
                pending = None
            for command in mapping.ready(now):
                if (current is None or now-received > .5 or not client.service_is_ready() or
                        len(node.get_publishers_info_by_topic('/uav/bt/task_status')) != 1 or
                        (pending is not None and command != 'CANCEL')):
                    print('Command dropped: unavailable/ambiguous status or pending request', flush=True)
                    continue
                snapshot = deepcopy(current)
                if command == 'START' and snapshot.phase != 'WAIT_START':
                    print('START dropped: runner is not waiting on the ground', flush=True)
                    continue
                if command != 'START' and not any(snapshot.mission_uuid.uuid):
                    continue
                pending = send(command, snapshot)
                print(f'Operator intent: {command}', flush=True)
    except (OSError, KeyboardInterrupt) as error:
        # Bind to the last observed root. Do not rediscover/rebind after disconnect.
        if current is not None and any(current.mission_uuid.uuid) and client.service_is_ready():
            future = send('CANCEL', deepcopy(current))
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.)
            print('USB operator stopped; cancellation requested, physical cleanup follows the root Action', flush=True)
        print(str(error), flush=True)
    finally:
        joystick.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
