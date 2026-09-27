"""Save/load a map bundle through the session manager, never directly into nvblox."""
import argparse
import time
from pathlib import Path
import rclpy
from nvblox_msgs.srv import FilePath
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('operation', choices=['save', 'load'])
    parser.add_argument('directory')
    args = parser.parse_args()
    directory = Path(args.directory).expanduser().resolve()
    if args.operation == 'load':
        for name in ('manifest.json', 'static_map.nvblx'):
            if not (directory / name).is_file():
                parser.error(
                    f'Map bundle file not found: {directory / name}. '
                    'Choose an existing map directory (list .cache/maps), '
                    'or create a map with survey_gazebo_map.py first.')
    rclpy.init()
    node = rclpy.create_node('uav_map_bundle_cli')
    client = node.create_client(FilePath, '/uav/map/'+args.operation)
    executor = []
    node.create_subscription(String, '/uav/executor/state', lambda msg: executor.append(msg.data), 1)
    try:
        if not client.wait_for_service(timeout_sec=15):
            raise RuntimeError('Map manager service unavailable')
        deadline = time.monotonic()+15
        while not executor and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if not executor or executor[-1] != 'HOLD':
            raise RuntimeError('Map operation requires an available, stopped executor')
        future = client.call_async(FilePath.Request(file_path=str(directory)))
        rclpy.spin_until_future_complete(node, future, timeout_sec=60)
        if not future.done() or not future.result().success:
            raise RuntimeError('Map operation failed or timed out; inspect map session logs')
        print(f'{args.operation}: {args.directory}')
    finally:
        node.destroy_node()
        rclpy.shutdown()
