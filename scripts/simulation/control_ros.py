#!/usr/bin/env python3
"""Map operations through the full project's guarded map-session services."""
import argparse
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from simulation import session


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('save', 'export-mesh'))
    parser.add_argument('output')
    args = parser.parse_args()
    state = session.read()
    session.runtime(state)
    if state['id'] != os.environ.get('UAV_SIM_OPERATOR_ID') or state['program'] != 'full':
        raise RuntimeError('当前完整项目会话已变更')
    import rclpy
    from nvblox_msgs.srv import FilePath
    rclpy.init()
    node = rclpy.create_node('project_map_control')
    try:
        service = '/uav/map/' + ('save' if args.action == 'save' else 'export_mesh')
        client = node.create_client(FilePath, service)
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError('地图服务未就绪；请检查 sim.sh logs')
        future = client.call_async(FilePath.Request(file_path=args.output))
        rclpy.spin_until_future_complete(node, future, timeout_sec=75)
        if not future.done():
            raise RuntimeError('地图操作超时，可能仍在执行；先检查文件和日志')
        if not future.result().success:
            raise RuntimeError('地图操作被拒绝；要求新鲜的落地上锁状态及有效地图，详见 sim.sh logs')
        print('已保存：' + args.output)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, RuntimeError) as exc:
        sys.exit(str(exc))
