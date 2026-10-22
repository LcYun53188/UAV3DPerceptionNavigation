#!/usr/bin/env python3
"""One identity-bound ground source selection request; no Pocket dependency."""

# Support canonical direct execution and historical flat imports.
import sys as _sys
from pathlib import Path as _Path
_scripts = str(_Path(__file__).resolve().parents[2])
if _scripts not in _sys.path:
    _sys.path.insert(0, _scripts)
from support.paths import enable_imports as _enable_imports
_enable_imports()

import argparse
import json
import os
import time
import uuid


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('profile',choices=('VIO_FLOW','FLOW_HORIZONTAL'))
    parser.add_argument('--enable-sitl-parameters',action='store_true')
    args=parser.parse_args()
    if not args.enable_sitl_parameters or os.environ.get('ROS_DOMAIN_ID')!='78':
        parser.error('Requires isolated ROS_DOMAIN_ID=78 and --enable-sitl-parameters')
    import rclpy
    from uav_nav_interfaces.msg import LocalizationSelection
    from uav_nav_interfaces.srv import SelectLocalization
    from unique_identifier_msgs.msg import UUID
    rclpy.init();node=rclpy.create_node('select_localization_once')
    status,received=None,0.
    def receive(message):
        nonlocal status,received
        status,received=message,time.monotonic()
    node.create_subscription(LocalizationSelection,'/uav/localization/selection',receive,10)
    client=node.create_client(SelectLocalization,'/uav/localization/select')
    try:
        until=time.monotonic()+5.
        while (status is None or not client.service_is_ready()) and time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.05)
        if (status is None or time.monotonic()-received>.5 or not client.service_is_ready() or
                len(node.get_publishers_info_by_topic('/uav/localization/selection'))!=1):
            raise RuntimeError('Fresh unique selector unavailable')
        if status.mock:raise RuntimeError('Synthetic benchmark selector cannot be controlled by this operational CLI')
        future=client.call_async(SelectLocalization.Request(coordinator_instance=status.coordinator_instance,
            request_id=UUID(uuid=list(uuid.uuid4().bytes)),profile=args.profile))
        rclpy.spin_until_future_complete(node,future,timeout_sec=3.)
        if not future.done():raise RuntimeError('Admission timeout; actual fusion is unconfirmed')
        response=future.result()
        print(json.dumps(dict(accepted=response.accepted,reason=response.reason,phase=response.phase,
            scope='Admission only; /uav/localization/selection reports actual completion')))
        return 0 if response.accepted else 2
    finally:node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':raise SystemExit(main())
