#!/usr/bin/env python3
"""Read-only stereo/SDK/PX4 viewer; bounded timestamped images, no control publishers."""

# Support canonical direct execution and historical flat imports.
import sys as _sys
from pathlib import Path as _Path
_scripts = str(_Path(__file__).resolve().parents[3])
if _scripts not in _sys.path:
    _sys.path.insert(0, _scripts)
from support.paths import enable_imports as _enable_imports
_enable_imports()

import json
from pathlib import Path
import sys
import time
import tkinter as tk
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from px4_msgs.msg import VehicleLocalPosition


def main():
    out=Path(sys.argv[1]); capture=out/'flight-images';capture.mkdir(exist_ok=True)
    estimator=json.loads((out/'manifest.json').read_text()).get('estimator','cuvslam')
    rclpy.init();node=rclpy.create_node('vio_stereo_readonly_ui')
    root=tk.Tk();root.geometry('900x530+980+480');root.title('PX4 VIO stereo / mission diagnostic (read only)')
    panels={s:tk.Label(root,text=s) for s in ('left','right')}
    for i,p in enumerate(panels.values()):p.grid(row=0,column=i)
    status=tk.Label(root,justify='left',font=('monospace',10),wraplength=880);status.grid(row=1,column=0,columnspan=2,sticky='w')
    latest={};cov=None;height=None;last_save=0.;count=0
    def image(side,m):latest[side]=m
    def pose(m):
        nonlocal cov
        cov=dict(stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9,z_variance=m.pose.covariance[14])
    def position(m):
        nonlocal height
        height=dict(px4_timestamp=m.timestamp,z=m.z,z_valid=m.z_valid)
    for side in panels:node.create_subscription(Image,'/vio/'+side+'/image',lambda m,s=side:image(s,m),qos_profile_sensor_data)
    node.create_subscription(PoseWithCovarianceStamped,('/openvins/pose' if estimator=='openvins' else '/visual_slam/tracking/vo_pose_covariance'),pose,10)
    node.create_subscription(VehicleLocalPosition,'/px4_7/fmu/out/vehicle_local_position'+('_v'+str(VehicleLocalPosition.MESSAGE_VERSION) if getattr(VehicleLocalPosition,'MESSAGE_VERSION',0) else ''),position,qos_profile_sensor_data)
    def ppm(m):
        if m.encoding not in ('rgb8','mono8') or len(m.data)!=m.step*m.height:return None
        channels=1 if m.encoding=='mono8' else 3
        if m.step<m.width*channels:return None
        data=bytes(m.data)
        magic='P5' if channels==1 else 'P6'
        return f'{magic}\n{m.width} {m.height}\n255\n'.encode()+b''.join(data[i*m.step:i*m.step+channels*m.width] for i in range(m.height))
    def tick():
        nonlocal last_save,count
        for _ in range(20):rclpy.spin_once(node,timeout_sec=0)
        payload={}
        stamps={s:m.header.stamp.sec+m.header.stamp.nanosec/1e9 for s,m in latest.items()}
        for side,m in latest.items():
            data=ppm(m)
            if data:
                payload[side]=data
                photo=tk.PhotoImage(data=data,format='PPM').subsample(2)
                panels[side].configure(image=photo);panels[side].photo=photo
        if len(payload)==2 and stamps['left']==stamps['right'] and time.monotonic()-last_save>=1 and count<240:
            last_save=time.monotonic();count+=1
            name=f'{count:04d}'
            for side,data in payload.items():(capture/(name+'-'+side+'.ppm')).write_bytes(data)
            (capture/(name+'.json')).write_text(json.dumps(dict(system_ns=time.time_ns(),mono=time.monotonic(),image_stamp=stamps['left'],sdk_pose=cov,px4_local=height),indent=2)+'\n')
        terminal='Waiting for BT terminal; images recorded at 1 Hz'
        for file in ('flight-observation.json','ui-session.json'):
            try:
                receipt=json.loads((out/file).read_text())
                if file=='flight-observation.json':
                    mission=receipt.get('result',{})
                    terminal=f"Mission: {mission.get('code','UNKNOWN')} / {mission.get('reason','')}\nCleanup confirmed: {mission.get('cleanup_confirmed',False)}"
                else:
                    snapshot=receipt.get('result_snapshot',{})
                    terminal=f"UI state: {receipt.get('state','UNKNOWN')}\nDiagnostic checks passed: {snapshot.get('diagnostic_checks_passed',False)}; flight qualified: {receipt.get('qualification',False)}"
            except (OSError,ValueError):pass
        status.configure(text=f'Run: {out.name}\nStereo stamps: {stamps}\n{estimator} covariance (raw): {cov}\nPX4 local: {height}\n{terminal}\nImage structure is NOT a tracked-feature count.\nClose owned session: touch {out}/close-ui')
        root.after(100,tick)
    tick()
    try:root.mainloop()
    finally:node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
