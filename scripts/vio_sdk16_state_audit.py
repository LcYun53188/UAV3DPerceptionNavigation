"""Observe exact SDK16 Track initialization state; no estimator/control writes."""
import math
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped


def valid_state(data):
    if len(data)!=14 or data[1]!=1 or not all(math.isfinite(v) for v in data):return False
    return 8.<=math.sqrt(sum(v*v for v in data[2:5]))<=11.


class Sdk16StateAudit:
    def __init__(self,node,*,state_topic='/visual_slam/sdk16_inertial_state',pose_topic='/visual_slam/tracking/vo_pose_covariance'):
        self.state_topic=state_topic
        self.node=node;self.states=[];self.covariance_stamps=[];self.writer_gid=None;self.writer_valid=True
        node.create_subscription(Float64MultiArray,state_topic,self.on_state,10)
        node.create_subscription(PoseWithCovarianceStamped,pose_topic,self.on_pose,10)

    def on_state(self,msg):
        endpoints=self.node.get_publishers_info_by_topic(self.state_topic)
        gid=bytes(endpoints[0].endpoint_gid).hex() if len(endpoints)==1 else None
        self.writer_valid &= gid is not None and (self.writer_gid is None or self.writer_gid==gid)
        self.writer_gid=gid
        self.states.append(list(msg.data))

    def on_pose(self,msg):
        self.covariance_stamps.append(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec)

    def result(self):
        ready={round(row[0]):row for row in self.states if valid_state(row)}
        first=min(ready,default=None)
        last=max(ready,default=None)
        duration=(last-first)/1e9 if first is not None else 0.
        after=[row for row in self.states if first is not None and row[0]>=first]
        checks=dict(initialized=first is not None,
            initialized_duration=duration>=120.,
            initialization_retained=bool(after) and all(valid_state(row) for row in after),
            covariance_output=bool(self.covariance_stamps),
            original_time_state_match=bool(self.covariance_stamps) and all(s in ready for s in self.covariance_stamps),
            no_output_before_initialization=first is not None and bool(self.covariance_stamps) and min(self.covariance_stamps)>=first,
            state_writer=self.writer_gid is not None and self.writer_valid)
        return dict(passed=all(checks.values()),checks=checks,track_samples=len(self.states),
            initialized_samples=len(ready),first_track_ns=round(self.states[0][0]) if self.states else None,
            first_initialized_ns=first,last_initialized_ns=last,initialized_duration_s=duration,
            covariance_samples=len(self.covariance_stamps),
            scope='matched SDK gravity and finite IMU state gate; no global covariance or flight qualification')


def loaded_candidate_receipt(pid,prefix):
    """Inspect the owned launch process tree, including a component container."""
    import hashlib
    from pathlib import Path
    queue=[pid];seen=set();libraries={}
    while queue:
        current=queue.pop()
        if current in seen:continue
        seen.add(current)
        try:
            queue += [int(v) for v in Path(f'/proc/{current}/task/{current}/children').read_text().split()]
            for line in Path(f'/proc/{current}/maps').read_text().splitlines():
                path=line.split()[-1]
                if path.endswith(('/libcuvslam.so','/libvisual_slam_node.so')):
                    libraries[path]=hashlib.sha256(Path(path).read_bytes()).hexdigest()
        except (FileNotFoundError,ProcessLookupError):continue
    expected={str(prefix/'lib'/name) for name in ('libcuvslam.so','libvisual_slam_node.so')}
    return dict(passed=set(libraries)==expected,root_pid=pid,observed_pids=sorted(seen),
        loaded_libraries_sha256=libraries,scope='actual /proc maps in owned SDK process tree')
