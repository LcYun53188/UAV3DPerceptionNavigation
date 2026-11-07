"""ROS task observer and map trajectory display for automatic follow shadow episodes."""
import json
import math
from collections import deque
from pathlib import Path
import time

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path as RosPath
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from std_srvs.srv import SetBool
from uav_nav_interfaces.msg import TaskStatus
from visualization_msgs.msg import Marker, MarkerArray

from mission_person_observation import MissionPersonObservation
from mission_person_intervention import MissionInterlude
from person_follow_core import camera_pixel, context_identity
from vision_overlay import TargetSelection,fresh,stamp_seconds


def reliable_person_boxes(message):
    """Count confident people, suppressing near-identical duplicate boxes only.

    Distinct nonempty track IDs remain separate even when their boxes overlap.
    Main detector output already passes NMS; this also guards repeated entries.
    """
    accepted=[]
    for detection in message.detections:
        if not any(result.hypothesis.class_id in ('person','0')
                   and math.isfinite(result.hypothesis.score) and .5<=result.hypothesis.score<=1.
                   for result in detection.results):
            continue
        box=detection.bbox
        x,y,w,h=box.center.position.x,box.center.position.y,box.size_x,box.size_y
        if not all(math.isfinite(v) for v in (x,y,w,h)) or min(w,h)<=0:
            continue
        rect=(x-w/2,y-h/2,x+w/2,y+h/2)
        identifier=detection.id
        duplicate=False
        for old_id,old_box in accepted:
            if identifier and identifier==old_id:
                duplicate=True
                break
            if identifier and old_id and identifier!=old_id:
                continue
            if TargetSelection.overlap(rect,old_box)>=.95:
                duplicate=True
                break
        if not duplicate:
            accepted.append((identifier,rect))
    return accepted


class AutomaticPersonObserver:
    def __init__(self,backend,trajectory_log=None):
        self.backend = backend
        self.policy = MissionPersonObservation()
        self.intervention = MissionInterlude()
        self.multi_samples = deque(maxlen=100)
        self.task = None
        self.task_received = 0.
        self.log = Path(trajectory_log) if trajectory_log else None
        if self.log and (self.log.exists() or self.log.with_suffix('.jsonl').exists()):
            raise ValueError('TRAJECTORY_LOG_ALREADY_EXISTS')
        self.archived = deque(maxlen=128)
        self.recording_error = None
        self.last_save = self.last_marker = 0.
        self.node = backend
        qos = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.markers = self.node.create_publisher(MarkerArray,'/uav/people/motion_trajectories',qos)
        self.path = self.node.create_publisher(RosPath,'/uav/people/motion_path',qos)
        self.node.create_subscription(TaskStatus,'/uav/bt/task_status',self.receive_task,qos_profile_sensor_data)
        self.node.create_service(SetBool,'/uav/people/follow/auto_mode',self.set_mode)

    def receive_task(self,msg):
        previous = self.task
        if previous and previous.coordinator_instance == msg.coordinator_instance:
            if (stamp_seconds(msg.header.stamp)<=stamp_seconds(previous.header.stamp)
                    and msg.phase == previous.phase and msg.result_code == previous.result_code):
                return
        self.task,self.task_received = msg,time.monotonic()

    def set_mode(self,request,response):
        if not self.backend.publisher_unique('/uav/people/follow/status'):
            response.success,response.message = False,'DUPLICATE_FOLLOW_BACKEND'
            return response
        self.multi_samples.clear()
        self.policy.enable(request.data,time.monotonic())
        self.intervention.set_mode(self.policy.enabled,time.monotonic())
        self.intervention.inspect(None,time.monotonic())
        self.publish()
        self.save(force=True)
        response.success,response.message = True,('AUTO_SHADOW_ENABLED_NO_FLIGHT_CONTROL' if request.data else 'AUTO_MODE_DISABLED')
        return response

    def observe(self):
        b,p = self.backend,self.policy
        if not p.enabled:
            return
        now,mono = b.now_s(),time.monotonic()
        task = self.task
        if (task is None or not fresh(stamp_seconds(task.header.stamp),now,self.task_received,mono,.75)
                or task.mock or task.result_code or not any(task.mission_uuid.uuid)
                or not any(task.control_session.session_id.uuid) or task.control_session.owner != 'TASK'
                or not task.coordinator_instance or task.control_session.generation <= 0
                or task.phase not in ('NAVIGATE','HOVER')
                or task.total_remaining_s < max(p.config.moving_follow_s+p.config.stationary_observe_s,p.config.multi_observe_s,p.config.lost_wait_s)+8.
                or not b.publisher_unique('/uav/bt/task_status')):
            self.multi_samples.clear()
            p.unavailable('WAITING_ELIGIBLE_RUNNING_MISSION',mono)
            return
        context,people,info,odom,detections = (b.current(key,age) for key,age in (
            ('context',1.),('observations',1.),('info',1.),('odom',.5),('detections',1.)))
        if b.clock_fault or context is None or not context.valid or context.header.frame_id != 'map':
            self.multi_samples.clear()
            p.unavailable('MAP_OR_CLOCK_UNAVAILABLE',mono)
            return
        if (people is None or not people.valid or people.header.frame_id!='map'
                or people.point_semantics!='visible_torso_surface'
                or context_identity(people)!=context_identity(context)
                or detections is None or info is None or odom is None
                or odom.header.frame_id!='map' or odom.child_frame_id!='base_link'
                or info.header.frame_id!=detections.header.frame_id
                or any(abs(v)>1e-9 for v in info.d) or not all(math.isfinite(v) for v in info.d)
                or not all(b.publisher_unique(topic) for topic in
                           ('/uav/people/observations','/planning/context','/uav/people/detections'))):
            self.multi_samples.clear()
            p.unavailable('PERSON_SENSOR_OR_CONTEXT_UNAVAILABLE',mono)
            return
        mission=(bytes(task.mission_uuid.uuid).hex(),task.coordinator_instance,
                 bytes(task.control_session.session_id.uuid).hex(),task.plan_revision)
        if not p._scope(context_identity(context),mission,mono):
            return
        if p.active is not None and p.clock.multiple_since is not None:
            p.advance(mono)  # Lock survives disappearance or reduction to a single target.
            return
        boxes=reliable_person_boxes(detections)
        observed=[person for person in people.observations
            if person.track_id and .5<=person.detection_confidence<=1.
            and -.05<=now-stamp_seconds(person.last_observed)<=1.]
        count=len(boxes)
        if count>=2:
            source=stamp_seconds(detections.header.stamp)
            if self.multi_samples and (source<self.multi_samples[-1][0]
                    or max(source-self.multi_samples[-1][0],mono-self.multi_samples[-1][1])>p.config.max_gap_s):
                self.multi_samples.clear()
            if not self.multi_samples or source>self.multi_samples[-1][0]:
                self.multi_samples.append((source,mono))
            if (len(self.multi_samples)>=p.config.multi_min_samples
                    and min(self.multi_samples[-1][0]-self.multi_samples[0][0],
                            self.multi_samples[-1][1]-self.multi_samples[0][1])>=p.config.multi_confirm_s):
                p.multiple(mono=mono,context=context_identity(context),mission=mission,count=count)
            else:
                p.lost(mono,'CONFIRMING_DEDUPLICATED_MULTIPLE_PEOPLE')
            return
        self.multi_samples.clear()
        if len(boxes)!=1 or len(observed)!=1:
            p.lost(mono,'TARGET_NOT_RELIABLY_LOCALIZED')
            return
        person=observed[0]
        source=stamp_seconds(person.last_observed)
        if person.depth_samples<1 or not 0<=person.depth_spread_m<=.5:
            p.lost(mono,'TARGET_DEPTH_UNAVAILABLE')
            return
        # Perception and detection are asynchronous. Locate the exact capture pair.
        matching = next((msg for msg,arrival in reversed(b.detections)
            if abs(stamp_seconds(msg.header.stamp)-source)<1e-6
            and msg.header.frame_id == info.header.frame_id and len(reliable_person_boxes(msg))==1
            and fresh(stamp_seconds(msg.header.stamp),now,arrival,mono)),None)
        if matching is None:
            if p.last_mono is None or mono-p.last_mono>p.config.max_gap_s:
                p.lost(mono,'WAITING_CAPTURE_PAIR')
            return
        try:
            transform = b.tf.lookup_transform(info.header.frame_id,'map',Time.from_msg(person.last_observed)).transform
            point = [person.position.x,person.position.y,person.position.z]
            u,v = camera_pixel(point,transform,info.k)
            _,rect = reliable_person_boxes(matching)[0]
            if not (0<=u<info.width and 0<=v<info.height
                    and rect[0]<=u<=rect[2] and rect[1]<=v<=rect[3]):
                raise ValueError('PERSON_OUTSIDE_DETECTION')
            mission = (bytes(task.mission_uuid.uuid).hex(),task.coordinator_instance,
                       bytes(task.control_session.session_id.uuid).hex(),task.plan_revision)
            p.sample(track_id=person.track_id,point=point,source_stamp=source,mono=mono,
                     context=context_identity(context),mission=mission)
        except Exception:
            self.multi_samples.clear()
            p.unavailable('PERSON_PROJECTION_OR_SAMPLE_INVALID',mono)

    def publish(self):
        b,p = self.backend,self.policy
        stamp = b.get_clock().now().to_msg()
        markers = [Marker(action=Marker.DELETEALL)]
        latest_path = RosPath()
        latest_path.header.frame_id,latest_path.header.stamp = 'map',stamp
        context = b.current('context')
        current = tuple(context_identity(context)) if context is not None and context.valid else None
        for index,episode in enumerate(p.episodes):
            if tuple(episode['context']) != current:
                continue
            marker = Marker()
            marker.header.frame_id,marker.header.stamp = 'map',stamp
            marker.ns,marker.id,marker.type,marker.action = 'person_motion',index,Marker.LINE_LIST,Marker.ADD
            marker.pose.orientation.w = 1.
            marker.scale.x = .06
            marker.color.r,marker.color.g,marker.color.b,marker.color.a = (1.,.65,.15,.95)
            if not episode['points']:
                continue
            for first,second in zip(episode['points'],episode['points'][1:]):
                if first.get('segment',0)!=second.get('segment',0):
                    continue
                for sample in (first,second):
                    marker.points.append(Point(x=sample['position'][0],y=sample['position'][1],z=sample['position'][2]))
            markers.append(marker)
            if episode['points']:
                label = Marker()
                label.header, label.ns, label.id = marker.header,'person_motion_label',index
                label.type,label.action = Marker.TEXT_VIEW_FACING,Marker.ADD
                last=episode['points'][-1]['position']
                label.pose.position = Point(x=last[0],y=last[1],z=last[2]+.25)
                label.pose.orientation.w = 1.
                label.scale.z = .22
                label.color = marker.color
                label.text = f"person {episode['track_id']} / SHADOW"
                markers.append(label)
                latest_path.poses = []
                latest_segment=episode['points'][-1].get('segment',0)
                for sample in episode['points']:
                    if sample.get('segment',0)!=latest_segment:
                        continue
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'
                    pose.header.stamp = Time(seconds=sample['stamp']).to_msg()
                    pose.pose.position.x,pose.pose.position.y,pose.pose.position.z = sample['position']
                    pose.pose.orientation.w = 1.
                    latest_path.poses.append(pose)
        self.markers.publish(MarkerArray(markers=markers))
        self.path.publish(latest_path)

    def save(self,force=False):
        if self.log is None or not force and time.monotonic()-self.last_save<1.:
            return
        self.last_save = time.monotonic()
        payload = dict(schema=2,mode='SHADOW_ONLY',flight_execution=False,
                       config=self.policy.status()['config'],episodes=list(self.policy.episodes))
        try:
            for episode in self.policy.episodes:
                if episode['end_reason'] is not None and episode['episode_id'] not in self.archived:
                    with self.log.with_suffix('.jsonl').open('a') as output:
                        output.write(json.dumps(episode,allow_nan=False,ensure_ascii=False)+'\n')
                    self.archived.append(episode['episode_id'])
            temp = self.log.with_suffix(self.log.suffix+'.tmp')
            temp.write_text(json.dumps(payload,allow_nan=False,ensure_ascii=False,indent=2)+'\n')
            temp.replace(self.log)
            self.recording_error = None
        except (OSError,ValueError) as error:
            self.recording_error = str(error)

    def tick(self):
        self.observe()
        self.policy.advance(time.monotonic())
        self.intervention.set_mode(self.policy.enabled,time.monotonic())
        # No executor/checkpoint provider is registered yet. Inspecting this
        # readiness is read-only and cannot emit a mission PAUSE request.
        self.intervention.inspect(None,time.monotonic())
        if time.monotonic()-self.last_marker>=.2:
            self.last_marker = time.monotonic()
            self.publish()
            self.save()
