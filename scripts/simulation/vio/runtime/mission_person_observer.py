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
from person_follow_core import camera_pixel, context_identity
from vision_overlay import fresh, stamp_seconds


class AutomaticPersonObserver:
    def __init__(self,backend,trajectory_log=None):
        self.backend = backend
        self.policy = MissionPersonObservation()
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
        self.policy.enable(request.data,time.monotonic())
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
                or task.phase not in ('NAVIGATE','PLAN_REQUEST','HOVER')
                or task.total_remaining_s < p.config.follow_s+p.config.observe_s+8.
                or not b.publisher_unique('/uav/bt/task_status')):
            p.unavailable('WAITING_ELIGIBLE_RUNNING_MISSION',mono)
            return
        context,people,info,odom,detections = (b.current(key,age) for key,age in (
            ('context',1.),('observations',1.),('info',1.),('odom',.5),('detections',1.)))
        if b.clock_fault or context is None or not context.valid or context.header.frame_id != 'map':
            p.unavailable('MAP_OR_CLOCK_UNAVAILABLE',mono)
            return
        if (people is None or not people.valid or people.header.frame_id != 'map'
                or people.point_semantics != 'visible_torso_surface'
                or context_identity(people) != context_identity(context)
                or len(people.observations) != 1 or detections is None or len(detections.detections) != 1
                or not all(b.publisher_unique(topic) for topic in ('/uav/people/observations','/planning/context','/uav/people/detections'))):
            p.unavailable('SINGLE_FRESH_PERSON_REQUIRED',mono)
            return
        person = people.observations[0]
        source = stamp_seconds(person.last_observed)
        if (not -.05<=now-source<=1. or info is None or odom is None
                or odom.header.frame_id != 'map' or odom.child_frame_id != 'base_link'
                or info.header.frame_id != detections.header.frame_id
                or any(abs(v)>1e-9 for v in info.d) or not all(math.isfinite(v) for v in info.d)
                or person.depth_samples<1 or not .5<=person.detection_confidence<=1.
                or not 0<=person.depth_spread_m<=.5):
            p.unavailable('PERSON_LOCALIZATION_UNAVAILABLE',mono)
            return
        # Perception and detection are asynchronous. Locate the exact capture pair.
        matching = next((msg for msg,arrival in reversed(b.detections)
            if abs(stamp_seconds(msg.header.stamp)-source)<1e-6
            and msg.header.frame_id == info.header.frame_id and len(msg.detections)==1
            and fresh(stamp_seconds(msg.header.stamp),now,arrival,mono)),None)
        if matching is None:
            if p.last_mono is None or mono-p.last_mono>p.config.max_gap_s:
                p.unavailable('WAITING_CAPTURE_PAIR',mono)
            return
        try:
            transform = b.tf.lookup_transform(info.header.frame_id,'map',Time.from_msg(person.last_observed)).transform
            point = [person.position.x,person.position.y,person.position.z]
            u,v = camera_pixel(point,transform,info.k)
            box = matching.detections[0].bbox
            if not (0<=u<info.width and 0<=v<info.height
                    and abs(u-box.center.position.x)<=box.size_x/2
                    and abs(v-box.center.position.y)<=box.size_y/2):
                raise ValueError('PERSON_OUTSIDE_DETECTION')
            mission = (bytes(task.mission_uuid.uuid).hex(),task.coordinator_instance,
                       bytes(task.control_session.session_id.uuid).hex(),task.plan_revision)
            p.sample(track_id=person.track_id,point=point,source_stamp=source,mono=mono,
                     context=context_identity(context),mission=mission)
        except Exception:
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
            marker.ns,marker.id,marker.type,marker.action = 'person_motion',index,Marker.LINE_STRIP,Marker.ADD
            marker.pose.orientation.w = 1.
            marker.scale.x = .06
            marker.color.r,marker.color.g,marker.color.b,marker.color.a = (1.,.65,.15,.95)
            for sample in episode['points']:
                point = Point(x=sample['position'][0],y=sample['position'][1],z=sample['position'][2])
                marker.points.append(point)
            markers.append(marker)
            if episode['points']:
                label = Marker()
                label.header, label.ns, label.id = marker.header,'person_motion_label',index
                label.type,label.action = Marker.TEXT_VIEW_FACING,Marker.ADD
                last = marker.points[-1]
                label.pose.position = Point(x=last.x,y=last.y,z=last.z+.25)
                label.pose.orientation.w = 1.
                label.scale.z = .22
                label.color = marker.color
                label.text = f"person {episode['track_id']} / SHADOW"
                markers.append(label)
                latest_path.poses = []
                for sample in episode['points']:
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
        payload = dict(schema=1,mode='SHADOW_ONLY',flight_execution=False,
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
        if time.monotonic()-self.last_marker>=.2:
            self.last_marker = time.monotonic()
            self.publish()
            self.save()
