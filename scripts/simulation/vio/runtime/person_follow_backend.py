#!/usr/bin/env python3
"""Identity-bound RGB target selection and 3-D follow preview; no motion commands."""
import argparse
from collections import OrderedDict, deque
import json
import math
from pathlib import Path
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
from uav_nav_interfaces.msg import PersonObservationArray, PlanningContext
from uav_nav_interfaces.srv import SetMissionIntent
from tf2_ros import Buffer, TransformListener

from person_follow_core import FollowPreview, camera_pixel, context_identity
from vision_overlay import TargetSelection, fresh, stamp_seconds

PREFIX = '/uav/people/follow'


class PersonFollowBackend(Node):
    def __init__(self,trajectory_log=None):
        super().__init__('person_follow_backend', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.instance = str(uuid.uuid4())
        self.session = str(uuid.uuid4())
        self.client = None
        self.selection = None
        self.selection_key = None
        self.selection_at = 0.
        self.last_sequence = -1
        self.cache = {}
        self.observations = deque(maxlen=16)
        self.detections = deque(maxlen=16)
        self.context_key = None
        self.core = FollowPreview()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.requests = OrderedDict()
        self.last_clock = self.clock_at = None
        self.clock_fault = False
        self.bound = False
        self.sample_at = 0.
        self.status_sequence = 0
        self.status_pub = self.create_publisher(String, PREFIX+'/status', 10)
        self.goal_pub = self.create_publisher(PoseStamped, PREFIX+'/preview_goal', 1)
        self.service = self.create_service(SetMissionIntent, PREFIX+'/set_intent', self.intent)
        for key,kind,topic in (
            ('context',PlanningContext,'/planning/context'),
            ('info',CameraInfo,'/px4_depth/camera_info'),
            ('odom',Odometry,'/planning/ego/odometry'),
            ('observations',PersonObservationArray,'/uav/people/observations'),
            ('detections',Detection2DArray,'/uav/people/detections')):
            self.create_subscription(kind,topic,lambda msg,key=key:self.receive(key,msg),qos_profile_sensor_data)
        self.create_subscription(String,PREFIX+'/selection',self.select,10)
        self.timer = self.create_timer(.1,self.tick,clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME))
        from mission_person_observer import AutomaticPersonObserver
        self.automatic = AutomaticPersonObserver(self,trajectory_log)

    def now_s(self):
        return self.get_clock().now().nanoseconds*1e-9

    def receive(self,key,msg):
        previous = self.cache.get(key)
        if previous and stamp_seconds(msg.header.stamp) <= stamp_seconds(previous[0].header.stamp):
            if key in ('context','observations') and not msg.valid:
                self.core.invalidate('SOURCE_INVALIDATED')
                self.cache[key] = (msg,previous[1])
                if key == 'context':
                    self.context_key = None
            return
        self.cache[key] = (msg,time.monotonic())
        if key == 'observations':
            self.observations.append((msg,time.monotonic()))
        elif key == 'detections':
            self.detections.append((msg,time.monotonic()))
        elif key == 'context':
            identity = context_identity(msg)
            if self.context_key is not None and (not msg.valid or identity != self.context_key):
                self.core.invalidate('MAP_OR_LOCALIZATION_CONTEXT_CHANGED')
            self.context_key = identity if msg.valid else None

    def current(self,key,limit=1.):
        record = self.cache.get(key)
        if record and fresh(stamp_seconds(record[0].header.stamp),self.now_s(),record[1],time.monotonic(),limit):
            return record[0]
        return None

    def select(self,msg):
        try:
            if len(msg.data) > 8192:
                raise ValueError('SELECTION_TOO_LARGE')
            item = json.loads(msg.data)
            if not isinstance(item,dict) or not isinstance(item.get('client_id'),str):
                return
            client = str(uuid.UUID(item['client_id']))
            sequence, selection_id = item['sequence'], item['selection_id']
            if (type(sequence) is not int or type(selection_id) is not int or sequence < 0 or selection_id < 1
                    or type(item['valid']) is not bool):
                raise ValueError('INVALID_SELECTION_IDENTITY')
            # Another live observer cannot displace a selection or active preview.
            if self.client and client != self.client and time.monotonic()-self.selection_at < 1.:
                return
            if client == self.client and sequence <= self.last_sequence:
                return
            if not item['valid']:
                if client == self.client:
                    self.core.invalidate('UI_SELECTION_LOST_OR_CLEARED')
                    self.selection = None
                    self.last_sequence = sequence
                return
            source = float(item['source_stamp'])
            box = item['box']
            if (not math.isfinite(source) or not isinstance(item['frame_id'],str) or not item['frame_id']
                    or len(box) != 4 or not all(type(v) in (float,int) and math.isfinite(v) for v in box)
                    or not box[0] < box[2] or not box[1] < box[3]):
                raise ValueError('INVALID_SELECTION_GEOMETRY')
            if not -.05 <= self.now_s()-source <= 1.:
                return
            key = (client,selection_id)
            if key != self.selection_key:
                if self.core.active:
                    # A new selection ends the old preview; it never inherits START.
                    self.core.command('CANCEL')
                self.core.reset()
                self.bound = False
                self.sample_at = 0.
                self.session = str(uuid.uuid4())
                self.selection_key = key
                self.requests.clear()
            self.client, self.last_sequence = client, sequence
            self.selection, self.selection_at = item, time.monotonic()
        except (ValueError,TypeError,KeyError,OverflowError):
            # A malformed packet from any publisher cannot mutate a valid selection.
            return

    def validate_inputs(self):
        if self.clock_fault:
            raise ValueError('CLOCK_FAULT_RESTART_REQUIRED')
        if self.selection is None or time.monotonic()-self.selection_at > .75:
            raise ValueError('UI_SELECTION_HEARTBEAT_LOST')
        context,info,odom = self.current('context'),self.current('info'),self.current('odom',.5)
        if (context is None or not context.valid or not context.map_id or context.header.frame_id != 'map'):
            raise ValueError('PLANNING_CONTEXT_UNAVAILABLE')
        if self.selection.get('context') != list(context_identity(context)):
            raise ValueError('SELECTION_CONTEXT_MISMATCH')
        if info is None or info.header.frame_id != self.selection['frame_id']:
            raise ValueError('CAMERA_CALIBRATION_UNAVAILABLE')
        if not np_finite(info.d) or any(abs(v) > 1e-9 for v in info.d):
            raise ValueError('DISTORTED_CAMERA_UNSUPPORTED')
        if odom is None or odom.header.frame_id != 'map' or odom.child_frame_id != 'base_link':
            raise ValueError('MAP_ODOMETRY_UNAVAILABLE')
        if not self.publisher_unique('/uav/people/observations') or not self.publisher_unique('/planning/context'):
            raise ValueError('AMBIGUOUS_CONTEXT_OR_OBSERVATION_SOURCE')
        latest_people, latest_detections = self.current('observations'), self.current('detections')
        if latest_people is None or not latest_people.valid:
            raise ValueError('PERSON_OBSERVATIONS_UNAVAILABLE')
        if (len(latest_people.observations) != 1 or latest_detections is None
                or len(latest_detections.detections) != 1):
            raise ValueError('SINGLE_PERSON_REQUIRED')
        if (context_identity(latest_people) != context_identity(context)
                or (self.core.track_id is not None and latest_people.observations[0].track_id != self.core.track_id)):
            raise ValueError('TARGET_CONTEXT_OR_TRACK_CHANGED')
        return context,info,odom

    def publisher_unique(self,topic):
        return len(self.get_publishers_info_by_topic(topic)) == 1

    def sample(self):
        context,info,odom = self.validate_inputs()
        selected_stamp = self.selection['source_stamp']
        selected_frame = self.selection['frame_id']
        selected_box = self.selection['box']
        detection = next((msg for msg,arrival in reversed(self.detections)
            if msg.header.frame_id == selected_frame and abs(stamp_seconds(msg.header.stamp)-selected_stamp)<1e-6
            and fresh(stamp_seconds(msg.header.stamp),self.now_s(),arrival,time.monotonic())),None)
        if detection is None:
            raise ValueError('WAITING_MATCHED_DETECTION')
        # First backend supports one person only; reject crossing and partial multi-person observations.
        if len(detection.detections) != 1:
            raise ValueError('SINGLE_PERSON_REQUIRED')
        box = detection.detections[0].bbox
        rect = [box.center.position.x-box.size_x/2,box.center.position.y-box.size_y/2,
                box.center.position.x+box.size_x/2,box.center.position.y+box.size_y/2]
        if not np_finite(rect) or TargetSelection.overlap(rect,selected_box)<.85:
            raise ValueError('SELECTED_BOX_DOES_NOT_MATCH_SOURCE')
        matched = None
        for msg,arrival in reversed(self.observations):
            if (not msg.valid or msg.header.frame_id != 'map' or msg.point_semantics != 'visible_torso_surface'
                    or context_identity(msg) != context_identity(context)
                    or not fresh(stamp_seconds(msg.header.stamp),self.now_s(),arrival,time.monotonic())):
                continue
            if len(msg.observations) != 1:
                continue
            person = msg.observations[0]
            if abs(stamp_seconds(person.last_observed)-selected_stamp)<1e-6:
                matched = person
                break
        if matched is None:
            raise ValueError('WAITING_MATCHED_3D_OBSERVATION')
        if not -.05 <= self.now_s()-selected_stamp <= 1.:
            raise ValueError('TARGET_SOURCE_EXPIRED')
        position = [matched.position.x,matched.position.y,matched.position.z]
        transform = self.tf.lookup_transform(selected_frame,'map',Time(seconds=selected_stamp)).transform
        u,v = camera_pixel(position,transform,info.k)
        if not (0 <= u < info.width and 0 <= v < info.height
                and selected_box[0] <= u <= selected_box[2] and selected_box[1] <= v <= selected_box[3]):
            raise ValueError('OBSERVATION_OUTSIDE_SELECTED_BOX')
        if (not matched.track_id or matched.depth_samples < 1 or not math.isfinite(matched.depth_spread_m)
                or not 0 <= matched.depth_spread_m <= .5 or not .5 <= matched.detection_confidence <= 1.):
            raise ValueError('TARGET_DEPTH_OR_CONFIDENCE_INVALID')
        p = odom.pose.pose.position
        previous_stamp = self.core.samples[-1][0] if self.core.samples else None
        self.core.observe(matched.track_id,position,[p.x,p.y,p.z],selected_stamp)
        if previous_stamp is None or selected_stamp > previous_stamp:
            self.sample_at = time.monotonic()
        self.bound = self.core.track_id is not None

    def refresh(self):
        now,mono = self.now_s(),time.monotonic()
        if self.last_clock is not None and now < self.last_clock:
            self.clock_fault = True
            self.tf.clear()
        if self.last_clock is None or now > self.last_clock:
            self.clock_at = mono
        if now > 0 and self.clock_at is not None and mono-self.clock_at > .5:
            self.clock_fault = True
        self.last_clock = now
        if self.selection is None:
            return
        try:
            self.sample()
        except Exception as error:
            reason = str(error) if isinstance(error,ValueError) else 'HISTORICAL_TF_UNAVAILABLE'
            if (reason in ('WAITING_MATCHED_DETECTION','WAITING_MATCHED_3D_OBSERVATION') and self.bound
                    and time.monotonic()-self.sample_at <= .35 and self.core.state != 'LOST'):
                # RGB, inference and depth observations arrive independently.
                # Wait briefly for a new exact pair; never republish the old candidate.
                self.core.goal = None
                self.core.reason = 'WAITING_NEXT_MATCHED_OBSERVATION'
                return
            if self.bound or self.core.active:
                self.core.invalidate(reason)
            elif self.core.state != 'LOST':
                self.core.state,self.core.reason = 'ACQUIRE',reason
                self.core.goal = None

    def intent(self,request,response):
        self.refresh()
        try:
            identity = str(uuid.UUID(bytes=bytes(request.mission_uuid.uuid)))
            request_id = str(uuid.UUID(bytes=bytes(request.request_id.uuid)))
            if (identity != self.session or request.coordinator_instance != self.instance
                    or request_id == str(uuid.UUID(int=0))):
                raise ValueError('FOLLOW_SESSION_MISMATCH')
            if not self.publisher_unique(PREFIX+'/status'):
                raise ValueError('DUPLICATE_FOLLOW_BACKEND')
            if request_id in self.requests:
                verb,accepted,reason,phase = self.requests[request_id]
                if verb != request.command:
                    raise ValueError('REQUEST_ID_CONFLICT')
            else:
                accepted,reason = self.core.command(request.command)
                phase = self.core.state
                self.requests[request_id] = (request.command,accepted,reason,phase)
                if len(self.requests)>128:
                    self.requests.popitem(last=False)
            response.accepted,response.reason,response.phase = accepted,reason,phase
        except ValueError as error:
            response.accepted,response.reason,response.phase = False,str(error),self.core.state
        self.publish_status()
        return response

    def publish_status(self):
        self.status_sequence += 1
        payload = dict(schema=1,coordinator_instance=self.instance,mission_uuid=self.session,
            client_id=self.client,selection_id=self.selection_key[1] if self.selection_key else None,
            sequence=self.status_sequence,source_stamp=self.now_s(),phase=self.core.state,reason=self.core.reason,
            track_id=self.core.track_id,mode='PREVIEW_ONLY',flight_execution=False,
            cleanup_confirmed=None,distance_m=self.core.distance,motion=self.core.motion,velocity_xy=self.core.velocity,
            candidate_goal=self.core.goal,execution_blockers=['DYNAMIC_SAFETY_UNQUALIFIED','PX4_FOLLOW_BT_STEP_UNAVAILABLE'])
        payload['automatic'] = self.automatic.policy.status()
        payload['automatic']['recording_error'] = self.automatic.recording_error
        self.status_pub.publish(String(data=json.dumps(payload,allow_nan=False)))

    def tick(self):
        self.refresh()
        self.automatic.tick()
        if self.core.goal is not None:
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = 'map'
            goal.pose.position.x,goal.pose.position.y,goal.pose.position.z = self.core.goal
            goal.pose.orientation.w = 1.
            self.goal_pub.publish(goal)  # Visualization-only topic; never /uav/goal or a planner input.
        self.publish_status()


def np_finite(values):
    return all(math.isfinite(v) for v in values)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trajectory-log',type=Path)
    args = parser.parse_args(argv)
    if args.trajectory_log is not None and not args.trajectory_log.parent.is_dir():
        parser.error('trajectory log parent must exist')
    rclpy.init()
    node = PersonFollowBackend(args.trajectory_log)
    try:
        rclpy.spin(node)
    finally:
        node.automatic.policy.finish('BACKEND_CLOSED',time.monotonic())
        node.automatic.save(force=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
