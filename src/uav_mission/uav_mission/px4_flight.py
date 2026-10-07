"""SITL-only known-region Mission Action and sole PX4 command gateway.

No planner/TF truth substitution: setpoints use PX4 local position, truth is audit.
Requires an owned supervisor nonce, fixed isolated instance and frozen W0 hashes.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import threading
import time
import uuid

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import (VehicleStatus, VehicleLocalPosition, VehicleLandDetected,
                          BatteryStatus, VehicleCommandAck, OffboardControlMode,
                          TrajectorySetpoint, VehicleCommand, VehicleOdometry)
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from uav_nav_interfaces.action import ExecuteMission
from uav_nav_interfaces.msg import TaskStatus, ControlSession, ControlStatus
from uav_nav_interfaces.srv import PauseMission, ResumeMission
from unique_identifier_msgs.msg import UUID
from px4_comm_bridge.converters import vehicle_odometry_to_ros
from .flight_geometry import Alignment, Segment, ned_enu, distance, finite3, in_region


class FlightServer(Node):
    def __init__(self):
        super().__init__('px4_flight_gateway')
        self.nonce = os.environ.get('UAV_SITL_AUTHORIZATION', '')
        if (not self.nonce or os.environ.get('ROS_DOMAIN_ID') != '78'
                or not os.environ.get('GZ_PARTITION', '').startswith('uav_px4_s0_')):
            raise RuntimeError('Requires owned local SITL supervisor, domain 78 and nonce')
        root = Path(os.environ['UAV_WORKSPACE'])
        self.config = json.loads((root/'simulation/safe_regions/W0.json').read_text())
        for name, digest in self.config['scene_files'].items():
            path = root/'.deps/PX4-Autopilot/Tools/simulation/gz'/name
            if hashlib.sha256(path.read_bytes()).hexdigest() != digest:
                raise RuntimeError('W0 scene/model hash mismatch: '+name)
        self.alignment = Alignment(self.config['map_translation'], self.config['map_yaw_rad'])
        self.lock, self.group = threading.RLock(), ReentrantCallbackGroup()
        self.state_group = MutuallyExclusiveCallbackGroup()
        self.samples, self.received = {}, {}
        self.instance = str(uuid.uuid4())
        self.active_goal = None
        self.reserved = False
        self.phase, self.reason = 'IDLE', ''
        self.sequence, self.generation = 0, 0
        self.session = UUID()
        self.events, self.trace, self.commands = [], [], []
        self.diagnostics=[]
        self.status_history=[]
        self.result = None
        self.canceling = False
        self.requests = {}
        self.owner = 'NONE'
        self.clock_last, self.clock_advance = None, time.monotonic()
        self.clock_fault_latched=False
        self.last_command = 0.
        self.stable_since = None
        self.segment = None
        self.step_index = -1
        self.stop_until = 0.
        self.land_committed = False
        self.output_count = 0
        self.last_graph_check=0.
        for name, kind in [('vehicle_status', VehicleStatus), ('vehicle_local_position', VehicleLocalPosition),
                           ('vehicle_land_detected', VehicleLandDetected), ('battery_status', BatteryStatus),
                           ('vehicle_command_ack', VehicleCommandAck), ('vehicle_odometry', VehicleOdometry)]:
            suffix = f'_v{kind.MESSAGE_VERSION}' if kind.MESSAGE_VERSION else ''
            self.create_subscription(kind, '/px4_7/fmu/out/'+name+suffix, self.callback(name),
                                     qos_profile_sensor_data, callback_group=self.state_group)
        self.mode_pub = self.create_publisher(OffboardControlMode, '/px4_7/fmu/in/offboard_control_mode', 10)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/px4_7/fmu/in/trajectory_setpoint', 10)
        self.command_pub = self.create_publisher(VehicleCommand, '/px4_7/fmu/in/vehicle_command', 10)
        self.tf_pub=TransformBroadcaster(self)
        self.static_tf_pub=StaticTransformBroadcaster(self)
        t=TransformStamped();t.header.frame_id='map';t.child_frame_id='odom'
        t.header.stamp=self.get_clock().now().to_msg()
        t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=self.alignment.translation
        t.transform.rotation.z=math.sin(self.alignment.yaw/2);t.transform.rotation.w=math.cos(self.alignment.yaw/2)
        self.static_tf_pub.sendTransform(t)
        self.child=UUID()
        self.odom_pub = self.create_publisher(Odometry, '/uav/px4/odometry', 10)
        self.control_pub=self.create_publisher(ControlStatus,'/uav/px4/control_status',10)
        self.status_pub = self.create_publisher(TaskStatus, '/uav/px4/task_status', 10)
        self.server = ActionServer(self, ExecuteMission, '/uav/px4/execute_mission',
                                  execute_callback=self.execute, goal_callback=self.goal,
                                  handle_accepted_callback=self.accept, cancel_callback=self.cancel,
                                  callback_group=self.group)
        self.create_service(PauseMission, '/uav/px4/pause', self.pause, callback_group=self.group)
        self.create_service(ResumeMission, '/uav/px4/resume', self.resume, callback_group=self.group)
        self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME), callback_group=self.state_group)

    def callback(self, name):
        def receive(msg):
            with self.lock:
                old = self.samples.get(name)
                if old is not None and msg.timestamp <= old.timestamp:
                    if msg.timestamp < old.timestamp and self.active_goal:
                        self.fault('SOURCE_TIME_RESET')
                    return
                self.samples[name], self.received[name] = msg, time.monotonic()
                if name=='vehicle_status':self.status_history.append(dict(mono=time.monotonic(),ros=msg.timestamp/1e6,nav_state=msg.nav_state,arming_state=msg.arming_state,failsafe=msg.failsafe))
                if name == 'vehicle_odometry':
                    try:
                        odom=vehicle_odometry_to_ros(msg)
                        self.odom_pub.publish(odom)
                        t=TransformStamped();t.header=odom.header;t.child_frame_id=odom.child_frame_id
                        t.transform.translation.x=odom.pose.pose.position.x
                        t.transform.translation.y=odom.pose.pose.position.y
                        t.transform.translation.z=odom.pose.pose.position.z
                        t.transform.rotation=odom.pose.pose.orientation
                        self.tf_pub.sendTransform(t)
                    except ValueError:
                        if self.active_goal:
                            self.fault('INVALID_ODOMETRY_FRAME')
        return receive

    def change(self, phase, reason=''):
        self.phase, self.reason = phase, reason
        if phase in ('FAULT','COMPLETE'):self.child=UUID()
        self.phase_started = time.monotonic()
        self.sequence += 1
        self.stable_since = None
        self.events.append(dict(phase=phase, reason=reason, mono=self.phase_started,
                                ros=self.get_clock().now().nanoseconds/1e9, generation=self.generation,
                                step_index=self.step_index,position_enu=self.position() if 'vehicle_local_position' in self.samples else None,
                                target_enu=getattr(self,'target',None)))
        self.get_logger().info(phase+': '+reason)
        if self.result and hasattr(self,'done_future') and not self.done_future.done():
            self.done_future.set_result(self.result)
        if self.result and self.active_goal and phase in ('FAULT','COMPLETE'):self.publish_status()

    def fresh(self, name, age):
        msg = self.samples.get(name)
        now = time.monotonic()
        ros = self.get_clock().now().nanoseconds/1e9
        return bool(msg and now-self.received[name] <= age and -.05 <= ros-msg.timestamp/1e6 <= age)

    def healthy(self, ground=False):
        c = self.config
        required = [('vehicle_status', c['status_max_age_s']),
                    ('vehicle_local_position', c['local_max_age_s']), ('battery_status', 1.5)]
        if ground:
            required.append(('vehicle_land_detected', c['land_max_age_s']))
        if not all(self.fresh(n, a) for n, a in required):
            return False
        s, p, b = (self.samples[n] for n in ('vehicle_status', 'vehicle_local_position', 'battery_status'))
        return (s.system_id == 8 and s.component_id == 1 and not s.failsafe and not s.failure_detector_status
                and p.heading_good_for_control and p.xy_valid and p.z_valid and p.v_xy_valid and p.v_z_valid and not p.dead_reckoning
                and all(math.isfinite(v) for v in (p.x,p.y,p.z,p.vx,p.vy,p.vz,p.eph,p.epv))
                and 0 <= p.eph <= 1 and 0 <= p.epv <= 1 and b.connected and b.warning == 0 and b.remaining > .2)

    def position(self):
        p = self.samples['vehicle_local_position']
        return ned_enu((p.x,p.y,p.z))

    def speed(self):
        p = self.samples['vehicle_local_position']
        return math.sqrt(p.vx*p.vx+p.vy*p.vy+p.vz*p.vz)

    def resets(self):
        p = self.samples['vehicle_local_position']
        return p.xy_reset_counter,p.z_reset_counter,p.heading_reset_counter

    def region(self, point):
        c = self.config
        return in_region(point,c['bounds_min'],c['bounds_max'],
                         c['body_radius_m']+c['tracking_margin_m']+c['braking_margin_m'])

    def parse(self, request):
        if request.backend != 'PX4_KNOWN_REGION' or request.mission_type != 'FLIGHT_SEQUENCE':
            raise ValueError('Unsupported mission backend/type')
        if not math.isfinite(request.timeout_s) or not 20 <= request.timeout_s <= 240:
            raise ValueError('Invalid deadline')
        params = json.loads(request.parameters_json)
        if not isinstance(params,dict):raise ValueError('Mission parameters must be an object')
        if params.get('authorization') != self.nonce:
            raise ValueError('Missing fresh supervisor authorization')
        steps = params['steps']
        if not isinstance(steps,list) or not 2 <= len(steps) <= 20:
            raise ValueError('Invalid steps')
        if any(not isinstance(s,dict) or 'type' not in s for s in steps):raise ValueError('Invalid step object')
        types = [s['type'] for s in steps]
        if types[0] != 'TAKEOFF' or types[-1] != 'LAND' or types.count('TAKEOFF') != 1 or types.count('LAND') != 1:
            raise ValueError('Sequence must take off once and end in native landing')
        home = self.position()
        for step in steps:
            kind = step['type']
            if kind == 'TAKEOFF':
                height = step['height_m']
                if isinstance(height,bool) or not math.isfinite(height) or not .8 <= height <= 3:
                    raise ValueError('Invalid takeoff height')
                target = home[0],home[1],home[2]+height
            elif kind == 'NAVIGATE':
                target = self.alignment.to_odom(finite3(step['target_map']))
                if target[2] < home[2]+.8:
                    raise ValueError('Navigation target lacks ground clearance')
            elif kind == 'HOVER':
                if not math.isfinite(step['duration_s']) or not 2 <= step['duration_s'] <= 60:
                    raise ValueError('Invalid hover duration')
                continue
            elif kind in ('RETURN','LAND'):
                continue
            else:
                raise ValueError('Unsupported flight step')
            if not self.region(target):
                raise ValueError('Target/envelope outside W0')
        return steps

    def goal(self, request):
        with self.lock:
            if self.clock_fault_latched or self.active_goal or self.reserved or self.owner != 'NONE' or not self.healthy(ground=True):
                return GoalResponse.REJECT
            status, land = self.samples['vehicle_status'], self.samples['vehicle_land_detected']
            if status.arming_state != 1 or not land.landed or not status.pre_flight_checks_pass or self.speed() > .1:
                return GoalResponse.REJECT
            try:
                self.parse(request)
                if not self.region(self.position()):
                    return GoalResponse.REJECT
                for topic in ('offboard_control_mode','trajectory_setpoint','vehicle_command'):
                    if len(self.get_publishers_info_by_topic('/px4_7/fmu/in/'+topic)) != 1:
                        return GoalResponse.REJECT
            except (ValueError,TypeError,KeyError):
                return GoalResponse.REJECT
            self.reserved = True
            return GoalResponse.ACCEPT

    def accept(self, handle):
        with self.lock:
            self.steps = self.parse(handle.request)
            self.active_goal, self.reserved = handle, False
            self.done_future=Future()
            self.home = self.position()
            self.reference = self.home
            self.yaw = float(self.samples['vehicle_local_position'].heading)
            self.reset_baseline = self.resets()
            self.initial_mode=self.samples['vehicle_status'].nav_state
            self.deadline = time.monotonic()+handle.request.timeout_s
            self.owner = 'TASK'
            self.generation += 1
            self.session = UUID(uuid=list(uuid.uuid4().bytes))
            self.step_index = -1
            self.result = None
            self.canceling = self.land_committed = False
            self.requests.clear()
            self.change('PRESTREAM')
            handle.execute()

    def cancel(self, handle):
        with self.lock:
            if handle != self.active_goal or self.land_committed or self.phase in ('FAULT','COMPLETE'):
                return CancelResponse.REJECT
            self.canceling = True
            if self.samples['vehicle_status'].arming_state == 1:
                self.result = ('CANCELED','GROUND_CANCEL',True)
                self.owner = 'NONE'
                self.change('COMPLETE')
            else:
                self.begin_stop('CANCEL_BRAKE')
            return CancelResponse.ACCEPT

    def service(self, verb, request, response):
        with self.lock:
            now=time.monotonic()
            if (not self.active_goal or request.coordinator_instance != self.instance
                    or list(request.mission_uuid.uuid) != list(self.active_goal.goal_id.uuid)):
                response.accepted=False;response.reason='STALE_IDENTITY';response.phase=self.phase
                return response
            key=bytes(request.request_id.uuid)
            if not any(key):
                response.accepted=False;response.reason='INVALID_REQUEST_ID';response.phase=self.phase
                return response
            if key in self.requests:
                oldverb, decision=self.requests[key]
                if oldverb == verb:
                    response.accepted,response.reason,response.phase=decision
                else:
                    response.accepted=False;response.reason='REQUEST_ID_CONFLICT';response.phase=self.phase
                return response
            accepted=False;reason='INVALID_PHASE'
            if verb == 'pause' and self.phase in ('NAVIGATE','RETURN','HOVER'):
                self.saved_step=self.step_index
                self.saved_hover=max(0.,self.hover_until-now) if self.phase=='HOVER' else None
                self.begin_stop('PAUSING')
                accepted=True;reason='ACCEPTED'
            elif verb == 'resume' and self.phase=='PAUSED' and now < self.pause_until and self.healthy():
                self.owner='TASK';self.generation+=1
                self.step_index=self.saved_step-1
                self.next_step()
                if self.saved_hover is not None:
                    self.hover_until=now+self.saved_hover
                accepted=True;reason='ACCEPTED'
            response.accepted,response.reason,response.phase=accepted,reason,self.phase
            self.requests[key]=(verb,(accepted,reason,self.phase))
            return response

    def pause(self,r,s): return self.service('pause',r,s)
    def resume(self,r,s): return self.service('resume',r,s)

    def begin_stop(self, phase):
        self.segment = None  # Retire old reference timeline before braking.
        self.child=UUID()
        p=self.position()
        v=ned_enu((self.samples['vehicle_local_position'].vx,self.samples['vehicle_local_position'].vy,
                   self.samples['vehicle_local_position'].vz))
        self.reference=tuple(a+b*.6 for a,b in zip(p,v))
        if not self.region(self.reference):
            self.fault('STOP_ENVELOPE_OUTSIDE_W0');return
        self.stop_until=time.monotonic()+8.
        self.change(phase)

    def next_step(self):
        self.step_index+=1
        self.child=UUID(uuid=list(uuid.uuid4().bytes))
        step=self.steps[self.step_index]
        kind=step['type']
        self.segment=None
        if kind in ('TAKEOFF','NAVIGATE','RETURN'):
            if kind=='TAKEOFF': target=(self.home[0],self.home[1],self.home[2]+step['height_m'])
            elif kind=='NAVIGATE': target=self.alignment.to_odom(step['target_map'])
            else: target=(self.home[0],self.home[1],self.reference[2])
            if not self.region(target):
                self.fault('TARGET_OUTSIDE_W0');return
            self.segment=Segment(self.position(),target)
            self.target=target
            self.segment_start=self.get_clock().now().nanoseconds/1e9
        elif kind=='HOVER':
            self.hover_until=time.monotonic()+step['duration_s']
            self.target=self.reference
        elif kind=='LAND':
            self.land_committed=True
            self.command(21)
            self.change('LAND_REQUEST');return
        self.change(kind)

    def command(self, command, p1=0., p2=0.):
        now=self.get_clock().now().nanoseconds//1000
        msg=VehicleCommand(timestamp=now,command=command,param1=float(p1),param2=float(p2),
                           target_system=8,target_component=1,source_system=250,source_component=1,from_external=True)
        self.command_pub.publish(msg)
        self.last_command=time.monotonic()
        self.commands.append(dict(command=command,param1=p1,param2=p2,ros=now/1e6,phase=self.phase))

    def stream(self):
        stamp=self.get_clock().now().nanoseconds//1000
        self.mode_pub.publish(OffboardControlMode(timestamp=stamp,position=True))
        msg=TrajectorySetpoint(timestamp=stamp,position=list(ned_enu(self.reference)),
                               velocity=[math.nan]*3,acceleration=[math.nan]*3,jerk=[math.nan]*3,
                               yaw=self.yaw,yawspeed=math.nan)
        self.setpoint_pub.publish(msg)
        self.output_count+=1

    def fault(self, reason):
        now=time.monotonic();ros=self.get_clock().now().nanoseconds/1e9
        self.diagnostics.append(dict(reason=reason,ages={n:dict(receive=now-self.received[n],source=ros-m.timestamp/1e6) for n,m in self.samples.items()},local=str(self.samples.get('vehicle_local_position')),status=str(self.samples.get('vehicle_status')),battery=str(self.samples.get('battery_status'))))
        if self.phase not in ('FAULT','COMPLETE'):
            self.owner='NONE';self.generation+=1;self.segment=None
            self.result=('ABORTED',reason,False)
            self.change('FAULT',reason)
        # Stop all automatic output. Frozen PX4 Offboard-loss policy owns fallback.

    def stable(self, target, now):
        if distance(self.position(),target) <= self.config['position_tolerance_m'] and self.speed() <= .1:
            if self.stable_since is None:self.stable_since=now
            return now-self.stable_since >= 2.
        self.stable_since=None
        return False

    def tick(self):
        with self.lock:
            now=time.monotonic();ros=self.get_clock().now().nanoseconds/1e9
            if self.clock_last is None or ros > self.clock_last:self.clock_advance=now
            backwards=self.clock_last is not None and ros < self.clock_last
            self.clock_last=ros
            if backwards or now-self.clock_advance>.5:self.clock_fault_latched=True
            self.publish_control(now)
            if not self.active_goal or self.result:
                if self.owner=='HOLD_CONTROLLER' and hasattr(self,'final_hold_until'):
                    if backwards or now-self.clock_advance>.5 or not self.healthy() or self.resets()!=self.reset_baseline:
                        self.owner='NONE';self.change('FAULT','FINAL_HOLD_HEALTH_LOST')
                    elif self.samples['vehicle_status'].nav_state!=14:
                        self.owner='NONE';self.change('FAULT','CONTROL_LOST')
                    elif now>=self.final_hold_until:
                        self.command(21);self.owner='AUTOPILOT';self.generation+=1;self.change('FINAL_NATIVE_LAND')
                    else:self.stream()
                elif self.owner=='AUTOPILOT' and self.phase=='FINAL_NATIVE_LAND' and self.fresh('vehicle_land_detected',1.2) and self.fresh('vehicle_status',.75):
                    if self.samples['vehicle_land_detected'].landed and self.samples['vehicle_status'].arming_state==1:
                        self.owner='NONE';self.generation+=1;self.change('FINAL_HOLD_CLOSED')
                return
            if self.clock_fault_latched:
                self.fault('CLOCK_FAULT');return
            if now-self.last_graph_check>=.5:
                self.last_graph_check=now
                if any(len(self.get_publishers_info_by_topic('/px4_7/fmu/in/'+t))!=1 for t in ('offboard_control_mode','trajectory_setpoint','vehicle_command')) or len(self.get_publishers_info_by_topic('/clock'))!=1:
                    self.fault('NON_UNIQUE_CONTROL_OR_CLOCK_PUBLISHER');return
            if now >= self.deadline:
                self.fault('MISSION_TIMEOUT');return
            if not self.healthy(ground=self.phase=='LANDING'):
                self.fault('STALE_OR_INVALID_AIRCRAFT_STATE');return
            if self.resets()!=self.reset_baseline:
                self.fault('LOCALIZATION_RESET');return
            status=self.samples['vehicle_status'];phase=self.phase
            if not self.region(self.position()):
                self.fault('W0_ENVELOPE_VIOLATION');return
            if phase not in ('PRESTREAM','OFFBOARD_REQUEST','ARM_REQUEST','LANDING') and status.arming_state!=2:
                self.fault('UNEXPECTED_DISARM');return
            if phase=='LANDING' and status.arming_state==1 and (not self.samples['vehicle_land_detected'].landed or self.speed()>.1):
                self.fault('UNEXPECTED_DISARM_DURING_LANDING');return
            if phase=='PRESTREAM' and status.nav_state!=self.initial_mode:
                self.fault('CONTROL_LOST_BEFORE_OFFBOARD');return
            if phase=='OFFBOARD_REQUEST' and status.nav_state not in (self.initial_mode,14):
                self.fault('CONTROL_LOST_DURING_OFFBOARD_REQUEST');return
            if phase=='LAND_REQUEST' and status.nav_state not in (14,18,20):
                self.fault('CONTROL_LOST_DURING_LAND_REQUEST');return
            if phase=='LANDING' and status.arming_state==2 and status.nav_state not in (18,20):
                self.fault('CONTROL_LOST_DURING_LANDING');return
            if phase not in ('PRESTREAM','OFFBOARD_REQUEST','LAND_REQUEST','LANDING') and status.nav_state != 14:
                self.fault('CONTROL_LOST');return
            if phase not in ('PRESTREAM','OFFBOARD_REQUEST','ARM_REQUEST','LANDING') and status.arming_state!=2:
                self.fault('UNEXPECTED_DISARM');return
            if phase=='LANDING' and status.arming_state==1 and (not self.samples['vehicle_land_detected'].landed or self.speed()>.1):
                self.fault('UNEXPECTED_DISARM_DURING_LANDING');return
            if phase=='PRESTREAM' and now-self.phase_started >= 1.5:
                self.command(176,1,6);self.change('OFFBOARD_REQUEST')
            elif phase=='OFFBOARD_REQUEST':
                if status.nav_state==14:
                    self.command(400,1);self.change('ARM_REQUEST')
                elif now-self.phase_started > 8:self.fault('OFFBOARD_REJECTED')
                elif now-self.last_command > 1:self.command(176,1,6)
            elif phase=='ARM_REQUEST':
                if status.arming_state==2:self.next_step()
                elif now-self.phase_started > 8:self.fault('ARM_REJECTED')
                elif now-self.last_command > 1:self.command(400,1)
            elif phase in ('TAKEOFF','NAVIGATE','RETURN'):
                elapsed=ros-self.segment_start
                self.reference=self.segment.at(elapsed)
                if elapsed >= self.segment.duration and self.stable(self.target,now):self.next_step()
                elif now-self.phase_started > 50:self.fault('WAYPOINT_TIMEOUT')
            elif phase=='HOVER':
                if distance(self.position(),self.target) > .4:self.fault('HOVER_DRIFT')
                elif now >= self.hover_until and self.stable(self.target,now):self.next_step()
            elif phase in ('PAUSING','CANCEL_BRAKE'):
                if self.stable(self.reference,now):
                    self.owner='HOLD_CONTROLLER';self.generation+=1
                    self.hold_ack_count=self.output_count+3
                    self.change('HOLD_HANDOFF')
                elif now > self.stop_until:self.fault('CANCEL_TIMEOUT')
            elif phase=='HOLD_HANDOFF' and self.output_count >= self.hold_ack_count:
                # Gateway is the only owner; acknowledge only after fresh stream cycles.
                if self.canceling:
                    self.final_hold_until=now+30.
                    self.result=('CANCELED','STOPPED_AND_HOLDING',True)
                    self.change('COMPLETE')
                else:
                    self.pause_until=min(now+60.,self.deadline-8.)
                    self.change('PAUSED')
            elif phase=='PAUSED' and now >= self.pause_until:self.fault('PAUSE_TIMEOUT')
            elif phase=='LAND_REQUEST':
                if status.nav_state in (18,20):self.owner='AUTOPILOT';self.generation+=1;self.change('LANDING')
                elif now-self.phase_started > 8:self.fault('LAND_MODE_TIMEOUT')
                elif now-self.last_command > 1:self.command(21)
            elif phase=='LANDING':
                land=self.samples['vehicle_land_detected']
                if land.landed and status.arming_state==1 and self.speed()<.1:
                    if self.stable_since is None:self.stable_since=now
                    if now-self.stable_since >= 2:
                        self.owner='NONE';self.generation+=1;self.result=('SUCCEEDED','LANDED_AND_DISARMED',True);self.change('COMPLETE')
                else:self.stable_since=None
            if not self.result and self.phase not in ('FAULT','LANDING'):
                self.stream()
            self.publish_status()
            p=self.position()
            self.trace.append(dict(mono=now,ros=ros,phase=self.phase,position_enu=p,reference=self.reference,
                                   speed=self.speed(),nav_state=status.nav_state,armed=status.arming_state,
                                   generation=self.generation,owner=self.owner))

    def publish_control(self,now):
        mode='UNKNOWN'
        if self.fresh('vehicle_status',.75):
            raw=self.samples['vehicle_status'].nav_state
            mode='OFFBOARD' if raw==14 else 'AUTO_LAND' if raw in (18,20) else 'OTHER'
        status=ControlStatus(coordinator_instance=self.instance,
                             session=ControlSession(session_id=self.session,generation=self.generation,owner=self.owner),
                             lease_remaining_s=.1 if self.owner in ('TASK','HOLD_CONTROLLER') else 0.,
                             final_hold_remaining_s=max(0.,getattr(self,'final_hold_until',now)-now),
                             actual_flight_mode=mode,reason=self.reason,mock=False)
        status.header.stamp=self.get_clock().now().to_msg()
        status.allowed_operations=['POSITION_REFERENCE'] if self.owner=='TASK' else ['HOLD_REFERENCE'] if self.owner=='HOLD_CONTROLLER' else []
        self.control_pub.publish(status)

    def publish_status(self):
        s=TaskStatus(mission_uuid=self.active_goal.goal_id,coordinator_instance=self.instance,
                     event_sequence=self.sequence,phase=self.result[0] if self.result else self.phase,result_code=self.result[0] if self.result else '',
                     reason=self.result[1] if self.result else self.reason,control_session=ControlSession(session_id=self.session,generation=self.generation,owner=self.owner),
                     child_uuid=self.child,has_child=any(self.child.uuid),
                     map_session=self.config['alignment_id'],total_remaining_s=max(0.,self.deadline-time.monotonic()),mock=False)
        s.header.stamp=self.get_clock().now().to_msg()
        self.status_pub.publish(s)
        if not self.result:self.active_goal.publish_feedback(ExecuteMission.Feedback(status=s,tree_node='PX4_FLIGHT_SEQUENCE',fault=self.reason))

    async def execute(self,handle):
        code,reason,confirmed=await self.done_future
        with self.lock:
            result=ExecuteMission.Result(result_code=code,reason=reason,cleanup_confirmed=confirmed,mock=False)
            if code=='SUCCEEDED':handle.succeed()
            elif code=='CANCELED':handle.canceled()
            else:handle.abort()
            self.active_goal=None
            return result


def main(args=None):
    from rclpy.executors import SingleThreadedExecutor
    rclpy.init(args=args)
    node=FlightServer()
    executor=SingleThreadedExecutor();executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:
        executor.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
