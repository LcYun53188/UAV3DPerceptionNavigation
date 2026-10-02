"""Gazebo velocity-model executor. Deliberately has no PX4 dependency or output."""
import math
import time
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from uav_nav_interfaces.msg import MapSnapshot, TimedTrajectory
from .core import grid_from_message, spline, validate_trajectory, validate_handover
from .navigation import GoalManager
from .attitude import level_body_rates


class Executor(Node):
    def __init__(self):
        super().__init__('gazebo_trajectory_executor')
        for key, value in [('body_radius', 0.3), ('map_timeout', 2.0), ('max_velocity', 0.5),
                           ('max_acceleration', 1.0), ('max_jerk', 2.0), ('tracking_error', 0.15)]:
            self.declare_parameter(key, value)
        self.declare_parameter('managed_goals', False)
        self.radius = self.get_parameter('body_radius').value
        self.limits = [self.get_parameter(k).value for k in ('max_velocity','max_acceleration','max_jerk')]
        self.timeout = self.get_parameter('map_timeout').value
        self.grid = self.map = self.odom = self.curve = self.trajectory = None
        self.pending = None
        self.map_wall = self.odom_wall = 0.0
        self.last_id = 0
        self.session = None
        self.last_ros = None
        self.state = 'HOLD'
        self.command = self.create_publisher(Twist, '/cmd_vel', 1)
        self.status = self.create_publisher(String, '/uav/executor/state', 10)
        self.event = self.create_publisher(String, '/uav/executor/event', 10)
        self.navigation = GoalManager(self) if self.get_parameter('managed_goals').value else None
        self.create_subscription(MapSnapshot, '/uav/map/snapshot', self.map_cb,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, '/uav/localization/odometry', self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(TimedTrajectory, '/uav/trajectory', self.trajectory_cb, 10)
        self.create_service(Trigger, '/uav/cancel', self.cancel)
        self.create_timer(0.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def stop(self, reason):
        self.curve = self.trajectory = None
        self.pending = None
        self.state = 'HOLD'
        self.command.publish(Twist())
        if self.navigation is not None:
            if reason == 'GOAL_REACHED' and not self.navigation.local_final:
                reason = 'LOCAL_GOAL_REACHED'
            self.navigation.on_stop(reason)
        self.event.publish(String(data=reason))
        if reason not in ('GOAL_REACHED', 'LOCAL_GOAL_REACHED', 'GOAL_REPLACED'):
            self.get_logger().warning(reason)

    def odom_cb(self, msg):
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
            return
        self.odom = msg
        self.odom_wall = time.monotonic()

    def map_cb(self, msg):
        session = (msg.map_id, msg.epoch)
        if self.session is not None and (msg.epoch < self.session[1] or
                (msg.epoch == self.session[1] and msg.map_id != self.session[0])):
            return  # Delayed responses must never roll the session backward.
        if self.session != session:
            if self.session is not None:
                self.stop('MAP_SESSION_CHANGED')
            self.last_id = 0
            self.session = session
        if not msg.valid:
            self.map = self.grid = None
            if self.curve is not None or (getattr(self, 'navigation', None) is not None and
                    (self.navigation.goal is not None or self.navigation.autonomous.enabled)):
                self.stop('MAP_INVALID')
            return
        if self.map is not None and session == (self.map.map_id, self.map.epoch) and msg.version <= self.map.version:
            return
        try:
            self.grid = grid_from_message(msg)
            self.map = msg
            self.map_wall = time.monotonic()
        except ValueError as exc:
            self.map = self.grid = None
            self.stop(f'MAP_INVALID: {exc}')
            return
        if self.curve is not None:
            try:
                t = max(0.0, (self.get_clock().now()-Time.from_msg(self.trajectory.start_time)).nanoseconds/1e9)
                validate_trajectory(self.curve, self.grid, self.radius, self.limits,
                                    start=min(t, float(self.curve.t[-4])))
            except ValueError as exc:
                # The new map is valid; only the old remaining path is unsafe.
                # Keep it available for a stopped, bounded replan.
                self.stop(f'MAP_RECHECK_FAILED: {exc}')
        if getattr(self, 'pending', None) is not None:
            try:
                validate_trajectory(self.pending[0], self.grid, self.radius, self.limits)
            except ValueError as exc:
                self.pending = None
                self.navigation.replan_failed(f'PENDING_MAP_RECHECK_FAILED: {exc}')

    def ready(self):
        if self.map is None or self.grid is None or self.odom is None:
            return False
        now = self.get_clock().now()
        return (0 <= (now-Time.from_msg(self.map.header.stamp)).nanoseconds/1e9 <= self.timeout and
                0 <= (now-Time.from_msg(self.odom.header.stamp)).nanoseconds/1e9 <= 0.5 and
                time.monotonic()-self.map_wall <= self.timeout and time.monotonic()-self.odom_wall <= 0.5)

    def trajectory_cb(self, msg):
        if self.navigation is not None and not self.navigation.accepts(msg):
            self.event.publish(String(data='REJECTED:OBSOLETE_GOAL'))
            return
        try:
            moving = msg.parent_trajectory_id != 0
            if not self.ready() or (self.curve is not None and not moving):
                now = self.get_clock().now()
                ages = {name: None if msg is None else
                        (now-Time.from_msg(msg.header.stamp)).nanoseconds/1e9
                        for name, msg in [('map', self.map), ('odometry', self.odom)]}
                raise ValueError(f'Not ready or already executing: state={self.state}, ages={ages}')
            if (msg.header.frame_id != 'odom' or (msg.map_id,msg.epoch) != self.session or
                    msg.map_version > self.map.version or msg.trajectory_id <= self.last_id):
                raise ValueError('Frame/session/version/trajectory ID rejected')
            delay = (Time.from_msg(msg.start_time)-self.get_clock().now()).nanoseconds/1e9
            if not 0.02 <= delay <= 2.0:
                raise ValueError('Start time late or too early')
            curve = spline([[p.x,p.y,p.z] for p in msg.control_points], msg.knot_interval)
            limits = [msg.max_velocity, msg.max_acceleration, msg.max_jerk]
            if any(not np.isfinite(v) or v <= 0 or v > ceiling for v,ceiling in zip(limits,self.limits)):
                raise ValueError('Invalid declared limits')
            pos = self.odom.pose.pose.position
            vel = self.odom.twist.twist.linear
            if moving:
                if (self.navigation is None or self.curve is None or self.pending is not None or
                        self.trajectory.trajectory_id != msg.parent_trajectory_id):
                    raise ValueError('Handover parent is not active')
                offset = (Time.from_msg(msg.start_time)-Time.from_msg(self.trajectory.start_time)).nanoseconds/1e9
                validate_handover(self.curve, curve, offset)
            elif (np.linalg.norm(curve(0)-[pos.x,pos.y,pos.z])>0.1 or
                    np.linalg.norm([vel.x,vel.y,vel.z])>0.05 or np.linalg.norm(curve(0,1))>1e-5):
                raise ValueError('Discontinuous takeover')
            validate_trajectory(curve,self.grid,self.radius,limits)
            if (Time.from_msg(msg.start_time)-self.get_clock().now()).nanoseconds <= 0:
                raise ValueError('Validation missed start time')
            if moving:
                self.pending = (curve,msg)
                self.last_id = msg.trajectory_id
                self.event.publish(String(data=f'QUEUED:{msg.trajectory_id}'))
                return
            self.curve,self.trajectory = curve,msg
            self.last_id = msg.trajectory_id
            self.state = 'EXECUTING'
            if self.navigation is not None:
                self.navigation.accepted()
            self.event.publish(String(data=f'ACCEPTED:{msg.trajectory_id}'))
        except ValueError as exc:
            self.event.publish(String(data=f'REJECTED:{exc}'))
            self.get_logger().warning(f'Trajectory rejected: {exc}')
            if self.navigation is not None:
                if msg.parent_trajectory_id:
                    self.navigation.replan_failed(f'REJECTED:{exc}')
                else:
                    self.navigation.failed_segment(f'REJECTED:{exc}')

    def cancel(self, _, response):
        self.stop('CANCELLED')
        response.success = True
        return response

    def publish_hold(self, command):
        # Stopping translation must not freeze an old observation tilt. Level
        # using fresh odometry even if the map is unavailable or the goal ended.
        # With stale/invalid pose, publish zero rather than an open-loop turn.
        now = self.get_clock().now()
        if (self.odom is None or time.monotonic()-self.odom_wall > .5 or
                not 0 <= (now-Time.from_msg(self.odom.header.stamp)).nanoseconds/1e9 <= .5):
            self.command.publish(Twist())
            return
        q = self.odom.pose.pose.orientation
        try:
            quaternion = np.array([q.x, q.y, q.z, q.w])
            if not np.all(np.isfinite(quaternion)) or np.linalg.norm(quaternion) < 1e-6:
                raise ValueError('Invalid orientation')
            rates = level_body_rates(Rotation.from_quat(quaternion), command.angular.z)
        except ValueError:
            self.command.publish(Twist())
            return
        command.angular.x, command.angular.y, command.angular.z = map(float, rates)
        self.command.publish(command)

    def tick(self):
        now = self.get_clock().now()
        ns = now.nanoseconds
        if self.last_ros is not None and ns < self.last_ros:
            self.stop('CLOCK_RESET')
        self.last_ros = ns
        self.status.publish(String(data=self.state))
        if self.curve is None:
            command = self.navigation.tick() if self.navigation is not None else Twist()
            self.publish_hold(command)
            return
        if not self.ready():
            self.stop('STALE_MAP_OR_ODOMETRY')
            return
        if self.pending is not None and ns >= Time.from_msg(self.pending[1].start_time).nanoseconds:
            curve, msg = self.pending
            lateness = (ns-Time.from_msg(msg.start_time).nanoseconds)/1e9
            p = self.odom.pose.pose.position
            old_t = (now-Time.from_msg(self.trajectory.start_time)).nanoseconds/1e9
            if (lateness > .1 or not self.navigation.accepts(msg) or
                    self.trajectory.trajectory_id != msg.parent_trajectory_id or
                    np.linalg.norm(self.curve(min(old_t,float(self.curve.t[-4])))-[p.x,p.y,p.z]) > .1):
                self.pending = None
                self.navigation.replan_failed('HANDOVER_MISSED_OR_TRACKING')
            else:
                self.curve, self.trajectory = curve, msg
                self.pending = None
                self.navigation.handover()
                self.event.publish(String(data=f'HANDOVER:{msg.parent_trajectory_id}->{msg.trajectory_id}'))
                self.event.publish(String(data=f'ACCEPTED:{msg.trajectory_id}'))
        t = (now-Time.from_msg(self.trajectory.start_time)).nanoseconds/1e9
        if t < 0:
            self.publish_hold(Twist())
            return
        duration = float(self.curve.t[-4])
        q = self.odom.pose.pose.orientation
        pos = self.odom.pose.pose.position
        current = np.array([pos.x,pos.y,pos.z])
        target = self.curve(min(t,duration))
        error = target-current
        if np.linalg.norm(error)>self.get_parameter('tracking_error').value:
            self.stop('TRACKING_ERROR')
            return
        if self.grid.collision(current,self.radius):
            self.stop('CURRENT_VOLUME_BLOCKED')
            return
        if t >= duration and np.linalg.norm(error)<0.04:
            self.stop('GOAL_REACHED')
            return
        if self.navigation is not None:
            self.navigation.inflight_tick(t, duration)
        velocity = self.curve(min(t,duration),1)+2.0*error
        speed = np.linalg.norm(velocity)
        if speed > self.limits[0]:
            velocity *= self.limits[0]/speed
        try:
            rotation = Rotation.from_quat([q.x,q.y,q.z,q.w])
        except ValueError:
            self.stop('INVALID_ORIENTATION')
            return
        # Gazebo model LinearVelocityCmd is body-relative; map/odom are world ENU.
        body = rotation.inv().apply(velocity)
        command = Twist()
        command.linear.x,command.linear.y,command.linear.z = map(float,body)
        _, _, yaw = rotation.as_euler('xyz')
        yaw_rate = 0.0
        if np.linalg.norm(velocity[:2])>0.05:
            desired = math.atan2(velocity[1],velocity[0])
            yaw_rate = float(np.clip(2*math.atan2(math.sin(desired-yaw),math.cos(desired-yaw)), -0.6,0.6))
        command.angular.x, command.angular.y, command.angular.z = map(float, level_body_rates(rotation, yaw_rate))
        self.command.publish(command)


def main():
    rclpy.init()
    node = Executor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.command.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
