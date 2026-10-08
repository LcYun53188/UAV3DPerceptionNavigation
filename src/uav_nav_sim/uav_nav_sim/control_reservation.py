"""Parent navigation reservation and tick lease; never publishes velocity."""
import math
import time
from rclpy.qos import QoSProfile, DurabilityPolicy
from uav_nav_interfaces.msg import ControlStatus, MissionProgress
from uav_nav_interfaces.srv import ManageControlSession
from .navigation import stamp_key


def session_key(session):
    return (bytes(session.session_id.uuid), session.generation, session.owner)


class ControlReservation:
    def __init__(self, node, action):
        self.node, self.action = node, action
        self.session = None
        self.fault = ''
        self.seen_owners = {}
        self.stable_since = None
        self.stamp = None
        self.advanced = time.monotonic()
        self.last_status = 0.
        self.publisher = node.create_publisher(ControlStatus, '/uav/algorithm/control_status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.service = node.create_service(ManageControlSession,
            '/uav/algorithm/control_session', self.manage)
        self.subscription = node.create_subscription(MissionProgress,
            '/uav/algorithm/mission_progress', self.progress, 10)

    def matches(self, session):
        return self.session is not None and session_key(session) == session_key(self.session)

    def permits(self, session):
        if self.session is None:
            return session.owner not in self.seen_owners
        return (self.matches(session) and self.current_map() == self.map_session and
                not self.fault and self.sequence > 0 and
                time.monotonic() < self.deadline)

    def manage(self, request, response):
        now = time.monotonic()
        session = request.control_session
        if request.operation == 'ACQUIRE':
            if self.matches(session) and request.map_session == self.map_session:
                response.accepted, response.reason = not bool(self.fault), self.fault
                return response
            nav = self.node.navigation
            if (self.session is not None or self.action.busy or nav.goal is not None or
                    nav.autonomous.enabled):
                response.reason = 'BUSY'
            elif (not any(session.session_id.uuid) or not session.owner.strip() or
                  session.generation <= self.seen_owners.get(session.owner, 0)):
                response.reason = 'INVALID_OR_RETIRED_SESSION'
            elif len(self.seen_owners) >= 256 and session.owner not in self.seen_owners:
                response.reason = 'OWNER_LIMIT'
            elif (not self.node.ready() or not self.current_odom_valid(now) or
                  request.map_session != self.current_map()):
                response.reason = 'MAP_OR_ODOMETRY_NOT_READY'
            else:
                self.session = session
                self.map_session = request.map_session
                self.seen_owners[session.owner] = session.generation
                self.sequence = 0
                self.deadline = now+2.  # No navigation before first actual tree tick.
                self.fault = ''
                response.accepted = True
        elif request.operation == 'RELEASE':
            if not self.matches(session):
                response.reason = 'SESSION_MISMATCH'
            elif (self.action.reserved or self.stable_since is None or
                  now-self.stable_since < .6 or not self.current_odom_valid(now)):
                response.reason = 'STOP_UNCONFIRMED'
            else:
                response.reason = self.fault
                status = ControlStatus()
                status.header.stamp = self.node.get_clock().now().to_msg()
                status.session = self.session
                status.coordinator_instance = self.session.owner
                status.reason = 'RELEASED'
                status.actual_flight_mode = 'ALGORITHM_VELOCITY_MODEL'
                self.publisher.publish(status)
                self.session = None
                self.fault = ''
                response.accepted = True
        else:
            response.reason = 'INVALID_OPERATION'
        return response

    def progress(self, message):
        if self.session is None or self.fault:
            return
        now = time.monotonic()
        if now >= self.deadline:
            self.fail('BT_PROGRESS_TIMEOUT')
            return
        if (bytes(message.mission_uuid.uuid) == bytes(self.session.session_id.uuid) and
                message.coordinator_instance == self.session.owner and
                message.tick_sequence > self.sequence):
            self.sequence = message.tick_sequence
            self.deadline = now+.5

    def current_map(self):
        return f'{self.node.session[0]}:{self.node.session[1]}' if self.node.session else ''

    def current_odom_valid(self, now):
        odom = self.node.odom
        if odom is None:
            return False
        p, v, w = odom.pose.pose.position, odom.twist.twist.linear, odom.twist.twist.angular
        values = [p.x, p.y, p.z, v.x, v.y, v.z, w.x, w.y, w.z]
        age = (self.node.get_clock().now().nanoseconds-stamp_key(odom.header.stamp))/1e9
        return (all(math.isfinite(x) for x in values) and 0 <= age <= .5 and
                0 <= now-self.node.odom_wall <= .5 and now-self.advanced < .5)

    def fail(self, reason):
        if self.fault:
            return
        self.fault = reason
        if self.action.active_goal is not None:
            self.action.finish('ABORTED', reason)
        else:
            self.node.stop('CANCELLED')
            self.node.navigation.report('STOPPED', reason)

    def tick(self):
        now = time.monotonic()
        odom = self.node.odom
        if odom is not None:
            stamp = stamp_key(odom.header.stamp)
            if self.stamp is None or stamp > self.stamp:
                self.advanced = now
            elif stamp < self.stamp and self.session is not None:
                self.fail('CLOCK_RESET')
            self.stamp = stamp
        valid = self.current_odom_valid(now)
        if (valid and self.node.curve is None and self.node.navigation.goal is None and
                math.hypot(odom.twist.twist.linear.x, odom.twist.twist.linear.y,
                           odom.twist.twist.linear.z) <= .05 and
                math.hypot(odom.twist.twist.angular.x, odom.twist.twist.angular.y,
                           odom.twist.twist.angular.z) <= .1):
            if self.stable_since is None:
                self.stable_since = now
        else:
            self.stable_since = None
        if self.session is None:
            return
        if not self.fault:
            if now >= self.deadline:
                self.fail('BT_PROGRESS_TIMEOUT')
            elif self.current_map() != self.map_session:
                self.fail('MAP_SESSION_CHANGED')
            elif not valid:
                self.fail('ODOMETRY_OR_CLOCK_FAULT')
            elif not self.node.ready():
                self.fail('MAP_OR_ODOMETRY_NOT_READY')
        if now-self.last_status >= .1:
            self.last_status = now
            status = ControlStatus()
            status.header.stamp = self.node.get_clock().now().to_msg()
            status.session = self.session
            status.coordinator_instance = self.session.owner
            status.lease_remaining_s = max(0., self.deadline-now)
            status.reason = self.fault
            status.actual_flight_mode = 'ALGORITHM_VELOCITY_MODEL'
            status.allowed_operations = ['HOLD']
            if self.sequence > 0 and not self.fault:
                status.allowed_operations.append('NAVIGATE')
            self.publisher.publish(status)
