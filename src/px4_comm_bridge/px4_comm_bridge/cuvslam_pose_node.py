"""Reviewed SDK pose normalization and terminal source retirement on reset.

ROS pose/status only. Never creates an FMU publisher or invents velocity.
"""
import re
import time
import uuid
from collections import OrderedDict
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.parameter_client import AsyncParameterClient
from rclpy.parameter import parameter_value_to_python
from geometry_msgs.msg import PoseWithCovarianceStamped
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from isaac_ros_visual_slam_interfaces.srv import Reset
from uav_nav_interfaces.msg import VioStatus
from .vio_input import SourceContinuity, stamp_s
from .cuvslam_pose import CONTRACT, SOURCE_PARAMETERS, normalize_pose
from .source_timing import SourceTiming


class CuvslamPose(Node):
    pose_topic = '/visual_slam/tracking/vo_pose_covariance'
    tracking_topic = '/visual_slam/status'

    def __init__(self):
        super().__init__('cuvslam_pose')
        self.declare_parameter('timing_path','')
        self.timing_path=self.get_parameter('timing_path').value
        self.timing=SourceTiming() if self.timing_path else None
        self.declare_parameter('calibration_id','')
        self.declare_parameter('source_contract','unverified')
        self.calibration = self.get_parameter('calibration_id').value
        if not re.fullmatch('[0-9a-f]{64}',self.calibration):
            raise ValueError('calibration_id must bind reviewed sensor/extrinsics/config SHA256')
        if self.get_parameter('source_contract').value != CONTRACT:
            raise ValueError('VIO_SOURCE_CONTRACT_UNVERIFIED')
        self.session = str(uuid.uuid4())
        self.counter = 0
        self.continuity = SourceContinuity()
        self.bound = False
        self.stable_since = None
        self.tracking = self.tracking_receive = self.tracking_gid = None
        self.sample = self.receive = None
        self.reason = 'VIO_MISSING'
        self.fault = ''
        self.verified_parameters = None
        self.parameters_future = None
        self.pending_reset = None
        self.reset_deadline = None
        self.last_clock = None
        self.pending = OrderedDict()
        self.tracking_samples = OrderedDict()
        self.clients_group = ReentrantCallbackGroup()
        self.parameters_client = AsyncParameterClient(self,'/visual_slam',callback_group=self.clients_group)
        self.reset_client = self.create_client(Reset,'/visual_slam/internal/reset',callback_group=self.clients_group)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped,'/uav/vio/pose',10)
        self.status_pub = self.create_publisher(VioStatus,'/uav/vio/pose_status',10)
        self.create_subscription(PoseWithCovarianceStamped,self.pose_topic,self.on_pose,10)
        self.create_subscription(VisualSlamStatus,self.tracking_topic,self.on_tracking,10)
        self.create_service(Reset,'/uav/vio/reset',self.on_reset)
        self.create_timer(.05,self.tick,callback_group=self.clients_group,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

    def trace(self,stage,sample=None,info=None,**details):
        if self.timing is not None:
            self.timing.record(stage,self.get_clock().now().nanoseconds/1e9,sample,info,**details)

    def retire(self, reason):
        if not self.fault:self.trace('retire',stamp_s(self.sample) if self.sample is not None else None,reason=reason)
        if not self.fault: self.get_logger().warning('Source retired: '+reason)
        self.fault = self.fault or reason
        self.reason = self.fault
        self.publish_status()

    def reject(self,reason):
        if self.bound:
            self.retire(reason)
        else:
            self.continuity = SourceContinuity()
            self.stable_since = None
            self.sample = self.receive = None
            self.reason = reason

    def publisher(self, topic):
        endpoints = self.get_publishers_info_by_topic(topic)
        if len(endpoints) != 1:
            raise ValueError('VIO_WRITER_COUNT')
        return bytes(endpoints[0].endpoint_gid).hex()

    def validate_tracking(self, sample=None, pair=None):
        now = self.get_clock().now().nanoseconds/1e9
        if self.last_clock is not None and now < self.last_clock:
            raise ValueError('VIO_CLOCK_RESET')
        self.last_clock = now
        if self.verified_parameters is None:
            raise ValueError('VIO_SOURCE_PARAMETERS_UNVERIFIED')
        if self.fault:
            raise ValueError(self.fault)
        tracking,received = pair if pair is not None else (self.tracking,self.tracking_receive)
        if (tracking is None or tracking.vo_state != 1
                or time.monotonic()-received > .2
                or not -.05 <= now-stamp_s(tracking.header.stamp) <= .2
                or (sample is not None and abs(sample-stamp_s(tracking.header.stamp)) > .001)):
            if self.bound and not self.fault:
                self.get_logger().warning('Tracking rejected: '+str(dict(ros=now,
                    tracking_stamp=stamp_s(tracking.header.stamp) if tracking is not None else None,
                    receive_gap=time.monotonic()-received if received is not None else None,
                    vo_state=tracking.vo_state if tracking is not None else None,sample=sample)))
            raise ValueError('VIO_TRACKING_INVALID')
        if self.publisher(self.tracking_topic) != self.tracking_gid:
            raise ValueError('VIO_PUBLISHER_CHANGED')
        if len(self.get_publishers_info_by_topic('/uav/vio/pose')) != 1:
            raise ValueError('VIO_OUTPUT_WRITER_COUNT')

    def on_tracking(self, message, info):
        started=time.monotonic()
        self.trace('tracking_rx',stamp_s(message.header.stamp),info)
        try:
            gid = self.publisher(self.tracking_topic)
            if self.bound and self.tracking_gid != gid:
                raise ValueError('VIO_PUBLISHER_CHANGED')
            self.tracking_gid = gid
            self.tracking,self.tracking_receive = message,time.monotonic()
            self.tracking_samples[stamp_s(message.header.stamp)] = (message,self.tracking_receive)
            while len(self.tracking_samples)>100: self.tracking_samples.popitem(last=False)
            if message.vo_state != 1:
                raise ValueError('VIO_TRACKING_INVALID')
            self.drain()
        except ValueError as exc:
            self.reject(str(exc))
        finally:self.trace('tracking_done',stamp_s(message.header.stamp),elapsed_s=time.monotonic()-started)

    def on_pose(self,message,info):
        started=time.monotonic()
        self.trace('pose_rx',stamp_s(message.header.stamp),info)
        if self.fault: return
        try:
            gid = self.publisher(self.pose_topic)
            if self.bound and gid != self.continuity.publisher:
                raise ValueError('VIO_PUBLISHER_CHANGED')
            key = stamp_s(message.header.stamp)
            if key in self.pending or (self.continuity.last_stamp is not None and key<=self.continuity.last_stamp):
                raise ValueError('VIO_TIME_DISCONTINUITY')
            if len(self.pending)>=20: raise ValueError('VIO_PAIR_QUEUE_OVERFLOW')
            self.pending[key] = (message,time.monotonic())
            self.drain()
        except ValueError as exc: self.reject(str(exc))
        finally:self.trace('pose_done',stamp_s(message.header.stamp),elapsed_s=time.monotonic()-started)

    def drain(self):
        for key in sorted(self.pending):
            message,received = self.pending[key]
            if time.monotonic()-received > .2:
                del self.pending[key]
                self.reject('VIO_TRACKING_PAIR_STALE')
            elif key in self.tracking_samples:
                del self.pending[key]
                self.accept_pose(message,self.tracking_samples[key])
            else: break
            if self.fault:
                self.pending.clear()
                break

    def accept_pose(self,message,pair):
        if self.fault: return
        try:
            if self.receive is not None and time.monotonic()-self.receive > .2:
                raise ValueError('VIO_RECEIVE_GAP')
            normalized = normalize_pose(message,self.get_clock().now().nanoseconds/1e9)
            self.validate_tracking(stamp_s(message.header.stamp),pair)
            self.continuity.accept(message,self.publisher(self.pose_topic))
            self.sample,self.receive = message.header.stamp,time.monotonic()
            if self.stable_since is None: self.stable_since = self.receive
            if not self.bound and self.receive-self.stable_since < 2.:
                self.reason = 'VIO_STABILIZING'
                self.publish_status()
                return
            self.bound = True
            self.reason = ''
            self.pose_pub.publish(normalized)
            self.trace('pose_emit',stamp_s(normalized.header.stamp))
        except ValueError as exc:
            self.reject(str(exc))
        self.publish_status()

    async def on_reset(self,request,response):
        # Retire BEFORE forwarding, regardless of whether the SDK resets successfully.
        self.counter = (self.counter+1)%256
        self.retire('VIO_RESET_REQUESTED')
        if not self.reset_client.service_is_ready():
            response.success = False
            return response
        future = self.reset_client.call_async(request)
        self.pending_reset,self.reset_deadline = future,time.monotonic()+5.
        try:
            upstream = await future
            response.success = upstream is not None and upstream.success
        except Exception:
            response.success = False
        finally:
            self.pending_reset = self.reset_deadline = None
        return response

    def tick(self):
        if self.pending_reset is not None and time.monotonic() > self.reset_deadline:
            self.pending_reset.cancel()
        if self.verified_parameters is None and not self.fault:
            if self.parameters_future is None and self.parameters_client.services_are_ready():
                self.parameters_future = self.parameters_client.get_parameters(list(SOURCE_PARAMETERS))
            if self.parameters_future is not None and self.parameters_future.done():
                try:
                    response = self.parameters_future.result()
                    values = dict(zip(SOURCE_PARAMETERS,[parameter_value_to_python(v) for v in response.values]))
                    if values != SOURCE_PARAMETERS: raise ValueError('VIO_SOURCE_PARAMETERS_INVALID')
                    if any(name=='/visual_slam/reset' for name,_ in self.get_service_names_and_types()):
                        raise ValueError('VIO_RESET_PROXY_BYPASSED')
                    self.verified_parameters = values
                except Exception:
                    self.retire('VIO_SOURCE_PARAMETERS_INVALID')
        self.drain()
        self.publish_status()

    def publish_status(self):
        reason = self.fault or self.reason
        try:
            self.validate_tracking()
            if (self.receive is None or time.monotonic()-self.receive > .2
                    or not -.05 <= self.get_clock().now().nanoseconds/1e9-stamp_s(self.sample) <= .2):
                raise ValueError('VIO_SAMPLE_STALE')
            if self.publisher(self.pose_topic) != self.continuity.publisher:
                raise ValueError('VIO_PUBLISHER_CHANGED')
        except ValueError as exc:
            reason = reason or str(exc)
            if self.bound and not self.fault:
                self.fault = reason
                self.trace('retire',stamp_s(self.sample) if self.sample is not None else None,reason=reason)
                self.get_logger().warning('Source retired: '+reason)
        if not self.bound: reason = reason or 'VIO_STABILIZING'
        status = VioStatus(localization_session=self.session,calibration_id=self.calibration,
            reset_counter=self.counter,valid=not bool(reason),reason=reason)
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = 'odom'
        if self.sample is not None: status.sample_stamp = self.sample
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = CuvslamPose()
    try: rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException): pass
    finally:
        if node.timing is not None:node.timing.write(node.timing_path)
        node.destroy_node()
        rclpy.try_shutdown()
