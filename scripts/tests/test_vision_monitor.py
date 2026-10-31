"""Isolated ROS/Tk integration: real DDS images, delayed detections and historical TF."""
import os
import sys
import time
from copy import deepcopy
from types import SimpleNamespace
from pathlib import Path
import tkinter as tk

import pytest

pytest.importorskip('rclpy')
if not os.environ.get('DISPLAY'):
    pytest.skip('Desktop display required', allow_module_level=True)

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path as RosPath
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from uav_nav_interfaces.msg import PlanningContext, TimedTrajectory
from tf2_ros import StaticTransformBroadcaster

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'simulation/vio/runtime'))
from vision_monitor import VisionMonitor


def test_live_ros_images_ar_then_stale_clock_and_context_invalidation(tmp_path):
    rclpy.init()
    source = rclpy.create_node('vision_monitor_test_source')
    root = tk.Tk()
    monitor = None
    try:
        monitor = VisionMonitor(root, tmp_path)
        root.update()
        clock = source.create_publisher(Clock, '/clock', 10)
        context_pub = source.create_publisher(PlanningContext, '/planning/context', 10)
        info_pub = source.create_publisher(CameraInfo, '/px4_depth/camera_info', 10)
        path_pub = source.create_publisher(RosPath, '/planning/ego/path', 10)
        trajectory_pub = source.create_publisher(TimedTrajectory, '/planning/ego/trajectory', 10)
        detection_pub = source.create_publisher(Detection2DArray, '/uav/people/detections', 10)
        image_pubs = {key: source.create_publisher(Image, topic, 10) for key, topic in
                      [('rgb','/px4_rgb/image'),('left','/vio/left/image'),('right','/vio/right/image')]}
        c = Clock()
        c.clock.sec = 10
        context = PlanningContext()
        context.header.stamp.sec = 10
        context.header.frame_id = 'map'
        context.map_id, context.context_id = 'test-map', 'test-context'
        context.localization_session, context.alignment_id = 'test-localization', 'test-alignment'
        context.map_epoch = context.alignment_generation = 1
        context.valid, context.reason = True, 'OK'
        info = CameraInfo()
        info.header.stamp.sec = 10
        info.header.frame_id = 'test_camera'
        info.width = info.height = 200
        info.k = [100.,0.,100.,0.,100.,100.,0.,0.,1.]
        path = RosPath()
        path.header.stamp.sec = 10
        path.header.frame_id = 'map'
        trajectory = TimedTrajectory()
        trajectory.header = path.header
        trajectory.map_id, trajectory.epoch, trajectory.trajectory_id = 'test-map', 1, 1
        for x in (0.,1.):
            p = PoseStamped()
            p.header = path.header
            p.pose.position.x, p.pose.position.z = x, 2.
            path.poses.append(p)
        image = Image()
        image.header = info.header
        image.width = image.height = 200
        image.encoding, image.step = 'rgb8', 600
        image.data = bytes([10,20,30]) * 40000
        detection = Detection2DArray()
        detection.header = info.header
        item = Detection2D()
        item.bbox.center.position.x = item.bbox.center.position.y = 90.
        item.bbox.size_x = item.bbox.size_y = 50.
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id, hypothesis.hypothesis.score = 'person', .9
        item.results.append(hypothesis)
        detection.detections.append(item)
        broadcaster = StaticTransformBroadcaster(source)
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'map', 'test_camera'
        transform.transform.rotation.w = 1.
        broadcaster.sendTransform(transform)

        def pump(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                clock.publish(c)
                rclpy.spin_once(source, timeout_sec=.005)
                root.update()

        pump(.4)
        context_pub.publish(context)
        info_pub.publish(info)
        for pub in image_pubs.values():
            pub.publish(image)
        pump(.15)
        path_pub.publish(path)
        trajectory_pub.publish(trajectory)
        detection_pub.publish(detection)
        pump(.2)
        assert len(monitor.hitboxes) == 1, monitor.notice
        x1,y1,x2,y2,_ = monitor.hitboxes[0]
        monitor.select(SimpleNamespace(x=(x1+x2)/2, y=(y1+y2)/2))
        assert monitor.selection.state == 'SELECTED'
        assert '规划参考路线' in monitor.ar_reason, monitor.ar_reason
        assert 'rgb' in monitor.rendered
        assert monitor.rendered['rgb'][100,120].tolist() != [30,20,10]  # projected segment
        assert (tmp_path/'flight-images/0001.json').exists()
        monitor.snapshot()
        assert len(list((tmp_path/'vision-snapshots').glob('*.png'))) == 3
        # Monitor has no application publishers (rclpy's own rosout/parameter events excepted).
        own = monitor.node.get_publisher_names_and_types_by_node(monitor.node.get_name(), '/')
        assert all(topic in ('/rosout','/parameter_events','/uav/people/follow/selection') for topic, _ in own)

        # Newer RGB arrives before inference. Display the source frame matching the delayed detection.
        newer = Image()
        newer.header.frame_id = 'test_camera'
        newer.header.stamp.sec = 10
        newer.header.stamp.nanosec = 100000000
        newer.width = newer.height = 200
        newer.encoding, newer.step, newer.data = image.encoding, image.step, image.data
        c.clock.nanosec = 100000000
        image_pubs['rgb'].publish(newer)
        pump(.15)
        assert len(monitor.hitboxes) == 1
        assert monitor.image_serial['rgb'][1] == 10.
        next_detection = deepcopy(detection)
        next_detection.header.stamp.nanosec = 100000000
        next_detection.detections[0].bbox.center.position.x += 5.
        detection_pub.publish(next_detection)
        pump(.15)
        assert monitor.selection.state == 'SELECTED'
        assert monitor.selection.serial == ('test_camera', 10.1)

        # A path without a matching map-bound trajectory must never appear as current AR.
        trajectory.header.stamp.nanosec = 100000000
        trajectory.map_id = 'wrong-map'
        path.header.stamp.nanosec = 100000000
        trajectory_pub.publish(trajectory)
        path_pub.publish(path)
        pump(.15)
        assert '身份不匹配' in monitor.ar_reason

        # A context change immediately retires the prior path even when fresh.
        context.header.stamp.nanosec = 100000000
        context.context_id = 'replacement'
        context_pub.publish(context)
        pump(.15)
        assert 'path' not in monitor.cache
        assert '隐藏' in monitor.ar_reason
        assert monitor.selection.state == 'LOST'

        # Clock advances, old/replayed images cannot remain displayed.
        c.clock.sec = 13
        for pub in image_pubs.values():
            pub.publish(image)
        pump(.15)
        assert monitor.rendered == {}
        assert monitor.hitboxes == []
        c.clock.sec = 2
        pump(.15)
        assert monitor.context_key is None
    finally:
        if monitor:
            monitor.close()
        else:
            root.destroy()
        source.destroy_node()
        rclpy.try_shutdown()
