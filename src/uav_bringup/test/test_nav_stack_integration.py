import unittest

from geometry_msgs.msg import TwistStamped
import launch
import launch_ros
import launch_testing
from nav_msgs.msg import OccupancyGrid
import pytest
import rclpy
from sensor_msgs.msg import PointCloud2

try:
    from px4_msgs.msg import TrajectorySetpoint

    PX4_AVAILABLE = True
except ImportError:
    PX4_AVAILABLE = False


@pytest.mark.rostest
def generate_test_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='nav_mapping',
            executable='local_map_builder',
            name='local_map_builder',
            parameters=[{'transform_timeout_sec': 5.0}]
        ),
        launch_ros.actions.Node(
            package='nav_planning',
            executable='se2_dwa_local_planner',
            name='se2_dwa_local_planner',
        ),
        launch_ros.actions.Node(
            package='px4_comm_bridge',
            executable='px4_bridge_node',
            name='px4_comm_bridge',
        ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '0',
                '0',
                '0',
                '0',
                '0',
                '0',
                'map',
                'oakd_camera_optical_frame',
            ],
        ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '0',
                '0',
                '0',
                '0',
                '0',
                '0',
                'map',
                'base_link',
            ],
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestNavStackIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_node')

    def tearDown(self):
        self.node.destroy_node()

    def test_pipeline_flow(self):
        pc_pub = self.node.create_publisher(PointCloud2, '/oakd/points_filtered', 10)

        map_received = []
        self.node.create_subscription(
            OccupancyGrid,
            '/local_map/occupancy',
            lambda msg: map_received.append(msg),
            10,
        )

        cmd_vel_received = []
        self.node.create_subscription(
            TwistStamped,
            '/nav/cmd_vel',
            lambda msg: cmd_vel_received.append(msg),
            10,
        )

        px4_setpoint_received = []
        if PX4_AVAILABLE:
            self.node.create_subscription(
                TrajectorySetpoint,
                '/fmu/in/trajectory_setpoint',
                lambda msg: px4_setpoint_received.append(msg),
                10,
            )

        msg = PointCloud2()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'oakd_camera_optical_frame'
        msg.width = 1
        msg.height = 1
        msg.point_step = 12
        msg.row_step = 12
        msg.is_dense = True
        import struct
        msg.data = struct.pack('fff', 1.0, 0.0, 0.0)
        from sensor_msgs.msg import PointField
        msg.fields = [
            PointField(
                name='x',
                offset=0,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name='y',
                offset=4,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name='z',
                offset=8,
                datatype=PointField.FLOAT32,
                count=1,
            ),
        ]

        end_time = self.node.get_clock().now() + rclpy.duration.Duration(
            seconds=10.0
        )
        while rclpy.ok() and self.node.get_clock().now() < end_time:
            pc_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if len(map_received) > 0 and len(cmd_vel_received) > 0:
                if not PX4_AVAILABLE or len(px4_setpoint_received) > 0:
                    break

        self.assertGreater(len(map_received), 0)
        self.assertGreater(len(cmd_vel_received), 0)
        if PX4_AVAILABLE:
            self.assertGreater(len(px4_setpoint_received), 0)
