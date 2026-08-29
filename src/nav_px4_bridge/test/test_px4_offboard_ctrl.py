import unittest
from nav_px4_bridge.px4_offboard_ctrl import Px4OffboardCtrl
from geometry_msgs.msg import TwistStamped

class TestPx4OffboardCtrl(unittest.TestCase):
    def setUp(self):
        # We need to mock rclpy.init if we want to instantiate the node
        # but for unit testing static/internal methods we might not need it
        pass

    def test_convert_velocity_enu_to_ned(self):
        # Mocking node for parameter access
        class MockNode:
            def get_parameter(self, name):
                class Param:
                    value = 'enu'
                return Param()
        
        node = MockNode()
        # ENU (vx, vy, vz) -> NED (vy, vx, -vz)
        cmd = TwistStamped()
        cmd.twist.linear.x = 1.0  # East
        cmd.twist.linear.y = 2.0  # North
        cmd.twist.linear.z = 3.0  # Up
        
        vx, vy, vz = Px4OffboardCtrl._convert_velocity(node, cmd)
        self.assertEqual(vx, 2.0)  # North is X in NED
        self.assertEqual(vy, 1.0)  # East is Y in NED
        self.assertEqual(vz, -3.0) # Up is -Z in NED

    def test_convert_velocity_ned(self):
        class MockNode:
            def get_parameter(self, name):
                class Param:
                    value = 'ned'
                return Param()
        
        node = MockNode()
        cmd = TwistStamped()
        cmd.twist.linear.x = 1.0
        cmd.twist.linear.y = 2.0
        cmd.twist.linear.z = 3.0
        
        vx, vy, vz = Px4OffboardCtrl._convert_velocity(node, cmd)
        self.assertEqual(vx, 1.0)
        self.assertEqual(vy, 2.0)
        self.assertEqual(vz, 3.0)

if __name__ == '__main__':
    unittest.main()
