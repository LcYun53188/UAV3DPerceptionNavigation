import math

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus


def us_to_time(timestamp_us: int) -> Time:
    msg = Time()
    msg.sec = int(timestamp_us // 1000000)
    msg.nanosec = int((timestamp_us % 1000000) * 1000)
    return msg


def vehicle_odometry_to_ros(msg) -> Odometry:
    odom = Odometry()
    odom.header.frame_id = 'map'
    odom.child_frame_id = 'base_link'

    try:
        odom.header.stamp = us_to_time(int(msg.timestamp))
    except Exception:
        pass

    try:
        odom.pose.pose.position.x = float(msg.position[0])
        odom.pose.pose.position.y = float(msg.position[1])
        odom.pose.pose.position.z = float(msg.position[2])
    except Exception:
        pass

    try:
        odom.twist.twist.linear.x = float(msg.velocity[0])
        odom.twist.twist.linear.y = float(msg.velocity[1])
        odom.twist.twist.linear.z = float(msg.velocity[2])
    except Exception:
        pass

    return odom


def vehicle_imu_to_ros(msg) -> Imu:
    imu = Imu()
    imu.header.frame_id = 'base_link'

    try:
        imu.header.stamp = us_to_time(int(msg.timestamp))
    except Exception:
        pass

    try:
        imu.angular_velocity.x = float(msg.angular_velocity[0])
        imu.angular_velocity.y = float(msg.angular_velocity[1])
        imu.angular_velocity.z = float(msg.angular_velocity[2])
    except Exception:
        pass

    try:
        imu.linear_acceleration.x = float(msg.linear_acceleration[0])
        imu.linear_acceleration.y = float(msg.linear_acceleration[1])
        imu.linear_acceleration.z = float(msg.linear_acceleration[2])
    except Exception:
        pass

    return imu


def vehicle_attitude_to_pose(msg) -> PoseWithCovarianceStamped:
    """Convert PX4 VehicleAttitude to a ROS pose orientation measurement.

    PX4 VehicleAttitude stores q as Hamilton [w, x, y, z] for FRD body in
    NED earth. The static NED->ENU and FRD->FLU transforms produce a ROS ENU
    orientation for base_link. Position is intentionally unused by EKF config.
    """
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = 'odom'

    try:
        pose.header.stamp = us_to_time(int(msg.timestamp))
    except Exception:
        pass

    try:
        qw, qx, qy, qz = [float(value) for value in msg.q]
        # q_enu_flu = q_ned_to_enu * q_ned_frd * q_frd_to_flu
        # q_ned_to_enu = [0, sqrt(1/2), sqrt(1/2), 0]
        # q_frd_to_flu = [0, 1, 0, 0]
        s = math.sqrt(0.5)
        ros_qw = s * (qw + qz)
        ros_qx = s * (qx + qy)
        ros_qy = s * (qx - qy)
        ros_qz = s * (qw - qz)
        norm = math.sqrt(
            ros_qw * ros_qw + ros_qx * ros_qx + ros_qy * ros_qy + ros_qz * ros_qz
        )
        if norm > 0.0:
            ros_qw /= norm
            ros_qx /= norm
            ros_qy /= norm
            ros_qz /= norm
        pose.pose.pose.orientation.w = ros_qw
        pose.pose.pose.orientation.x = ros_qx
        pose.pose.pose.orientation.y = ros_qy
        pose.pose.pose.orientation.z = ros_qz
    except Exception:
        pose.pose.pose.orientation.w = 1.0

    covariance = [0.0] * 36
    covariance[0] = 1e6
    covariance[7] = 1e6
    covariance[14] = 1e6
    covariance[21] = 0.05
    covariance[28] = 0.05
    covariance[35] = 1e6
    pose.pose.covariance = covariance
    return pose


def planner_twist_to_ned_velocity(cmd_msg, input_frame: str):
    vx, vy, vz, _yawspeed = planner_twist_to_ned_velocity_and_yawspeed(
        cmd_msg, input_frame
    )
    return vx, vy, vz


def planner_twist_to_ned_velocity_and_yawspeed(cmd_msg, input_frame: str):
    vx = float(cmd_msg.twist.linear.x)
    vy = float(cmd_msg.twist.linear.y)
    vz = float(cmd_msg.twist.linear.z)
    yawspeed = float(cmd_msg.twist.angular.z)

    if input_frame.lower() == 'ned':
        return vx, vy, vz, yawspeed

    # ROS ENU -> PX4 NED
    return vy, vx, -vz, -yawspeed


def fill_trajectory_setpoint(
    setpoint_msg,
    timestamp_us: int,
    vx: float,
    vy: float,
    vz: float,
    yawspeed: float = math.nan,
):
    setpoint_msg.timestamp = int(timestamp_us)
    setpoint_msg.position = [math.nan, math.nan, math.nan]
    setpoint_msg.velocity = [float(vx), float(vy), float(vz)]
    setpoint_msg.acceleration = [math.nan, math.nan, math.nan]
    setpoint_msg.jerk = [math.nan, math.nan, math.nan]
    setpoint_msg.yaw = math.nan
    setpoint_msg.yawspeed = float(yawspeed)


def fill_offboard_control_mode(offboard_msg, timestamp_us: int):
    offboard_msg.timestamp = int(timestamp_us)
    offboard_msg.position = False
    offboard_msg.velocity = True
    offboard_msg.acceleration = False
    offboard_msg.attitude = False
    offboard_msg.body_rate = False
    offboard_msg.thrust_and_torque = False
    offboard_msg.direct_actuator = False


def sensor_gps_to_navsatfix(msg) -> NavSatFix:
    """Convert px4_msgs/SensorGps to sensor_msgs/NavSatFix."""
    fix = NavSatFix()
    fix.header.frame_id = 'gps_link'

    try:
        fix.header.stamp = us_to_time(int(msg.timestamp))
    except Exception:
        pass

    # Position
    fix.latitude = float(msg.latitude_deg)
    fix.longitude = float(msg.longitude_deg)
    fix.altitude = float(msg.altitude_msl_m)

    # Fix status mapping: PX4 fix_type -> NavSatStatus
    # PX4: 0=NONE, 2=2D, 3=3D, 4=RTCM, 5=RTK_FLOAT, 6=RTK_FIXED
    status = NavSatStatus()
    fix_type = int(msg.fix_type)
    if fix_type <= 1:
        status.status = NavSatStatus.STATUS_NO_FIX
    elif fix_type <= 3:
        status.status = NavSatStatus.STATUS_FIX
    elif fix_type == 4:
        status.status = NavSatStatus.STATUS_SBAS
    else:
        # RTK float/fixed
        status.status = NavSatStatus.STATUS_GBAS_FIX
    status.service = NavSatStatus.SERVICE_GPS
    fix.status = status

    # Covariance from eph/epv (horizontal/vertical accuracy in metres)
    try:
        eph = float(msg.eph)
        epv = float(msg.epv)
    except Exception:
        eph = 100.0
        epv = 100.0

    # Diagonal covariance: [east², north², up²]
    fix.position_covariance = [
        eph * eph, 0.0, 0.0,
        0.0, eph * eph, 0.0,
        0.0, 0.0, epv * epv,
    ]
    fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

    return fix
