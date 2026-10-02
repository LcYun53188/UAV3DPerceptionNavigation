# Moving handover in the simulation's identity map/odom frames.
# header.stamp is a unique request token, echoed in TimedTrajectory.goal_stamp.
std_msgs/Header header
string map_id
uint64 epoch
uint64 parent_trajectory_id
builtin_interfaces/Time start_time
geometry_msgs/Point start_position
geometry_msgs/Vector3 start_velocity
geometry_msgs/Vector3 start_acceleration
geometry_msgs/Point goal
