# Uniform cubic B-spline, knot interval in seconds, XYZ controls in header.frame_id.
std_msgs/Header header
string map_id
uint64 epoch
uint64 map_version
uint64 trajectory_id
builtin_interfaces/Time start_time
float64 knot_interval
geometry_msgs/Point[] control_points
float64 max_velocity
float64 max_acceleration
float64 max_jerk
# Endpoint has zero velocity and acceleration; no arbitrary interpolation.
