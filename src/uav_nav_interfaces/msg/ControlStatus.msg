std_msgs/Header header
string coordinator_instance
ControlSession session
# Monotonic deadlines are process-local diagnostics, never ROS timestamps.
float64 lease_remaining_s
float64 final_hold_remaining_s
string actual_flight_mode
string[] allowed_operations
string reason
bool mock
