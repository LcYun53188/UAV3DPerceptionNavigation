# Source identity and reset counters must travel with the pose, not only alignment.
std_msgs/Header header
string localization_session
uint8[6] reset_counters
nav_msgs/Odometry odometry
