# Explicit, externally verified map <- odom transform; no inferred alignment.
std_msgs/Header header
string map_id
uint64 map_epoch
string localization_session
string alignment_id
uint64 generation
# xy, z, vxy, vz, heading, VehicleOdometry reset counters.
uint8[6] reset_counters
geometry_msgs/Transform map_to_odom
bool valid
