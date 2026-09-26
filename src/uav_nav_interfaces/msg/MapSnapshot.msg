# Immutable canonical x/y/z grid: flat index = (x * shape[1] + y) * shape[2] + z.
std_msgs/Header header
builtin_interfaces/Time source_stamp
string map_id
uint64 epoch
uint64 version
bool valid
bool static_map
geometry_msgs/Point origin
float64 resolution
uint32[3] shape
float32[] distance
# ESDF observed flag only; this is not per-voxel observation age.
uint8[] observed
