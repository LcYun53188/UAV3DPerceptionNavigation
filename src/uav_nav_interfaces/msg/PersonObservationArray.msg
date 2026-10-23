# header is publication time in map; each observation retains capture time.
std_msgs/Header header
string map_id
uint64 map_epoch
string localization_session
string alignment_id
uint64 alignment_generation
string context_id
bool valid
string reason
# First version: visible_torso_surface. Do not interpret as body centre or feet.
string point_semantics
PersonObservation[] observations
