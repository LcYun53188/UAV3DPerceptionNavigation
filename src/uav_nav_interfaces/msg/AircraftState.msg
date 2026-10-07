# Read-only facts. No field grants permission to publish controls.
std_msgs/Header header
string aggregator_instance
uint64 sequence
StateDimension link
StateDimension arming
StateDimension ground
StateDimension mode
StateDimension ownership
StateDimension localization
StateDimension navigation
StateDimension autopilot_exception
StateDimension battery
# -1 when VehicleStatus is unavailable/stale.
int16 raw_nav_state
string[] reasons
