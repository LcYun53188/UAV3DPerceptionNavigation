# Temporary spatial track, not a persistent person identity.
string track_id
geometry_msgs/Point position
float32 detection_confidence
# Depth support metrics, NOT a covariance or total map localization accuracy.
float32 depth_spread_m
uint32 depth_samples
builtin_interfaces/Time last_observed
