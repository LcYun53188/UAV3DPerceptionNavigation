# EGO nvblox adapter

Single-threaded ROS 2 node holding an immutable nvblox snapshot throughout each
attempt. Uses EGO's original A* as a detour seed and its rebound B-spline optimizer;
there is no substitute APF or separate occupancy integrator. Stops are enforced by
three repeated controls at each end. Retiming uses full derivative control-hull
bounds, then the entire curve is checked against the map. The separate Python
Gazebo executor independently checks the same timed representation.

Current scope: stopped start/goal tasks in a static Gazebo world, identity
map/odom alignment, conservative unknown-space policy. A failure never emits an
unchecked alternative path. Moving handover and real vehicle braking remain
future flight-control work. See `docs/EGO_NVBLOX_GAZEBO.md` for commands.
