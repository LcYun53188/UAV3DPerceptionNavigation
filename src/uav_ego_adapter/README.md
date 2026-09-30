# EGO nvblox adapter

Single-threaded ROS 2 node holding an immutable nvblox snapshot throughout each
attempt. Uses EGO's original A* as a detour seed and its rebound B-spline optimizer;
there is no substitute APF or separate occupancy integrator. Stops are enforced by
three repeated controls at each end. Retiming uses full derivative control-hull
bounds, then the entire curve is checked against the map. The separate Python
Gazebo executor independently checks the same timed representation.

The A* seed reserves half a voxel of extra clearance for swept-curve sampling.
If rebound optimization fails or its smoothed curve fails validation, the adapter
shortcuts the original seed using conservative segment checks and builds a cubic
B-spline with three repeated controls at each waypoint. This fallback stops at
waypoints and can be slower, but does not cut corners. It passes the same dynamic,
collision, duration, and map-age checks before publication. Planner status
`SAFE_SEED_FALLBACK` identifies this case; state changes are also logged.

Current scope: stopped start/goal tasks in a static Gazebo world, identity
map/odom alignment, conservative unknown-space policy. A failure never emits an
unchecked alternative path. Moving handover and real vehicle braking remain
future flight-control work. See `docs/EGO_NVBLOX_GAZEBO.md` for commands.

The standard launch enables `managed_goals` and remaps the planner input to
`/uav/local_goal`. The executor-owned goal manager retains `/uav/goal`, selects
safe observation positions in live mapping mode, and owns finite retries and
terminal task states. `PlannerStatus` and `TimedTrajectory.goal_stamp` echo the
local goal stamp so cancelled or replaced work cannot restart execution.
Standalone unmanaged planner behavior remains available with the parameter off.
