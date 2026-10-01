# EGO nvblox adapter

Single-threaded ROS 2 node holding an immutable nvblox snapshot throughout each
attempt. Uses EGO's original A* as a detour seed and its rebound B-spline optimizer;
there is no substitute APF or separate occupancy integrator. Stops are enforced by
three repeated controls at each end. Retiming uses full derivative control-hull
bounds, then the entire curve is checked against the map. The separate Python
Gazebo executor independently checks the same timed representation.

The A* seed reserves half a voxel of extra clearance for swept-curve sampling.
Exploration viewpoints use the same seed margin. A* connects exact endpoints
to nearby free lattice cells using checked segments, instead of pushing blocked
rounded endpoints along the start/goal ray. The adapter retains those connector
segments when passing the seed to the optimizer and fallback builder.
An obstructed incoming edge does not mark its destination as discovered; another
neighbor can still connect to it. Coarse-search failure triggers one retry at
map resolution, with the same clearance and collision rules.
If rebound optimization fails or its smoothed curve fails validation, the adapter
shortcuts the original seed using conservative segment checks and builds a cubic
B-spline with three repeated controls at each retained route corner. Single
interior controls spaced at most 0.5 m apart keep straight travel continuous;
they do not introduce intermediate stops. This fallback still stops at corners
and does not cut them. It passes the same dynamic,
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
