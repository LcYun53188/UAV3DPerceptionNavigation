# Algorithm simulation contracts

`MapSnapshot`: canonical x-major, z-contiguous distance grid, voxel centers at
`origin + (index + 0.5) * resolution`. `observed=0`, non-finite distance, invalid
snapshot and outside AABB are unavailable. `header.stamp` is cache completion;
`source_stamp` is nvblox's most recent integrated depth, not per-voxel age. Static
maps renew the cache lease only in explicit same-scene offline simulation mode.

`epoch` changes on map operations. Versions start at zero after epoch change;
only valid query results increment them. Retain one immutable snapshot for each
planning attempt. Consumers must reject an older epoch or decreasing version.

`TimedTrajectory`: 3-D uniform cubic B-spline. For N control points and knot
interval dt, knot vector is `[-3,-2,...,N] * dt`, time domain is
`[0,(N-3)*dt]`. Control array order is time order, coordinates are metres, absolute
start uses ROS simulation time, derivative units are m/s, m/s² and m/s³. First and
last three controls coincide, defining zero velocity/acceleration at each end.
The simulation executor accepts only a stopped takeover and validates the full
curve; it does not interpolate `nav_msgs/Path`. `map_version` identifies the
planning snapshot, while the latest available snapshot independently rechecks it.

Simulation uses a fixed identity `map -> odom`; this contract does not authorize
using map-frame curves as odom-frame curves when real localization can correct TF.

`PlannerStatus`: per-attempt state plus the exact `goal_stamp` from the local
`PoseStamped.header.stamp`. `TimedTrajectory.goal_stamp` carries the same token.
In managed mode the executor accepts only the outstanding planning token; cancel,
replacement, completion, timeout, or terminal failure retires it. Tokens are
strictly increasing within an executor process, including simulated clock resets.
Rebuild both interface consumers after changing these message definitions.

`/uav/navigation/state` is the overall task result; planner status and
`LOCAL_GOAL_REACHED` refer only to one segment. A new map does not automatically
resume a terminal BLOCKED, STOPPED, or CANCELLED task.
