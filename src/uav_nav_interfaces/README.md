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
start uses ROS time (simulation time in Gazebo), derivative units are m/s, m/s²
and m/s³. Last three controls coincide, defining a stopped endpoint. A stopped
start (`parent_trajectory_id=0`) also has three coincident initial controls.
A moving start preserves the exact position, velocity and acceleration of its
identified parent at `start_time`. The executor queues the successor while the
parent continues, checks C2 continuity and the full curve, and switches only at
that timestamp. A failed or late successor leaves the parent's checked stop
trajectory active. It does not interpolate `nav_msgs/Path`. `map_version` identifies the
planning snapshot, while the latest available snapshot independently rechecks it.

Simulation uses a fixed identity `map -> odom`; this contract does not authorize
using map-frame curves as odom-frame curves when real localization can correct TF.

`PlannerStatus`: per-attempt state plus the exact `goal_stamp` from the local
`PoseStamped.header.stamp`. `TimedTrajectory.goal_stamp` carries the same token.
In managed mode the executor accepts only the outstanding planning token; cancel,
replacement, completion, timeout, or terminal failure retires it. Tokens are
strictly increasing within an executor process, including simulated clock resets.
Rebuild both interface consumers after changing these message definitions.

`TrajectoryRequest` on `/uav/replan_request` carries a separate request token in
`header.stamp`, the map session, parent trajectory ID, absolute handover time,
predicted initial position/velocity/acceleration, and the next goal. Only managed
simulation planning accepts it. Its token does not replace the active token until
handover. The executor rejects a wrong parent/session/token, boundary discontinuity,
expired start or invalid curve; it rechecks queued curves on map updates. Cancel,
health faults and session changes discard both the active and queued trajectory.
`QUEUED`, `HANDOVER` and `REPLAN_REQUESTED` executor events expose this lifecycle.

`/uav/navigation/state` is the overall task result; planner status and
`LOCAL_GOAL_REACHED` refer only to one segment. A new map does not automatically
resume a terminal BLOCKED, STOPPED, or CANCELLED task.

`AircraftState` and `StateDimension` are the first S1 read-only aircraft facts.
Every dimension has its own validity, source/receive ages and reason. UNKNOWN or
invalid values cannot authorize motion; stale ARMED/ON_GROUND values are not kept
as current facts. The instance UUID and sequence identify observer restart/events,
not a mission or control session. See [observer contract](../uav_mission/README.md)
for supported values and remaining providers. These messages do not change the
algorithm trajectory/frame contracts above.
