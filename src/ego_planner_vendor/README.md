# EGO library vendor

Upstream: https://github.com/ZJU-FAST-Lab/ego-planner-swarm

Pinned ROS 2 revision: `23a8d5a191711dd65633df689bd00f55d4dea8f9`.
License: upstream GPL-3.0; individual retained files carry their own notices.

Run `../../scripts/prepare_ego_vendor.sh` from the workspace (or use its workspace
relative path `scripts/prepare_ego_vendor.sh`). Source is kept in ignored
`.deps/ego_planner_src`; this package contains only the project build wrapper.
All upstream source changes are in `patches/vendor/ego_planner.patch`.

Built: original A*, uniform B-spline, rebound optimizer, polynomial and object
prediction library units needed for linking. The project never initializes the
object predictor or swarm members; it supplies an empty swarm. Not built: native
sensor integration, upstream FSM, ROS control outputs, simulator, TCP bridge or
upstream launch. `GridMap` is a query callback adapter; no second map is created.

Map callers in the built units: `BsplineOptimizer::{initControlPoints,
check_collision_and_rebound,rebound_optimize,refine_optimize,distinctiveTrajs}`
use `getInflateOccupancy/getResolution`; `AStar::checkOccupancy` uses the same
callback. The project patch also checks every A* edge through `segmentCollision`.
The project planner and executor independently validate the entire final curve.
