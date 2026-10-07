# S1 AircraftState observer

First S1 increment: read-only PX4 1.16 facts. This package does not implement
MissionServer, FlightSession, a control lease, navigation authorization or flight
Actions. It publishes no PX4 input, TF or localization messages.

Build against the pinned isolated PX4 messages (do not source the older workspace
`px4_msgs` over this environment):

```bash
./scripts/with_px4_sim.sh colcon --log-base .cache/simulation/mission-colcon-log build \
  --base-paths src/uav_nav_interfaces src/uav_mission \
  --build-base .deps/mission-build --install-base .deps/mission-install \
  --symlink-install --parallel-workers 1
./scripts/with_px4_sim.sh bash -e -c \
  'source .deps/mission-install/local_setup.bash; python scripts/run_px4_sitl_smoke.py --duration 30 --aircraft-state'
```

The owned smoke session starts the observer with `use_sim_time:=true`, namespace
`/px4_7`, ROS domain 78, then stops PX4 and the clock bridge in turn. It requires
fresh DISARMED/ON_GROUND facts, stale invalid facts after source loss, and continued
invalid `ROS_TIME_STALLED` publications after clock loss. It never arms the vehicle.
Outside that supervisor run `ros2 run uav_mission aircraft_state --ros-args -p
use_sim_time:=true -p px4_namespace:=/px4_7` in the same isolated overlays/domain.

`aircraft_state` carries an instance UUID, increasing sequence and one
`StateDimension` per dimension. Consumers must check `valid` as well as `value`.
Values/reasons describe this observer's evidence, not permission to control:

| Dimension | Values / source |
| --- | --- |
| link | CONNECTED, STALE, LOST, UNKNOWN; VehicleStatus freshness only, not Agent/client health |
| arming | DISARMED, ARMED, UNKNOWN; actual VehicleStatus |
| ground | ON_GROUND, IN_AIR, TRANSITION, UNKNOWN; land detector and its motion/conflict flags |
| mode | OFFBOARD, PILOT, AUTO_LAND, AUTO_OTHER, UNKNOWN; raw nav_state retained while status fresh |
| localization | LOCAL_POSITION, ALTITUDE, INVALID, UNKNOWN; PX4 local validity, finite coordinates, error estimates and reset counters |
| autopilot_exception | NOMINAL, FAILSAFE, UNKNOWN; failsafe/failure detector flags |
| ownership / navigation / battery | UNKNOWN / NOT_READY / UNKNOWN, all invalid until their providers exist |

Source age uses ROS time; receive age uses monotonic time. Both must be at most
`max_age_s` (default 0.5 s). The source clock may lead `/clock` by at most 0.05 s,
a declared transport ordering tolerance, not an arbitrary time offset. Missing
ages are -1. Duplicate timestamps never renew receive freshness. Out-of-order
source timestamps, local position/heading reset counters, backwards time and a
clock frozen for more than `clock_stall_s` (default 0.5 s) invalidate facts.
Clock faults latch for the instance; source/reset faults latch for that source.
Recovery requires a new observer instance and future task authorization. A steady
clock timer keeps publishing at 20 Hz even when `/clock` stops.

Default PX4 status is approximately 2 Hz and land detection 1 Hz: the strict
0.5 s threshold intentionally exposes gaps. The smoke waits for a fresh detector
window; it does not change this threshold to obtain a pass. A link marked LOST
means status age exceeds three freshness windows, not a transport diagnosis.
LOCAL_POSITION requires horizontal and vertical error estimates <=1 m and no
dead reckoning; ALTITUDE requires vertical error <=1 m. These are provisional
observation limits, not frozen S3/S4 flight thresholds. Attitude-only observation,
full localization quality, independent Agent health, arbiter, battery, map/TF and
navigation health checks are still pending. No readiness is inferred from this
partial aggregate.

Pure fault-boundary regression:

```bash
PYTHONPATH=src/uav_mission PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 SKIP_WS_SETUP=true \
  ./scripts/with_venv.sh python -m pytest -q src/uav_mission/test
```

The next S1 increment adds [task/FlightSession protocols and ROS mock](PROTOCOL.md).
Use its separate mock smoke for Action/service validation; it does not change the
read-only AircraftState observer or authorize a real vehicle.
