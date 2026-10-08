# S1 task identity and FlightSession mock

This increment defines the minimum mission/navigation Actions, pause/resume
services, TaskStatus and ControlSession/ControlStatus. The pure `protocol.py`
model and ROS `mission_protocol_mock` fixture validate protocol ordering only.
No PX4, planner, TF, localization or actuator messages are sent. Synthetic results
are marked `mock=true` and `MOCK_ONLY`; they are not proof of arrival, stopped
flight, successful landing, localization readiness or BT operation.

## Build and run

Use the isolated build commands in [README](README.md), then:

```bash
./scripts/with_px4_sim.sh bash -e -c \
  'source .deps/mission-install/local_setup.bash; python scripts/run_mission_protocol_smoke.py'
```

The supervisor uses ROS domain 79, exclusive local lock and owned process group.
It creates a server, uses ordinary ROS Action/service clients, archives results
under `.cache/simulation/mission-protocol/<run-id>`, then stops its own server.
The earlier SITL/algorithm domains remain 78/68. Run the standalone fixture via
`ros2 run uav_mission mission_protocol_mock` in the same isolated overlay and a
separate domain when manually testing; stop it before running the supervisor.

| Endpoint | Contract |
| --- | --- |
| `/uav/mock/execute_mission` | ExecuteMission; backend `MOCK`, mission_type `NAVIGATE`, parameters_json contains work_s (>0), optional boolean handoff_failure/progress_stall |
| `/uav/mock/navigate` | NavigateToPose3D; standalone mock root, map pose with normalized quaternion, positive tolerances/stability/timeout; empty external control_session |
| `/uav/mock/pause`, `/uav/mock/resume` | Mission UUID, coordinator instance and nonzero request UUID; acceptance is not stop confirmation |
| `/uav/mock/task_status` | Root/child UUID, instance, sequence, phase/result, map session and control generation |
| `/uav/mock/control_status` | One owner, lease/hold remaining time, allowed mock operations; actual flight mode UNKNOWN |

ExecuteMission uses the ROS Action goal UUID as its root task identity. The
navigation Action normally carries a parent's ControlSession and map session;
this standalone fixture deliberately accepts only an empty external session,
allocates its own root and does not simulate nested ROS Actions. Nested Action
integration and waypoint checkpoints belong to S2. Its pose feedback is marked
`mock_unknown`, distance -1 and segment count 0; a synthetic timer ending does
not establish geometric goal tolerance or stopped velocity.

## Owner/event ordering

One owner serializes all transitions (the ROS fixture uses one lock). A generation
changes atomically with ownership. Child completion must match root UUID,
coordinator instance, control session ID, generation and child UUID. Retiring a
child makes all old child events ineligible; real executor token/trajectory
revocation remains an integration requirement, not an effect of this mock.

Pause is RUNNING -> PAUSING -> PAUSED. It retires the child, waits for a fresh
simulated stopped observation, transfers to HOLD_CONTROLLER with a new generation,
then requires a matching fresh gateway hold ACK. The root remains active. Resume
requires matching map session, readiness and unexpired pause/total/hold lease;
it creates a new child UUID and generation through RESUMING -> RUNNING. The root
definition hash and original request remain unchanged; mock elapsed motion time
advances only in RUNNING. There is no persisted or restartable checkpoint.

The response for a repeated request UUID is the original decision, including its
original phase. Reusing it for another verb is REQUEST_ID_CONFLICT. Identity
validation precedes cache lookup, so an old root/instance cannot operate on a new
one. Completion and cancel use owner event order: completion committed first
rejects cancel; cancel first retires the child and prevents success. Cancel during
PAUSING takes precedence. Native landing commitment rejects cancel/pause in the
pure guard model; real mode command/ACK integration is not implemented here.

Cancellation: retire child -> bounded simulated braking -> fresh stopped event
-> atomic HOLD_CONTROLLER/generation transfer -> fresh matching gateway ACK ->
CANCELED. Failure to confirm the handoff yields ABORTED/CANCEL_TIMEOUT and
cleanup_confirmed=false. Root success also waits for this cleanup; it does not
return on a local segment event. A completed result is immutable if later hold
expiry/takeover changes the surviving session status.

The fixture synthesizes stopped/ACK evidence after 0.1/0.2 s; handoff_failure
suppresses it. Production adapters must replace those events with measured facts,
not timers. No physical braking, gateway acknowledgment, stop margin or continuity
of control output is established here.

## Deadlines and faults

Candidates: progress lease 0.5 s, cleanup 1 s, final hold 30 s, maximum pause 60 s.
These are mock limits, not frozen flight dynamics thresholds. All deadlines use
monotonic time; ROS stamps only timestamp diagnostics. The total task budget
includes pause and cleanup: cleanup begins at total_deadline minus cleanup budget,
and must finish by total_deadline. Pause cannot extend the original deadline.

A HoldController may renew after the root result, with fresh reference/healthy
checks, up to its fixed hold deadline. Renewal never moves that deadline. A paused
root renews via hold checks, not task progress. Progress or hold lease loss enters
bounded cleanup/fault; it never silently resumes the old task. Actual takeover or
failsafe invalidates automatic generations and blocks new admission until a future
explicit recovery protocol exists. Bad monotonic input faults the model and raises.

The pure model emits NONE/fault on unconfirmed cleanup/expired hold; the fixture
performs no landing or flight failsafe. S3/S4 must integrate verified gateway and
PX4 failure dispositions, fresh AircraftState, localization/map sessions and W0.
ResetFault/AcquireControl are not implemented; no real authorization is claimed.
S1 mock protocols are covered here; full S1 readiness still requires independent
state providers and the reset/re-authorize contract noted in the AircraftState report.

The real W0 flight backend is now available in [the local SITL guide](../../simulation/px4/README.md#w0-真实飞行任务). The mock contracts above remain synthetic; they do not establish the real backend's physical performance.

真实 W0 后端支持可选 `runner_progress_required=true` 与当前 `coordinator_instance`，
要求 `/uav/px4/mission_progress` 的根 UUID/实例/递增序列匹配。首次握手只在地面等待
最多 5 s，之后进展年龄不得超过 0.5 s。超时终态为 ABORTED/BT_PROGRESS_TIMEOUT；
cleanup_confirmed=true 仅在实际停稳与保持交接已确认时成立。最终保持仍有独立固定
期限，原生降落不因租约超时打断。旧直接客户端不声明此项，保留已有任务预算。
