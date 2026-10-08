import json
import threading
from types import SimpleNamespace

import pytest
from rclpy.action import GoalResponse,CancelResponse
from uav_nav_interfaces.action import ExecuteMission
from uav_mission.px4_flight import FlightServer
from uav_mission.flight_geometry import Alignment,in_region


def fixture():
    return SimpleNamespace(nonce='owned-test',position=lambda:(0.,0.,0.),
                           alignment=Alignment(),region=in_region)


def request(steps=None,authorization='owned-test'):
    return ExecuteMission.Goal(backend='PX4_KNOWN_REGION',mission_type='FLIGHT_SEQUENCE',timeout_s=120.,
          parameters_json=json.dumps(dict(authorization=authorization,steps=steps or
                               [dict(type='TAKEOFF',height_m=2.),dict(type='LAND')])))


def test_recipe_explicit_authorization_and_complete_landing():
    f=fixture()
    assert len(FlightServer.parse(f,request()))==2
    with pytest.raises(ValueError):FlightServer.parse(f,request(authorization='old-token'))
    with pytest.raises(ValueError):FlightServer.parse(f,request([dict(type='TAKEOFF',height_m=2.)]))


@pytest.mark.parametrize('target', [(20.,0.,2.),(0.,0.,-.1),(0.,0.,6.),(float('nan'),0.,2.)])
def test_airborne_route_rejects_ground_or_envelope_violation(target):
    f=fixture()
    req=request([dict(type='TAKEOFF',height_m=2.),dict(type='NAVIGATE',target_map=f.alignment.to_map(target)
                   if all(x==x for x in target) else target),dict(type='LAND')])
    with pytest.raises(ValueError):FlightServer.parse(f,req)


@pytest.mark.parametrize('parameters', ['[]','{}','{"authorization":"owned-test","steps":[null,{}]}'])
def test_malformed_recipe_rejected(parameters):
    r=request();r.parameters_json=parameters
    with pytest.raises((ValueError,KeyError)):FlightServer.parse(fixture(),r)


def test_native_landing_cancel_guard():
    handle=object()
    f=SimpleNamespace(lock=threading.RLock(),active_goal=handle,land_committed=True,phase='LAND_REQUEST')
    assert FlightServer.cancel(f,handle)==CancelResponse.REJECT
    f.phase='LANDING'
    assert FlightServer.cancel(f,handle)==CancelResponse.REJECT


def test_unknown_state_and_clock_fault_never_admit_goal():
    f=SimpleNamespace(lock=threading.RLock(),clock_fault_latched=True,active_goal=None,reserved=False,owner='NONE')
    assert FlightServer.goal(f,request())==GoalResponse.REJECT
    f.clock_fault_latched=False;f.healthy=lambda ground:False
    assert FlightServer.goal(f,request())==GoalResponse.REJECT


def test_runner_lease_rejects_old_identity_and_replays():
    from uav_nav_interfaces.msg import MissionProgress
    from unique_identifier_msgs.msg import UUID
    handle=SimpleNamespace(goal_id=UUID(uuid=[1]*16))
    f=SimpleNamespace(lock=threading.RLock(),active_goal=handle,runner_required=True,
                      runner_lost=False,result=None,instance='current',runner_sequence=3,runner_last_tick=0.,
                      runner_progress_log=[])
    for identity,sequence,root in [('old',4,[1]*16),('current',3,[1]*16),('current',4,[2]*16)]:
        FlightServer.runner_progress(f,MissionProgress(coordinator_instance=identity,tick_sequence=sequence,
                                                      mission_uuid=UUID(uuid=root)))
        assert f.runner_last_tick==0.
    msg=MissionProgress(coordinator_instance='current',tick_sequence=4,mission_uuid=handle.goal_id)
    FlightServer.runner_progress(f,msg)
    assert f.runner_last_tick>0 and f.runner_sequence==4
    before=f.runner_last_tick
    f.runner_lost=True
    msg.tick_sequence=5
    FlightServer.runner_progress(f,msg)
    assert f.runner_last_tick==before


@pytest.mark.parametrize('armed,landing,expected', [(2,False,'LEASE_BRAKE'),(1,False,'BT_PROGRESS_TIMEOUT'),(2,True,None)])
def test_runner_stall_brakes_or_rejects_before_arming_but_preserves_native_land(armed,landing,expected):
    events=[]
    f=SimpleNamespace(runner_required=True,runner_lost=False,land_committed=landing,canceling=False,
                      runner_last_tick=1.,runner_sequence=1,samples={'vehicle_status':SimpleNamespace(arming_state=armed)},
                      config={'runner_progress_max_age_s':.5,'runner_handshake_timeout_s':5.},
                      fault=events.append,begin_stop=events.append)
    FlightServer.check_runner_progress(f,1.49)
    assert not events
    FlightServer.check_runner_progress(f,1.51)
    assert events==([expected] if expected else [])
    if expected:
        assert f.runner_lost
        FlightServer.check_runner_progress(f,2.)
        assert len(events)==1


def test_runner_requires_current_coordinator():
    f=fixture();f.instance='new'
    r=request();p=json.loads(r.parameters_json)
    p.update(runner_progress_required=True,coordinator_instance='old')
    r.parameters_json=json.dumps(p)
    with pytest.raises(ValueError):FlightServer.parse(f,r)
    p['coordinator_instance']='new';r.parameters_json=json.dumps(p)
    assert len(FlightServer.parse(f,r))==2


def test_runner_first_tick_handshake_is_bounded_on_ground():
    events=[]
    f=SimpleNamespace(runner_required=True,runner_lost=False,land_committed=False,canceling=False,
                      runner_last_tick=1.,runner_sequence=0,config={'runner_handshake_timeout_s':5.},
                      samples={'vehicle_status':SimpleNamespace(arming_state=1)},
                      fault=events.append)
    FlightServer.check_runner_progress(f,5.99)
    assert not events
    FlightServer.check_runner_progress(f,6.01)
    assert events==['BT_PROGRESS_TIMEOUT'] and f.runner_lost
