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
