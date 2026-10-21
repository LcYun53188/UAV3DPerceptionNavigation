"""Safety end request uses native LAND without an unfinished map or rearming."""
import time
from types import SimpleNamespace as NS

import pytest
from uav_nav_interfaces.srv import RequestReturn
from uav_mission.px4_flight import FlightServer
from test_return_checkpoint import gateway, request


def aircraft(phase):
    f,p,events,starts=gateway(kind=phase,backend='EGO')
    f.samples['vehicle_status']=NS(system_id=8,component_id=1,nav_state=14,arming_state=2)
    f.samples['vehicle_attitude']=NS(q=[1.,0.,0.,0.])
    f.samples['battery_status']=NS(connected=True,warning=0,remaining=.8)
    f.samples['vehicle_local_position']=NS(z=-2.,vz=0.,epv=.1,z_valid=True,v_z_valid=True)
    f.samples['vehicle_land_detected']=NS(landed=False)
    f.fresh=lambda *args:True
    f.publish_status=lambda:events.append(('status',f.phase))
    f.fault=lambda reason:events.append(('fault',reason))
    f.last_command=time.monotonic()
    return f,p,events,starts


@pytest.mark.parametrize('phase',['PRESTREAM','OFFBOARD_REQUEST','ARM_REQUEST','TAKEOFF',
    'OBSERVE','PAUSING','HOLD_HANDOFF'])
def test_sd_before_return_readiness_commits_native_land_without_map_or_home(phase):
    f,p,events,starts=aircraft(phase)
    root,home,deadline=f.active_goal,f.home,f.deadline
    f.pilot_control='PRESTREAM'
    f.pilot_pending='resume'
    f.survey=object()
    f.planning_ready=lambda:pytest.fail('unfinished map must not be consulted')
    r=request(f,finish=True)
    response=f.service('finish',r,RequestReturn.Response())
    assert response.accepted and response.reason=='END_LOCAL_LAND_REQUESTED'
    assert f.phase=='LAND_REQUEST' and f.land_committed and f.end_local_land
    assert f.owner=='AUTOPILOT' and f.generation==4 and f.plan_revision==1
    assert f.active_goal is root and f.home==home and f.deadline==deadline
    assert not f.return_override and f.return_checkpoint is None
    assert f.survey is None and f.segment is None and not any(f.child.uuid)
    assert f.pilot_control is None and f.pilot_pending is None and f.step_grant is None
    assert [args for kind,args in events if kind=='command']==[(21,)]
    assert not starts
    assert f.service('finish',r,RequestReturn.Response()).accepted
    assert [args for kind,args in events if kind=='command']==[(21,)]


@pytest.mark.parametrize('fault',['height','status','writer','battery','clock','identity'])
def test_invalid_native_safety_context_rejects_without_output(fault):
    f,p,events,starts=aircraft('OBSERVE')
    if fault=='height':f.samples['vehicle_local_position'].z_valid=False
    if fault=='status':f.fresh=lambda name,*_:name!='vehicle_status'
    if fault=='writer':f.get_publishers_info_by_topic=lambda _:[object(),object()]
    if fault=='battery':f.samples['battery_status'].warning=1
    if fault=='clock':f.clock_fault_latched=True
    r=request(f,finish=True)
    if fault=='identity':r.coordinator_instance='old'
    response=f.service('finish',r,RequestReturn.Response())
    assert not response.accepted
    assert f.phase=='OBSERVE' and f.generation==3 and not f.land_committed
    assert not events and not starts


def test_native_landing_waits_for_actual_mode_landed_and_disarm_without_vio_streams():
    f,p,events,_=aircraft('OBSERVE')
    assert f.service('finish',request(f,finish=True),RequestReturn.Response()).accepted
    now=time.monotonic()
    f.phase_started=now
    f.healthy=lambda **_:pytest.fail('VIO must not gate committed native LAND')
    f.stream=lambda:pytest.fail('native LAND must not receive automatic trajectories')
    FlightServer.end_local_land_tick(f,now+.1)
    assert f.phase=='LAND_REQUEST' and f.result is None
    f.samples['vehicle_status'].nav_state=18
    FlightServer.end_local_land_tick(f,now+.2)
    assert f.phase=='LANDING' and f.result is None
    f.samples['vehicle_land_detected'].landed=True
    FlightServer.end_local_land_tick(f,now+.3)
    assert f.result is None
    f.samples['vehicle_status'].arming_state=1
    FlightServer.end_local_land_tick(f,now+.4)
    assert f.result is None
    FlightServer.end_local_land_tick(f,now+2.5)
    assert f.result==('SUCCEEDED','MISSION_ENDED_LOCAL_LANDED_AND_DISARMED',True)
    assert f.owner=='NONE' and f.phase=='COMPLETE'


def test_safety_land_has_bounded_observation_and_never_claims_cleanup_on_timeout():
    f,_,events,_=aircraft('TAKEOFF')
    assert f.service('finish',request(f,finish=True),RequestReturn.Response()).accepted
    FlightServer.end_local_land_tick(f,f.end_land_until)
    assert ('fault','END_LOCAL_LAND_TIMEOUT') in events
    assert f.result is None


def test_main_tick_does_not_resume_old_stream_or_require_vio_after_native_commit():
    f,_,events,_=aircraft('OBSERVE')
    assert f.service('finish',request(f,finish=True),RequestReturn.Response()).accepted
    f.update_clock_watchdog=lambda *args:False
    f.publish_control=lambda *args:None
    f.last_graph_check=time.monotonic()
    f.healthy=lambda **_:pytest.fail('native landing must not depend on VIO readiness')
    f.stream=lambda:pytest.fail('old survey must never stream after native LAND commit')
    f.phase_started=time.monotonic()
    f.samples['vehicle_status'].nav_state=18
    FlightServer.tick(f)
    assert f.phase=='LANDING' and ('status','LANDING') in events
