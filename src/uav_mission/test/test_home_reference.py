from types import SimpleNamespace
import math
import pytest
from px4_msgs.msg import HomePosition,VehicleLocalPosition,VehicleGlobalPosition,FailsafeFlags
from uav_mission.home_reference import local_home,native_rtl_home
from uav_mission.px4_flight import FlightServer


def home():
    return HomePosition(timestamp=5000000,x=2.,y=-3.,z=-.4,
        valid_lpos=True,valid_hpos=True,valid_alt=True,manual_home=True,
        lat=47.,lon=8.,alt=488.,update_count=2)


def test_static_px4_home_preserves_native_stamp_and_ned_frame_without_task_origin():
    h=home();r=local_home(h,100.,writer_count=1)
    assert r.position_enu==pytest.approx((-3.,2.,.4))
    assert r.sample_us==h.timestamp and r.manual
    assert r.evidence()['source']=='PX4_HOME_POSITION'


@pytest.mark.parametrize('fault',['missing','local_invalid','nan','future','reset','writers'])
def test_unusable_home_never_falls_back_to_mission_start(fault):
    h=home();epoch=0;writers=1
    if fault=='missing':h=None
    elif fault=='local_invalid':h.valid_lpos=False
    elif fault=='nan':h.x=math.nan
    elif fault=='future':h.timestamp=101000000
    elif fault=='reset':epoch=6000000
    elif fault=='writers':writers=2
    with pytest.raises(ValueError):local_home(h,100.,writer_count=writers,frame_epoch_us=epoch)


@pytest.mark.parametrize('fault',['none','no_global_home','no_global_position','native_invalid','stale','nonfinite'])
def test_native_rtl_requires_real_global_reference_and_native_home(fault):
    h=home();local=VehicleLocalPosition(xy_global=True,z_global=True)
    g=VehicleGlobalPosition(timestamp=100000000,lat=47.,lon=8.,alt=488.)
    flags=FailsafeFlags()
    if fault=='no_global_home':h.valid_hpos=False
    elif fault=='no_global_position':local.xy_global=False
    elif fault=='native_invalid':flags.global_position_invalid=True
    elif fault=='stale':g.timestamp=99000000
    elif fault=='nonfinite':g.lat=math.nan
    if fault=='none':assert native_rtl_home(h,local,g,flags,100.,writer_count=1)
    else:
        with pytest.raises(ValueError):native_rtl_home(h,local,g,flags,100.,writer_count=1)


def server():
    node=SimpleNamespace(config={'return_home_source':'PX4_HOME_POSITION'},navigation_backend='EGO',
        home=(99.,98.,0.),reference=(8.,9.,3.),return_home_reference=None,home_frame_epoch_us=0,
        samples={'home_position':home()},region=lambda p:True,diagnostics=[],
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=100000000000)),
        get_publishers_info_by_topic=lambda t:[object()])
    return node


def test_navigation_returns_to_px4_home_not_mission_start_and_freezes_it():
    node=server();target=FlightServer.return_target(node)
    assert target==(-3.,2.,3.) and node.home==(99.,98.,0.)
    node.samples['home_position'].update_count+=1  # Same target can be republished.
    assert FlightServer.return_target(node)==target
    node.samples['home_position'].x+=1.  # A QGC home move needs a new request/plan.
    with pytest.raises(ValueError,match='HOME_CHANGED'):FlightServer.return_target(node)


@pytest.mark.parametrize('fault',['direct','missing','outside','epoch'])
def test_avoidance_return_cannot_downgrade_or_reuse_an_invalid_point(fault):
    node=server()
    if fault=='direct':node.navigation_backend='DIRECT'
    elif fault=='missing':node.samples.clear()
    elif fault=='outside':node.region=lambda _:False
    elif fault=='epoch':node.home_frame_epoch_us=6000000
    with pytest.raises(ValueError):FlightServer.return_target(node)
    assert node.return_home_reference is None
