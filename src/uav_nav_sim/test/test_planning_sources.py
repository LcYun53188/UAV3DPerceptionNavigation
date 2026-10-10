from copy import deepcopy
import pytest
from geometry_msgs.msg import Transform
from test_planning_context import inputs, stamp
from uav_nav_sim.planning_sources import SourceBinding


def setup():
    m,o,a=inputs()
    m.source_stamp=stamp(10.)
    binding=SourceBinding(o.localization_session,'owned-explicit',a.map_to_odom)
    assert binding.update('map',m,1.) and binding.update('odom',o,1.)
    assert binding.ready(10.,1.)
    return binding,m,o


def test_alignment_heartbeat_preserves_sources():
    b,m,o=setup()
    a=b.alignment(10.1,1.1,stamp(10.1))
    assert a.valid and a.generation==1
    assert (a.map_id,a.map_epoch)==(m.map_id,m.epoch)
    assert a.localization_session==o.localization_session and tuple(a.reset_counters)==tuple(o.reset_counters)
    assert b.inputs['map'][0].header.stamp==stamp(10.)
    assert not b.alignment(10.6,1.6,stamp(10.6)).valid
    assert b.reason=='STALE_ODOM'


@pytest.mark.parametrize('mutation,reason', [
    (lambda m:setattr(m,'map_id','new'),'MAP_SESSION_CHANGED'),
    (lambda m:setattr(m,'epoch',2),'MAP_SESSION_CHANGED'),
    (lambda m:setattr(m,'version',0),'MAP_VERSION_REGRESSION'),
    (lambda m:setattr(m,'source_stamp',stamp(9.9)),'MAP_SOURCE_CLOCK_REGRESSION')])
def test_map_change_retires_binding(mutation,reason):
    b,m,o=setup();before=deepcopy(m)
    m.header.stamp=stamp(10.1);m.version+=1;mutation(m)
    assert not b.update('map',m,1.1) and b.retired==reason
    before.header.stamp=stamp(10.2);before.version+=1
    assert not b.update('map',before,1.2)
    assert not b.ready(10.2,1.2)


@pytest.mark.parametrize('index',range(6))
def test_every_reset_counter_retires_binding(index):
    b,m,o=setup();o.header.stamp=stamp(10.1);o.odometry.header=deepcopy(o.header)
    o.reset_counters[index]+=1
    assert not b.update('odom',o,1.1) and b.retired=='LOCALIZATION_RESET'


def test_session_change_cannot_be_implicitly_rebound():
    b,m,o=setup();o.header.stamp=stamp(10.1);o.localization_session='another'
    assert not b.update('odom',o,1.1) and b.retired=='LOCALIZATION_SESSION_CHANGED'


def test_replayed_data_does_not_refresh_receive_freshness():
    b,m,o=setup()
    assert not b.update('odom',o,1.49)
    assert b.inputs['odom'][1]==1.
    assert not b.ready(10.51,1.51)


def test_header_mismatch_retires():
    b,m,o=setup();o.header.stamp=stamp(10.1)
    assert b.update('odom',o,1.1)
    assert not b.ready(10.1,1.1) and b.retired=='ODOMETRY_HEADER_MISMATCH'


def test_invalid_map_never_becomes_ready_by_heartbeat():
    b,m,o=setup();m.header.stamp=stamp(10.1);m.valid=False
    b.update('map',m,1.1)
    assert not b.alignment(10.1,1.1,stamp(10.1)).valid
    assert b.reason=='INVALID_OR_STALE_ONLINE_MAP'


def test_clock_stall_and_backwards_clock_retire():
    b,m,o=setup()
    assert not b.ready(9.99,1.1) and b.retired=='CLOCK_REGRESSION'
    b,m,o=setup()
    assert not b.ready(10.,1.51) and b.retired=='CLOCK_STALL'


def test_identity_required():
    t=Transform();t.rotation.w=1.
    with pytest.raises(ValueError):SourceBinding('','align',t)


def test_startup_wait_does_not_activate_clock_watchdog_or_alignment_generation():
    m,o,a=inputs();m.source_stamp=stamp(10.)
    b=SourceBinding(o.localization_session,'owned',a.map_to_odom)
    assert not b.ready(1.,1.) and not b.activated
    assert not b.ready(1.,5.) and not b.retired
    invalid=b.alignment(1.,5.,stamp(1.))
    assert not invalid.valid and invalid.generation==0
    b.update('map',m,10.);b.update('odom',o,10.)
    assert b.alignment(10.,10.,stamp(10.)).valid and b.activated
    assert not b.ready(10.,10.6) and b.retired=='CLOCK_STALL'


def test_invalid_map_cannot_clear_version_high_water():
    b,m,o=setup()
    m.header.stamp=stamp(10.1);m.valid=False;m.version=0
    assert b.update('map',m,1.1)
    m.header.stamp=stamp(10.2);m.valid=True;m.version=1
    assert not b.update('map',m,1.2) and b.retired=='MAP_VERSION_REGRESSION'


def test_endpoint_loss_duplicate_and_replacement_retire():
    from types import SimpleNamespace as NS
    from uav_nav_sim.planning_sources_node import PlanningSources
    for condition in ('lost','duplicate','replaced'):
        b,m,o=setup()
        endpoints=[NS(endpoint_gid=[1])]
        node=NS(binding=b,topics={'map':'/map'},gids={},get_publishers_info_by_topic=lambda _:endpoints)
        assert PlanningSources.unique(node)
        endpoints[:] = [] if condition=='lost' else ([NS(endpoint_gid=[1])]*2 if condition=='duplicate' else [NS(endpoint_gid=[2])])
        assert not PlanningSources.unique(node) and b.retired
        endpoints[:]=[NS(endpoint_gid=[1])]
        assert not b.ready(10.1,1.1)


def test_context_waits_for_initial_binding_then_latches_active_clock_fault(monkeypatch):
    from types import SimpleNamespace as NS
    from uav_nav_sim.planning_context import PlanningGate
    import uav_nav_sim.planning_context_node as module
    mono=[1.]
    monkeypatch.setattr(module.time,'monotonic',lambda:mono[0])
    node=NS(gate=PlanningGate(),activated=False,unique_sources=lambda:True,now_s=lambda:10.)
    assert not module.PlanningContextNode.ready(node) and not node.gate.clock_fault
    mono[0]=5.
    assert not module.PlanningContextNode.ready(node) and not node.gate.clock_fault
    for name,message in zip(('map','odom','alignment'),inputs()):node.gate.update(name,message,5.)
    assert module.PlanningContextNode.ready(node) and node.activated
    mono[0]=5.6
    assert not module.PlanningContextNode.ready(node) and node.gate.clock_fault
