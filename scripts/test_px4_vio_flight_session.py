"""Unique gateway and aircraft state remain required through flight transitions."""
from types import SimpleNamespace
from unittest.mock import patch
from px4_vio_flight_session import FlightPoseSession


def fixture():
    s=object.__new__(FlightPoseSession)
    s.control_identity=None;s.gateway_seen=False;s.control_diagnostics=[];s.control_discovery_pending=False;s.stream=SimpleNamespace(bound=False)
    endpoints={n:[] for n in s.control_names}
    s.node=SimpleNamespace(get_publishers_info_by_topic=lambda t:endpoints[t.rsplit('/',1)[-1]])
    return s,endpoints


def endpoint(gid,name='px4_flight_gateway'):
    return SimpleNamespace(node_name=name,node_namespace='/',endpoint_gid=bytes([gid]))


def test_gateway_discovery_then_identity_cannot_change_or_disappear():
    s,e=fixture()
    assert s.controls_valid()
    e['vehicle_command']=[endpoint(1)]
    assert s.controls_valid() and not s.gateway_seen
    for i,n in enumerate(s.control_names):e[n]=[endpoint(i+1)]
    assert s.controls_valid() and s.gateway_seen
    e['vehicle_command']=[endpoint(9)]
    assert not s.controls_valid() and s.control_diagnostics[-1]['reason']=='GID'
    e['vehicle_command']=[]
    assert not s.controls_valid()


def test_second_or_unexpected_writer_is_never_accepted():
    s,e=fixture()
    e['vehicle_command']=[endpoint(1,name='other')]
    assert not s.controls_valid()
    e['vehicle_command']=[endpoint(1),endpoint(2)]
    assert not s.controls_valid()


def test_fresh_state_covers_ground_arm_lift_and_touchdown():
    s=object.__new__(FlightPoseSession)
    s.control_discovery_pending=False
    s.ros=lambda:10.
    s.received=dict(vehicle=100.,land=100.)
    s.topics=dict(vehicle='vehicle',land='land')
    s.node=SimpleNamespace(get_publishers_info_by_topic=lambda t:[object()])
    s.last=dict(vehicle=SimpleNamespace(timestamp=10000000,arming_state=1),land=SimpleNamespace(timestamp=10000000,landed=True))
    with patch('px4_vio_flight_session.time.monotonic',return_value=100.):
        assert s.state_keywords()==dict(ground=True,armed=False)
        s.last['vehicle'].arming_state=2
        assert s.state_keywords()==dict(ground=False,armed=True)
        s.last['land'].landed=False
        assert s.state_keywords()==dict(ground=False,armed=True)
        s.last['land'].landed=True
        assert s.state_keywords()==dict(ground=False,armed=True)
        s.received['vehicle']=98.
        assert s.state_keywords()==dict(ground=False,armed=False)


def test_unknown_startup_metadata_blocks_binding_but_is_terminal_after_binding():
    s,e=fixture()
    e['vehicle_command']=[endpoint(1,name='_NODE_NAME_UNKNOWN_')]
    assert s.controls_valid() and s.control_discovery_pending
    assert s.state_keywords()==dict(ground=False,armed=False)
    s.stream.bound=True
    assert not s.controls_valid()
