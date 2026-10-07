import math
import pytest
from uav_mission.flight_geometry import Alignment, Segment, ned_enu, in_region


@pytest.mark.parametrize('p', [(1.,2.,3.),(-4.,.2,1.),(0.,0.,0.)])
def test_nonidentity_alignment_roundtrip(p):
    a=Alignment((10.,-4.,.3),.7)
    assert a.to_odom(a.to_map(p))==pytest.approx(p)
    assert ned_enu(ned_enu(p))==pytest.approx(p)


def test_known_axis_and_nonidentity_rotation():
    a=Alignment((2.,3.,1.),math.pi/2)
    assert a.to_map((1.,0.,2.))==pytest.approx((2.,4.,3.))
    assert ned_enu((1.,2.,-3.))==(2.,1.,3.)


def test_path_stops_at_both_ends_and_respects_speed():
    s=Segment((0.,0.,0.),(3.,2.,2.))
    assert s.at(-1)==s.start and s.at(s.duration+1)==s.end
    dt=.001
    for k in range(1,1000):
        t=s.duration*k/1000
        v=math.dist(s.at(t-dt),s.at(t+dt))/(2*dt)
        assert v<=.60001
    assert math.dist(s.at(0),s.at(dt))/dt<.001
    assert math.dist(s.at(s.duration-dt),s.at(s.duration))/dt<.001


@pytest.mark.parametrize('p,valid', [((0,0,2),True),((7,0,2),False),((0,0,5.5),False),((0,0,-1),False)])
def test_full_envelope_region(p,valid):
    assert in_region(p)==valid


@pytest.mark.parametrize('p', [(math.nan,0,0),(math.inf,0,0),(1,2)])
def test_invalid_coordinates_rejected(p):
    with pytest.raises(ValueError):ned_enu(p)
