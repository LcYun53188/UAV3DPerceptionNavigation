import math
import pytest
from uav_mission.observation_survey import ObservationSurvey
from uav_mission.flight_geometry import in_region


def survey(region=in_region):
    return ObservationSurvey((0.,0.,2.),(0.,0.,0.),[(.8,.8,1.),(-.8,-.8,3.)],region,.35,(.6,.5,.6),0.)


def test_outside_region_rejected_before_motion():
    with pytest.raises(ValueError,match='OUTSIDE_KNOWN_REGION'):
        survey(lambda p:p[2]<2.5)


def test_each_scan_waits_for_arrival_and_finishes_at_original_anchor():
    s=survey();ros=0.
    for index in range(2):
        end=ros+s.segment.duration
        p,angle,done=s.sample(end,1.,lambda *args:False)
        assert p==s.targets[index] and s.state=='MOVE' and not done
        s.sample(end,2.,lambda *args:True)
        assert s.state=='SCAN'
        _,angle,done=s.sample(end+math.tau/.35,3.,lambda *args:True)
        assert angle==pytest.approx((index+1)*math.tau) and not done
        ros=end+math.tau/.35
    assert s.state=='RETURN'
    p,angle,done=s.sample(ros+s.segment.duration+1e-6,4.,lambda *args:True)
    assert p==pytest.approx(s.anchor) and done


def test_survey_motion_stays_inside_conservative_known_region():
    s=survey(lambda p:in_region(p,(-3,-3,-.4),(3,3,5),2.))
    for t in range(10):
        p,_,done=s.sample(t/2,0.,lambda *args:False)
        assert in_region(p,(-3,-3,-.4),(3,3,5),2.) and not done
