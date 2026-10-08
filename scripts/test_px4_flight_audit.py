"""Independent truth must not report an interrupted route as an arrival."""
import sys
from pathlib import Path
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parent))
from run_px4_flight_tasks import completed_waypoint_truth


@pytest.mark.parametrize('next_phase',['LEASE_BRAKE','CANCEL_BRAKE','PAUSING','FAULT'])
def test_stopped_child_does_not_count_as_reached_waypoint(next_phase):
    events=[dict(phase='NAVIGATE',mono=0.,target_enu=(3.,2.,2.)),dict(phase=next_phase,mono=5.)]
    truth=[dict(mono=4.9,position=(.8,.5,2.))]
    assert completed_waypoint_truth(events,truth,(0.,0.,0.),(0.,0.,0.))==[]


def test_completed_child_uses_initial_alignment_and_actual_truth():
    events=[dict(phase='NAVIGATE',mono=0.,target_enu=(3.,2.,2.)),dict(phase='HOVER',mono=5.)]
    truth=[dict(mono=4.9,position=(13.1,-2.,2.3))]
    reached=completed_waypoint_truth(events,truth,(0.,0.,0.),(10.,-4.,.3))
    assert len(reached)==1
    assert reached[0]['expected_world']==[13.,-2.,2.3]
    assert reached[0]['error_m']==pytest.approx(.1)
