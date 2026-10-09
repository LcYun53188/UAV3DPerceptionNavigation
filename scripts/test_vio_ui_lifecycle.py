from types import SimpleNamespace
from vio_ui_lifecycle import grounded_for_viewers


def test_retention_requires_disarmed_landed_and_fresh_independent_telemetry():
    last=dict(vehicle=SimpleNamespace(arming_state=1),land=SimpleNamespace(landed=True))
    received=dict(vehicle=10.,land=10.)
    assert grounded_for_viewers(last,received,10.1)
    assert not grounded_for_viewers({},received,10.1)
    assert not grounded_for_viewers(last,{},10.1)
    assert not grounded_for_viewers(last,received,10.75)
    assert not grounded_for_viewers(last,dict(vehicle=11.4,land=10.),11.5)
    assert not grounded_for_viewers(last,received,9.)
    last['vehicle'].arming_state=2
    assert not grounded_for_viewers(last,received,10.1)
    last['vehicle'].arming_state=1;last['land'].landed=False
    assert not grounded_for_viewers(last,received,10.1)
