"""Viewer retention requires current independent PX4 ground telemetry."""
def grounded_for_viewers(last, received, now):
    vehicle,land=last.get('vehicle'),last.get('land')
    return bool(vehicle is not None and land is not None and vehicle.arming_state==1
        and land.landed and 0<=now-received.get('vehicle',float('-inf'))<.75
        and 0<=now-received.get('land',float('-inf'))<1.5)
