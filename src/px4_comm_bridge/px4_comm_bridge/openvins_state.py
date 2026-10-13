"""Adapt native OpenVINS state receipts for common pose/status pairing checks."""
import math
from types import SimpleNamespace
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

CONTRACT='openvins_full_state_world_base_v1_disarmed'
COMMIT='69488123ed9362dd44b6f28e7f4680abbff1442b'

def tracking_receipt(data):
    if len(data) not in (2,14) or not math.isfinite(data[0]) or data[0]<0 or data[0]!=round(data[0]):
        raise ValueError('VIO_NATIVE_STATE_INVALID')
    ns=round(data[0]);ready=(len(data)==14 and data[1]==1 and all(math.isfinite(v) for v in data)
        and 8<=math.sqrt(sum(v*v for v in data[2:5]))<=11)
    return SimpleNamespace(header=Header(stamp=Time(sec=ns//10**9,nanosec=ns%10**9)),
                           vo_state=1 if ready else 0,native_initialized=ready)
