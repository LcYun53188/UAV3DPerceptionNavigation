"""Original MTF input integrity; quality zero remains an unavailable measurement.

The VIO-primary profile checks sensor connectivity without granting confidence
to rejected auxiliary measurements. Strict joint/flow qualification needs quality.
"""
import math


def mtf_source_health(messages, received, writers, now, ros, *, require_flow_quality=True):
    for key in ('flow','range'):
        if writers.get(key)!=1:
            return False,'MTF_WRITER_COUNT:'+key
        if key not in messages or key not in received:
            return False,'MTF_MISSING:'+key
        m=messages[key]
        stamp=(m.timestamp_sample if key=='flow' else m.timestamp)/1e6
        if not (stamp>0 and -.05<=ros-stamp<=.2 and 0<=now-received[key]<=.2):
            return False,'MTF_STALE:'+key
        if key=='flow':
            if (m.quality<(1 if require_flow_quality else 0) or not 0<m.integration_timespan_us<=50000
                    or not all(math.isfinite(v) for v in m.pixel_flow)
                    or abs(m.max_flow_rate-7)>1e-5 or abs(m.min_ground_distance-.08)>1e-5
                    or abs(m.max_ground_distance-8)>1e-5):
                return False,'MTF_FLOW_INVALID'
        elif (m.orientation!=25 or not math.isfinite(m.current_distance)
                or not .02<=m.current_distance<=8 or abs(m.min_distance-.02)>1e-5
                or abs(m.max_distance-8)>1e-5):
            return False,'MTF_RANGE_INVALID'
    return True,''
