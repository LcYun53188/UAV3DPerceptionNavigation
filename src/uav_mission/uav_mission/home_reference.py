"""PX4-confirmed Home in its estimator frame; QGC is never a second authority."""
from dataclasses import dataclass
import math
from .flight_geometry import ned_enu


@dataclass(frozen=True)
class HomeReference:
    position_enu: tuple
    sample_us: int
    update_count: int
    manual: bool
    native_position_ned: tuple

    def evidence(self):
        return dict(source='PX4_HOME_POSITION',position_enu=list(self.position_enu),
                    native_position_ned=list(self.native_position_ned),timestamp=self.sample_us,
                    update_count=self.update_count,manual_home=self.manual)


def local_home(home, ros, *, writer_count, frame_epoch_us=0):
    # Home is an event, not a periodically renewed sensor. Its original stamp
    # can be old; the current PX4 writer and estimator epoch must still match.
    if writer_count!=1:raise ValueError('PX4_HOME_WRITER_COUNT')
    if home is None or not home.valid_lpos:raise ValueError('PX4_LOCAL_HOME_UNAVAILABLE')
    if home.timestamp<=0 or home.timestamp/1e6>ros+.05:
        raise ValueError('PX4_HOME_TIME_INVALID')
    if home.timestamp<frame_epoch_us:raise ValueError('PX4_HOME_PRECEDES_LOCAL_RESET')
    try:point=ned_enu((home.x,home.y,home.z))
    except ValueError as error:raise ValueError('PX4_LOCAL_HOME_NONFINITE') from error
    return HomeReference(point,int(home.timestamp),int(home.update_count),bool(home.manual_home),
                         (float(home.x),float(home.y),float(home.z)))


def native_rtl_home(home, local, global_position, flags, ros, *, writer_count, frame_epoch_us=0):
    reference=local_home(home,ros,writer_count=writer_count,frame_epoch_us=frame_epoch_us)
    if (not home.valid_hpos or not home.valid_alt or
            not all(math.isfinite(v) for v in (home.lat,home.lon,home.alt)) or
            not -90<=home.lat<=90 or not -180<=home.lon<=180):
        raise ValueError('PX4_GLOBAL_HOME_UNAVAILABLE')
    if (local is None or not local.xy_global or not local.z_global or
            flags is None or flags.global_position_invalid or flags.home_position_invalid):
        raise ValueError('PX4_GLOBAL_POSITION_UNAVAILABLE')
    if (global_position is None or global_position.timestamp<=0 or
            not -.05<=ros-global_position.timestamp/1e6<=.5 or
            not all(math.isfinite(v) for v in
                    (global_position.lat,global_position.lon,global_position.alt))):
        raise ValueError('PX4_GLOBAL_POSITION_STALE_OR_INVALID')
    return reference
