"""Finite ENU/NED conversion, fixed map alignment and stopped quintic paths."""
import math


def finite3(values):
    if len(values) != 3 or not all(math.isfinite(v) for v in values):
        raise ValueError('Expected finite 3-vector')
    return tuple(float(v) for v in values)


def ned_enu(values):
    x, y, z = finite3(values)
    return y, x, -z


class Alignment:
    def __init__(self, translation=(10., -4., .3), yaw=.35):
        self.translation = finite3(translation)
        if not math.isfinite(yaw):
            raise ValueError('Invalid alignment yaw')
        self.yaw = yaw

    def to_map(self, odom):
        x, y, z = finite3(odom)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        a, b, d = self.translation
        return a+c*x-s*y, b+s*x+c*y, d+z

    def to_odom(self, point):
        a, b, d = self.translation
        x, y, z = finite3(point)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return c*(x-a)+s*(y-b), -s*(x-a)+c*(y-b), z-d


def distance(a, b):
    return math.dist(finite3(a), finite3(b))


class Segment:
    def __init__(self, start, end, speed=.6, acceleration=.5, jerk=.6):
        self.start, self.end = finite3(start), finite3(end)
        if not all(math.isfinite(x) and x > 0 for x in (speed, acceleration, jerk)):
            raise ValueError('Invalid motion limits')
        length = distance(start, end)
        # Exact maxima of the normalized quintic derivatives (conservative accel).
        self.duration = max(1., 1.875*length/speed,
                            math.sqrt(6*length/acceleration), (60*length/jerk)**(1/3))

    def at(self, elapsed):
        if not math.isfinite(elapsed):
            raise ValueError('Invalid trajectory time')
        t = min(1., max(0., elapsed/self.duration))
        u = 10*t**3-15*t**4+6*t**5
        return tuple(a+(b-a)*u for a, b in zip(self.start, self.end))


def in_region(point, lower=(-8., -8., -.4), upper=(8., 8., 6.), margin=2.):
    point = finite3(point)
    # Ground support is intentional for takeoff/landing; ceiling/horizontal inflated.
    return (all(lower[i]+margin <= point[i] <= upper[i]-margin for i in (0, 1))
            and lower[2] <= point[2] <= upper[2]-.8)
