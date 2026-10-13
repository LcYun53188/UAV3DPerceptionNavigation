"""Exclude estimator initialization using original sample time, never wall time."""
import math


class SourceWarmup:
    def __init__(self, seconds=0.):
        if not math.isfinite(seconds) or seconds < 0:
            raise ValueError('Invalid source warmup duration')
        self.duration_ns = round(seconds * 1_000_000_000)
        self.first_ns = None

    def ready(self, stamp):
        sample_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if sample_ns <= 0:
            raise ValueError('VIO_SAMPLE_STALE')
        # Startup can restart before binding. The caller still checks ROS clock
        # rollback and latches every discontinuity after source binding.
        if self.first_ns is None or sample_ns < self.first_ns:
            self.first_ns = sample_ns
        return sample_ns - self.first_ns >= self.duration_ns
