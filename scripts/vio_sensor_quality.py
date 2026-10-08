"""Numerical acceptance of observed sensor timestamps; independent of ROS callbacks."""
import math
import numpy as np


def sample_window(samples, now, expected_hz, window=5.):
    """Require a sustained recent stream, ordered source stamps and bounded gaps."""
    stamps = np.asarray(samples, dtype=float)
    recent = stamps[(stamps >= now-window) & (stamps <= now+.05)]
    delta = np.diff(recent)
    span = float(recent[-1]-recent[0]) if len(recent) > 1 else 0.
    rate = float((len(recent)-1)/span) if span > 0 else 0.
    max_gap = float(max(delta)) if len(delta) else None
    checks = dict(finite=bool(np.isfinite(stamps).all()),
        ordered=bool(len(stamps) > 1 and np.all(np.diff(stamps) > 0)),
        span=span >= window-.2,
        rate=.9*expected_hz <= rate <= 1.1*expected_hz,
        gap=max_gap is not None and max_gap <= 2./expected_hz+1e-6,
        fresh=bool(len(stamps) and -.05 <= now-stamps[-1] <= .2))
    return dict(passed=all(checks.values()),checks=checks,count=len(recent),
                span_s=span,rate_hz=rate,max_gap_s=max_gap)


def stereo_pairs(left, right, now):
    """Match a complete steady window, excluding the one-frame callback boundary."""
    a = np.asarray([t for t in left if now-5 <= t <= now-.1],dtype=float)
    b = np.asarray([t for t in right if now-5 <= t <= now-.1],dtype=float)
    if not len(a) or not len(b):
        return dict(passed=False,matched_fraction=0.,max_error_s=None)
    errors = np.min(np.abs(a[:,None]-b[None,:]),axis=1)
    reverse = np.min(np.abs(b[:,None]-a[None,:]),axis=1)
    fraction = min(float(np.mean(errors <= .001)),float(np.mean(reverse <= .001)))
    return dict(passed=fraction >= .95,matched_fraction=fraction,
                max_error_s=float(max(max(errors),max(reverse))))


def static_imu(samples):
    """FLU IMU at rest must measure upward gravity and negligible angular rate."""
    a = np.asarray(samples,dtype=float)
    if a.ndim != 2 or a.shape[0] < 100 or a.shape[1] != 6 or not np.isfinite(a).all():
        return dict(passed=False)
    mean = a.mean(axis=0)
    return dict(passed=bool(np.linalg.norm(mean[:3]-[0,0,9.8]) < .25
                and np.linalg.norm(mean[3:]) < .05),mean=mean.tolist())
