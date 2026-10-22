"""Preserve invalid SDK values in raw arrays and explicit nullable diagnostics."""

# Support canonical direct execution and historical flat imports.
import sys as _sys
from pathlib import Path as _Path
_scripts = str(_Path(__file__).resolve().parents[3])
if _scripts not in _sys.path:
    _sys.path.insert(0, _scripts)
from support.paths import enable_imports as _enable_imports
_enable_imports()

import math
import numpy as np


def nullable(value):
    if isinstance(value,float) and not math.isfinite(value):return None
    if isinstance(value,dict):return {k:nullable(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [nullable(v) for v in value]
    return value


def save_raw_sdk(path, raw):
    entries=list(raw.items())
    np.savez_compressed(path,stamp=np.array([t for t,_ in entries]),
        covariance=np.array([list(m.pose.covariance) for _,m in entries]),
        position=np.array([[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z] for _,m in entries]),
        quaternion=np.array([[m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w] for _,m in entries]))
