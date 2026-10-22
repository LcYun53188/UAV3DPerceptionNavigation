"""Source-rate evidence uses ROS/sample time; retirement uses monotonic time."""

# Support canonical direct execution and historical flat imports.
import sys as _sys
from pathlib import Path as _Path
_scripts = str(_Path(__file__).resolve().parents[3])
if _scripts not in _sys.path:
    _sys.path.insert(0, _scripts)
from support.paths import enable_imports as _enable_imports
_enable_imports()



def source_window(poses,ros,boundary):
    return [p for p in poses if ros-5<=p['stamp']<=ros and p['mono']<boundary]
