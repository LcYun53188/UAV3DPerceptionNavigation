#!/usr/bin/env python3
"""Camera monitor: RGB detections, AR planned route, stereo and diagnostics."""

# Support canonical direct execution and historical flat imports.
import sys as _sys
from pathlib import Path as _Path
_scripts = str(_Path(__file__).resolve().parents[3])
if _scripts not in _sys.path:
    _sys.path.insert(0, _scripts)
from support.paths import enable_imports as _enable_imports
_enable_imports()

from vision_monitor import main

if __name__ == '__main__':
    main()
