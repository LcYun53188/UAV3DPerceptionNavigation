"""Bind a successfully built fixture to its source and binary, not an old cache."""

# Support canonical direct execution and historical flat imports.
import sys as _sys
from pathlib import Path as _Path
_scripts = str(_Path(__file__).resolve().parents[3])
if _scripts not in _sys.path:
    _sys.path.insert(0, _scripts)
from support.paths import enable_imports as _enable_imports
_enable_imports()

from sim_validation import ROOT,file_hash,write_json
paths = ['simulation/px4/vio/plugins/motion/CMakeLists.txt',
         'simulation/px4/vio/plugins/motion/MotionCarrier.cc','scripts/build/build_vio_motion.sh',
         'scripts/simulation/vio/runtime/record_vio_motion_build.py','.deps/vio-motion-build/libvio_motion_carrier.so',
         '.deps/vio-motion-build/CMakeCache.txt']
write_json(ROOT/'.deps/vio-motion-build/build-inputs.json',
           {name:file_hash(ROOT/name) for name in paths})
