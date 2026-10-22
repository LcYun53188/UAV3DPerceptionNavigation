
# Support canonical direct execution and historical flat imports.
import sys as _sys
from pathlib import Path as _Path
_scripts = str(_Path(__file__).resolve().parents[3])
if _scripts not in _sys.path:
    _sys.path.insert(0, _scripts)
from support.paths import enable_imports as _enable_imports
_enable_imports()

from sim_validation import ROOT,file_hash,write_json
paths=['simulation/px4/vio/plugins/same_aircraft_motion/CMakeLists.txt',
       'simulation/px4/vio/plugins/same_aircraft_motion/SameAircraftMotion.cc',
       'scripts/build/build_vio_same_aircraft_motion.sh','scripts/simulation/vio/runtime/record_vio_same_aircraft_build.py',
       '.deps/vio-same-aircraft-build/libvio_same_aircraft_fixture.so',
       '.deps/vio-same-aircraft-build/CMakeCache.txt']
write_json(ROOT/'.deps/vio-same-aircraft-build/build-inputs.json',{p:file_hash(ROOT/p) for p in paths})
