"""Bind a successfully built fixture to its source and binary, not an old cache."""
from sim_validation import ROOT,file_hash,write_json
paths = ['simulation/px4/vio/motion/CMakeLists.txt',
         'simulation/px4/vio/motion/MotionCarrier.cc','scripts/build_vio_motion.sh',
         'scripts/record_vio_motion_build.py','.deps/vio-motion-build/libvio_motion_carrier.so',
         '.deps/vio-motion-build/CMakeCache.txt']
write_json(ROOT/'.deps/vio-motion-build/build-inputs.json',
           {name:file_hash(ROOT/name) for name in paths})
