from sim_validation import ROOT,file_hash,write_json
paths=['simulation/px4/vio/same_aircraft_motion/CMakeLists.txt',
       'simulation/px4/vio/same_aircraft_motion/SameAircraftMotion.cc',
       'scripts/build_vio_same_aircraft_motion.sh','scripts/record_vio_same_aircraft_build.py',
       '.deps/vio-same-aircraft-build/libvio_same_aircraft_fixture.so',
       '.deps/vio-same-aircraft-build/CMakeCache.txt']
write_json(ROOT/'.deps/vio-same-aircraft-build/build-inputs.json',{p:file_hash(ROOT/p) for p in paths})
