"""Native static evidence rejects movement; failed early attempts retain dynamic history."""
from pathlib import Path
import subprocess
import pytest

ROOT = Path(__file__).resolve().parents[2]
from build_vio_sdk16_stationary import FOLDER
CANDIDATE = FOLDER/'src'


@pytest.fixture(scope='module')
def detector(tmp_path_factory):
    folder=tmp_path_factory.mktemp('stationary-detector')
    source=folder/'test.cpp'
    source.write_text(r'''
#include "vio_stationary_initialization.hpp"
#include <cassert>
#include <limits>
#include <string>
using namespace uav_vio;
int main(int argc,char** argv) {
  const std::string mode=argv[1];
  std::vector<StaticVisualSample> vision;
  std::vector<StaticImuSample> imu;
  for (int i=0;i<6;++i) vision.push_back({int64_t(i)*1000000000LL,{4.,2.,-1.},0.});
  for (int i=0;i<=1250;++i) imu.push_back({int64_t(i)*4000000LL,{0.,-9.81,0.},{.001,.0003,0.}});
  if (mode=="translation") for (int i=0;i<6;++i) vision[i].position[0]+=.001*i;
  if (mode=="rotation") vision[3].angle=.003;
  if (mode=="angular_motion") for (auto& m:imu) m.gyro_body[0]=.03;
  if (mode=="accel_vibration") for (int i=0;i<1250;++i) imu[i].acceleration_world[0]=i%2 ? .1 : -.1;
  if (mode=="gyro_vibration") for (int i=0;i<1250;++i) imu[i].gyro_body[1]=i%2 ? .01 : -.01;
  if (mode=="gravity") for (auto& m:imu) m.acceleration_world[1]=-8.;
  if (mode=="gap") imu.erase(imu.begin()+100,imu.begin()+110);
  if (mode=="duplicate") imu[100].stamp=imu[99].stamp;
  if (mode=="nan") imu[100].acceleration_world[1]=std::numeric_limits<double>::quiet_NaN();
  if (mode=="short") vision.pop_back();
  if (mode=="stale") for (auto& m:imu) m.stamp+=100000000LL;
  StaticEstimate estimate; estimate.gyro_bias[0]=99.;
  const bool accepted=EstimateStationary(vision,imu,estimate);
  assert(accepted==(mode=="stationary"));
  if (accepted) {
    assert(std::abs(estimate.gyro_bias[0]-.001)<1e-12);
    assert(std::abs(estimate.acceleration_world[1]+9.81)<1e-10);
    assert(vision.front().position[0]==4.); // Input visual poses are never changed.
  } else assert(estimate.gyro_bias[0]==99.);
}
''')
    output=folder/'test'
    subprocess.run(['g++','-std=c++17','-O2','-I'+str(ROOT/'scripts/simulation/vio/sdk'),str(source),'-o',str(output)],check=True)
    return output


@pytest.mark.parametrize('mode',['stationary','translation','rotation','angular_motion','accel_vibration',
    'gyro_vibration','gravity','gap','duplicate','nan','short','stale'])
def test_stereo_and_original_imu_evidence(detector,mode):
    subprocess.run([str(detector),mode],check=True)


def test_original_near_stereo_tracks_distinguish_pose_solver_drift_from_image_motion(tmp_path):
    source=tmp_path/'stereo.cpp'
    source.write_text(r'''
#include "vio_stationary_initialization.hpp"
#include <cassert>
using namespace uav_vio;
int main() {
  std::vector<StaticFeatureTrack> tracks;
  for(int cam=0;cam<2;++cam)for(int i=0;i<40;++i) {
    StaticFeatureTrack track;track.camera=cam;
    for(int frame=0;frame<6;++frame) track.pixels.push_back({double(i),double(i+cam)});
    tracks.push_back(track);
  }
  assert(StationaryFeatures(tracks,6).stationary);
  auto moving=tracks;
  for(auto& t:moving)for(int i=0;i<6;++i)t.pixels[i][0]+=.1*i;
  assert(!StationaryFeatures(moving,6).stationary);
  auto monocular=tracks;monocular.resize(40);
  assert(!StationaryFeatures(monocular,6).stationary);
  auto missing=tracks;
  for(auto& t:missing)t.pixels.pop_back();
  assert(!StationaryFeatures(missing,6).stationary);
  std::vector<StaticVisualSample> visual;
  std::vector<StaticImuSample> imu;
  for(int i=0;i<6;++i)visual.push_back({int64_t(i)*1000000000LL,{.004*i,0.,0.},0.});
  for(int i=0;i<=1250;++i)imu.push_back({int64_t(i)*4000000LL,{0.,-9.81,0.},{0.,0.,0.}});
  StaticEstimate estimate;
  assert(!EstimateStationary(visual,imu,estimate));
  assert(EstimateStationary(visual,imu,estimate,StationaryFeatures(tracks,6).stationary));
  assert(!EstimateStationary(visual,imu,estimate,StationaryFeatures(moving,6).stationary));
  imu[600].acceleration_world[0]=2.;
  assert(!EstimateStationary(visual,imu,estimate,true)); // Image stillness alone cannot bypass inertial evidence.
}
''')
    output=tmp_path/'stereo'
    subprocess.run(['g++','-std=c++17','-O2','-I'+str(ROOT/'scripts/simulation/vio/sdk'),str(source),'-o',str(output)],check=True)
    subprocess.run([str(output)],check=True)


def test_zero_flow_seed_requires_exact_patch_identity_and_small_prediction(tmp_path):
    source=tmp_path/'seed.cpp'
    source.write_text(r'''
#include "vio_zero_flow_seed.hpp"
#include <cassert>
#include <limits>
int main() {
  using uav_vio::PreferZeroFlowSeed;
  assert(PreferZeroFlowSeed(0.f,0.f,0.f));
  assert(PreferZeroFlowSeed(0.f,-1.f,1.f));
  assert(!PreferZeroFlowSeed(1e-20f,0.f,0.f));
  assert(!PreferZeroFlowSeed(0.f,1.001f,0.f));
  assert(!PreferZeroFlowSeed(0.f,0.f,-1.001f));
  assert(!PreferZeroFlowSeed(std::numeric_limits<float>::quiet_NaN(),0.f,0.f));
  assert(!PreferZeroFlowSeed(0.f,std::numeric_limits<float>::quiet_NaN(),0.f));
  assert(!PreferZeroFlowSeed(0.f,0.f,std::numeric_limits<float>::infinity()));
}
''')
    output=tmp_path/'seed'
    subprocess.run(['g++','-std=c++17','-O2','-I'+str(ROOT/'scripts/simulation/vio/sdk'),str(source),'-o',str(output)],check=True)
    subprocess.run([str(output)],check=True)


def test_native_dynamic_fallback_does_not_reset_after_failed_static_attempt(tmp_path):
    if not (CANDIDATE/'libs/pipelines/tracker_state_machine.cpp').exists():
        pytest.skip('Build the pinned stationary candidate to exercise its native state machine')
    source=tmp_path/'state.cpp'
    source.write_text(r'''
#include "pipelines/tracker_state_machine.h"
#include <cassert>
using cuvslam::pipelines::StateMachine;
int main() {
  StateMachine stationary;
  int early=0;
  stationary.register_gravity_estimation_callback([&](size_t n){assert(n==6);++early;return true;});
  for (int i=0;i<5;++i) assert(stationary.update_frame_state(true,true,int64_t(i)*1000000000LL)==StateMachine::Uninitialized);
  assert(early==0);
  assert(stationary.update_frame_state(true,true,5000000000LL)==StateMachine::Ok);
  StateMachine moving; int normal=0, attempts=0;
  moving.register_gravity_estimation_callback([&](size_t n){if(n==6){++attempts;return false;}assert(n==20);++normal;return true;});
  for(int i=0;i<19;++i) assert(moving.update_frame_state(true,true,int64_t(i)*1000000000LL)==StateMachine::Uninitialized);
  assert(moving.update_frame_state(true,true,19000000000LL)==StateMachine::Ok);
  assert(attempts>0 && normal==1);
}
''')
    output=tmp_path/'state'
    subprocess.run(['g++','-std=c++17','-O1','-I'+str(CANDIDATE/'libs'),'-I/usr/include/eigen3',
                    str(source),str(CANDIDATE/'libs/pipelines/tracker_state_machine.cpp'),'-o',str(output)],check=True)
    subprocess.run([str(output)],check=True)


@pytest.mark.parametrize('baseline',[False,True])
def test_zero_residual_native_pnp_refreshes_observation_information(tmp_path,baseline):
    import shlex
    from build_vio_sdk16_stationary import BASE
    candidate=BASE if baseline else FOLDER
    build=candidate/'build'
    link=build/'libs/cuvslam/CMakeFiles/cuvslam.dir/link.txt'
    if not link.exists() or not (build/'libs/imu/libimu.a').exists():
        pytest.skip('Build the native stationary candidate for the information regression')
    tokens=shlex.split(link.read_text())
    # Native link paths are relative to the libs/cuvslam build directory.
    libraries=[]
    for index,token in enumerate(tokens):
        if not token.startswith('-') and token.endswith(('.a','.so')) and tokens[index-1]!='-o':
            path=Path(token)
            libraries.append(str(path if path.is_absolute() else (build/'libs/cuvslam'/path).resolve()))
    output=tmp_path/'information'
    circular=next((candidate/'dependency-sources/circularbuffer').rglob('circular_buffer.h')).parent
    subprocess.run(['g++','-std=c++17','-O1','-I'+str(candidate/'src/libs'),'-I/usr/include/eigen3',
        '-I'+str(circular),
        str(ROOT/'scripts/simulation/vio/sdk/vio_low_residual_information_test.cpp'),
        '-Wl,--start-group',*libraries,'-Wl,--end-group','-ldl','-lrt','-pthread',
        '-Wl,-rpath,/usr/local/cuda-13.2/lib64','-o',str(output)],check=True)
    if baseline:
        import resource,signal
        def no_core():resource.setrlimit(resource.RLIMIT_CORE,(0,0))
        result=subprocess.run([str(output)],stderr=subprocess.PIPE,preexec_fn=no_core)
        assert result.returncode==-signal.SIGABRT
        assert b'Matrix15T::Identity' in result.stderr
    else:
        subprocess.run([str(output)],check=True)
