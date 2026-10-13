// Diagnostic-only replay of SDK-consumed inputs. No ROS, PX4 or control output.
// Compile each candidate against its own reviewed header and binary.
#include <cuvslam/cuvslam2.h>
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <thread>
#ifndef CUVSLAM_REPLAY_SDK_MAJOR
#define CUVSLAM_REPLAY_SDK_MAJOR 15
#endif
using Json = nlohmann::json;
namespace fs = std::filesystem;

Json read_json(const fs::path& path) {
  std::ifstream in(path); if (!in) throw std::runtime_error("Missing " + path.string());
  return Json::parse(in);
}
std::vector<Json> read_rows(const fs::path& path) {
  std::ifstream in(path); if (!in) throw std::runtime_error("Missing " + path.string());
  std::vector<Json> rows; std::string line;
  while (std::getline(in,line)) if (!line.empty()) rows.push_back(Json::parse(line));
  return rows;
}
cuvslam::Pose transform(const Json& value) {
  Eigen::Matrix3f r;
  cuvslam::Pose pose;
  for (int i=0;i<3;++i) {
    for (int j=0;j<3;++j) r(i,j)=value[i][j].get<float>();
    pose.translation[i]=value[i][3].get<float>();
  }
  Eigen::Quaternionf q(r);q.normalize();
  pose.rotation={q.x(),q.y(),q.z(),q.w()};return pose;
}
struct Pixels { int width,height; std::vector<unsigned char> data; };
Pixels read_tga(const fs::path& path) {
  // SDK 15 dump writes uncompressed, top-origin, 8-bit grey TGA.
  std::ifstream in(path,std::ios::binary);unsigned char h[18];
  if (!in.read(reinterpret_cast<char*>(h),18)) throw std::runtime_error("Invalid TGA header");
  if(h[1]!=0 || h[2]!=3 || h[16]!=8 || h[17]!=32)
    throw std::runtime_error("Only SDK top-origin grey TGA is supported");
  Pixels p{h[12]+256*h[13],h[14]+256*h[15],{}};
  if(p.width<=0 || p.height<=0) throw std::runtime_error("Invalid TGA size");
  p.data.resize(static_cast<size_t>(p.width)*p.height);in.seekg(h[0],std::ios::cur);
  if(!in.read(reinterpret_cast<char*>(p.data.data()),p.data.size()))
    throw std::runtime_error("Truncated TGA pixels");
  return p;
}
int main(int argc,char** argv) {
  try {
    if(argc<5) throw std::runtime_error("Usage: replay SDK_INPUT VIO_PARAMS vio|vo OUTPUT.jsonl [--sync-sba] [--realtime] [--keyframe-max-seconds=N]");
    fs::path folder=fs::canonical(argv[1]),output=argv[4];
    if(fs::exists(output)) throw std::runtime_error("Refuse output overwrite");
    std::string mode=argv[3];if(mode!="vio" && mode!="vo") throw std::runtime_error("Invalid mode");
    bool async_sba=true,realtime=false;
    fs::path physical_calibration;
    int64_t keyframe_seconds=60;
    for(int i=5;i<argc;++i) {
      std::string flag=argv[i];
      if(flag=="--sync-sba") async_sba=false;
      else if(flag=="--realtime") realtime=true;
      else if(flag.rfind("--physical-calibration=",0)==0) physical_calibration=flag.substr(23);
      else if(flag.rfind("--keyframe-max-seconds=",0)==0) {
#if CUVSLAM_REPLAY_SDK_MAJOR < 16
        throw std::runtime_error("Keyframe track options require reviewed SDK16 API");
#else
        std::string value=flag.substr(23);size_t used=0;
        int64_t seconds=std::stoll(value,&used);
        if(used!=value.size() || seconds<1 || seconds>60)
          throw std::runtime_error("Keyframe interval must be an integer in [1,60]");
        keyframe_seconds=seconds;
#endif
      }
      else throw std::runtime_error("Unknown replay option: "+flag);
    }
    Json config=read_json(folder/"stereo.edex")[0];
    Json authority=nullptr;
    if(!physical_calibration.empty()) {
      authority=read_json(physical_calibration);
      if(authority["schema"]!=1 || authority["contract"]!="physical_base_sdk_opencv_imu_v1" ||
          authority["relative_extrinsics_preserved"]!=true || authority["rig_from_cameras"].size()!=2)
        throw std::runtime_error("Unsupported physical replay calibration");
      for(size_t i=0;i<2;++i)config["cameras"][i]["transform"]=authority["rig_from_cameras"][i];
      config["imu"]["transform"]=authority["rig_from_imu"];
    }
    Json params=read_json(argv[2])["visual_slam"]["ros__parameters"];
    int32_t major,minor,patch;std::string version(cuvslam::GetVersion(&major,&minor,&patch));
    if(major!=CUVSLAM_REPLAY_SDK_MAJOR || config["cuvslam_version_major"]!=15)
      throw std::runtime_error("Replay requires compile-time reviewed SDK and recorded SDK 15 input");
    cuvslam::Rig rig;
    for(const auto& entry:config["cameras"]) {
      cuvslam::Camera camera;
      camera.size=entry["intrinsics"]["size"].get<cuvslam::IntArray<2>>();
      camera.focal=entry["intrinsics"]["focal"].get<cuvslam::Array<2>>();
      camera.principal=entry["intrinsics"]["principal"].get<cuvslam::Array<2>>();
      if(entry["intrinsics"]["distortion_model"]!="pinhole")
        throw std::runtime_error("Only reviewed pinhole dump supported");
      camera.rig_from_camera=transform(entry["transform"]);rig.cameras.push_back(camera);
    }
    if(rig.cameras.size()!=2) throw std::runtime_error("Two cameras required");
    if(mode=="vio") {
      cuvslam::ImuCalibration imu;
      imu.rig_from_imu=transform(config["imu"]["transform"]);
      imu.gyroscope_noise_density=params["gyro_noise_density"].get<float>();
      imu.gyroscope_random_walk=params["gyro_random_walk"].get<float>();
      imu.accelerometer_noise_density=params["accel_noise_density"].get<float>();
      imu.accelerometer_random_walk=params["accel_random_walk"].get<float>();
      imu.frequency=params["calibration_frequency"].get<float>();rig.imus.push_back(imu);
    }
    auto cfg=cuvslam::Odometry::GetDefaultConfig();auto settings=config["configuration"];
    cfg.odometry_mode=mode=="vio"?cuvslam::Odometry::OdometryMode::Inertial:
                                  cuvslam::Odometry::OdometryMode::Multicamera;
    cfg.multicam_mode=static_cast<cuvslam::Odometry::MulticameraMode>(settings["multicam_mode"].get<int>());
    cfg.use_gpu=settings["use_gpu"];cfg.use_motion_model=settings["use_motion_model"];
    cfg.use_denoising=settings["use_denoising"];cfg.rectified_stereo_camera=settings["rectified_stereo_camera"];
    // Explicit diagnostic instrumentation; may change execution, never qualification.
    cfg.enable_observations_export=true;cfg.enable_landmarks_export=true;
    cfg.async_sba=async_sba;
#if CUVSLAM_REPLAY_SDK_MAJOR >= 16
    auto track_options=cuvslam::Odometry::GetDefaultTrackOptions();
    track_options.kf_max_timedelta_between_kfs_s=keyframe_seconds;
#endif
    auto frames=read_rows(folder/"frame_metadata.jsonl"),imus=read_rows(folder/"IMU.jsonl");
    std::ofstream out(output);if(!out) throw std::runtime_error("Cannot create output");
    cuvslam::WarmUpGPU();cuvslam::Odometry tracker(rig,cfg);size_t imu_index=0;
    int64_t last_frame=-1,last_imu=-1;
    auto replay_start=std::chrono::steady_clock::now();
    int64_t first_stamp=frames.at(0)["cams"][0]["timestamp"];
    for(const auto& frame:frames) {
      int64_t stamp=frame["cams"][0]["timestamp"];
      if(realtime) std::this_thread::sleep_until(replay_start+std::chrono::nanoseconds(stamp-first_stamp));
      if(stamp<=last_frame) throw std::runtime_error("Unordered frames");last_frame=stamp;
      while(imu_index<imus.size() && imus[imu_index]["timestamp"].get<int64_t>()<=stamp) {
        const auto& row=imus[imu_index++];int64_t t=row["timestamp"];
        if(t<=last_imu) throw std::runtime_error("Unordered IMU");last_imu=t;
        if(mode=="vio") {
          cuvslam::ImuMeasurement m;m.timestamp_ns=t;
          m.angular_velocities={row["AngularVelocityX"],row["AngularVelocityY"],row["AngularVelocityZ"]};
          m.linear_accelerations={row["LinearAccelerationX"],row["LinearAccelerationY"],row["LinearAccelerationZ"]};
          tracker.RegisterImuMeasurement(0,m);
        }
      }
      std::vector<Pixels> pixels; cuvslam::Odometry::ImageSet images;
      for(const auto& camera:frame["cams"]) {
        if(camera["timestamp"].get<int64_t>()!=stamp) throw std::runtime_error("Unsynced stereo");
        auto path=fs::canonical(folder/camera["filename"].get<std::string>());
        auto relative=path.lexically_relative(folder);
        if(relative.empty() || *relative.begin()=="..") throw std::runtime_error("Unsafe image path");
        pixels.push_back(read_tga(path));
      }
      for(size_t i=0;i<pixels.size();++i) {
        cuvslam::Image image{};auto& p=pixels[i];image.pixels=p.data.data();
        image.width=p.width;image.height=p.height;image.pitch=p.width;
        image.encoding=cuvslam::ImageData::Encoding::MONO;
        image.data_type=cuvslam::ImageData::DataType::UINT8;image.is_gpu_mem=false;
        image.timestamp_ns=stamp;image.camera_index=frame["cams"][i]["id"];images.push_back(image);
      }
      auto start=std::chrono::steady_clock::now();
#if CUVSLAM_REPLAY_SDK_MAJOR >= 16
      auto estimate=tracker.Track(images,{}, {},track_options);
#else
      auto estimate=tracker.Track(images);
#endif
      double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
      Json row={{"stamp",stamp/1e9},{"timestamp_ns",stamp},{"valid",estimate.world_from_rig.has_value()},
                {"track_ms",ms},{"mode",mode},{"sdk_version",version},
                {"async_sba",async_sba},{"realtime",realtime},
                {"body_reference_authority",authority.is_null()?"SDK15 debug dump; body translation unverified":"physical mount TF"},
                {"physical_calibration",authority},
                {"keyframe_max_seconds",CUVSLAM_REPLAY_SDK_MAJOR>=16?Json(keyframe_seconds):Json(nullptr)},
                {"covariance_semantics","raw SDK matrix; parameterization requires matched source review"}};
      if(estimate.world_from_rig) {
        auto& pose=*estimate.world_from_rig;row["position_cv"]=pose.pose.translation;
        row["quaternion_cv"]=pose.pose.rotation;row["covariance_rot_pos_right"]=pose.covariance;
        cuvslam::Odometry::State state;tracker.GetState(state);
        row["keyframe"]=state.keyframe;row["warming_up"]=state.warming_up;
        row["observations"]=state.observations.size();row["landmarks"]=state.landmarks.size();
        row["gravity_cv"]=state.gravity?Json(*state.gravity):Json(nullptr);
#if CUVSLAM_REPLAY_SDK_MAJOR >= 16
        if(mode=="vio") {
          auto inertial=tracker.GetImuState();
          row["imu_state"]=inertial?Json{{"velocity",inertial->velocity},
              {"gyro_bias",inertial->gyro_bias},{"acc_bias",inertial->acc_bias}}:Json(nullptr);
        }
#endif
      }
      out<<row.dump()<<'\n';
    }
    std::cout<<version<<" "<<mode<<" frames="<<frames.size()<<" IMU consumed="<<imu_index<<'\n';
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
