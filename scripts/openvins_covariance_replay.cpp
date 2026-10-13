// Pixel/IMU replay only. No ground truth input, ROS publishers, or controls.
#include <core/VioManager.h>
#include <state/StateHelper.h>
#include <state/State.h>
#include <utils/print.h>
#include <utils/sensor_data.h>
#include <utils/opencv_yaml_parse.h>
#include <nlohmann/json.hpp>
#include <Eigen/Geometry>
#include <filesystem>
#include <fstream>
#include <iostream>
#include "openvins_pose_covariance.hpp"
using Json=nlohmann::json;namespace fs=std::filesystem;
Json load(const fs::path& p){std::ifstream in(p);if(!in)throw std::runtime_error("Missing input");return Json::parse(in);}
std::vector<Json> rows(const fs::path& p){std::ifstream in(p);std::vector<Json> out;std::string s;
if(!in)throw std::runtime_error("Missing rows");while(std::getline(in,s))if(!s.empty())out.push_back(Json::parse(s));return out;}
cv::Mat image(const fs::path& p){std::ifstream in(p,std::ios::binary);unsigned char h[18];
if(!in.read(reinterpret_cast<char*>(h),18)||h[1]!=0||h[2]!=3||h[16]!=8||h[17]!=32)
throw std::runtime_error("Only top-origin SDK grey TGA supported");
cv::Mat m(h[14]+256*h[15],h[12]+256*h[13],CV_8UC1);in.seekg(h[0],std::ios::cur);
if(!in.read(reinterpret_cast<char*>(m.data),m.total()))throw std::runtime_error("Truncated pixels");return m;}
int main(int argc,char** argv){try{
if(argc!=4)throw std::runtime_error("Usage: openvins_replay SDK_INPUT CONFIG_DIR OUTPUT.jsonl");
fs::path input=fs::canonical(argv[1]),config=fs::canonical(argv[2]),output=argv[3];
if(fs::exists(output))throw std::runtime_error("Refuse output overwrite");
auto parser=std::make_shared<ov_core::YamlParser>((config/"estimator.yaml").string());
ov_core::Printer::setPrintLevel(std::getenv("OPENVINS_REPLAY_VERBOSE")?"DEBUG":"WARNING");ov_msckf::VioManagerOptions params;params.print_and_load(parser);
params.use_multi_threading_subs=false;params.use_multi_threading_pubs=false;
if(!parser->successful())throw std::runtime_error("Invalid estimator configuration");
auto app=std::make_shared<ov_msckf::VioManager>(params);
auto tf=load(config/"imu_from_base.json");Eigen::Matrix3d imu_from_base;Eigen::Vector3d t;
for(int i=0;i<3;++i){t(i)=tf[i][3].get<double>();for(int j=0;j<3;++j)imu_from_base(i,j)=tf[i][j].get<double>();}
auto frames=rows(input/"frame_metadata.jsonl"),imus=rows(input/"IMU.jsonl");size_t index=0;std::ofstream out(output);
int64_t previous=-1;
for(const auto& frame:frames){int64_t stamp=frame["cams"][0]["timestamp"];
if(stamp<=previous)throw std::runtime_error("Unordered images");previous=stamp;
// Include one following IMU measurement for endpoint interpolation.
while(index<imus.size()){
const auto& v=imus[index++];ov_core::ImuData m;m.timestamp=v["timestamp"].get<int64_t>()/1e9;
m.wm << v["AngularVelocityX"].get<double>(),v["AngularVelocityY"].get<double>(),v["AngularVelocityZ"].get<double>();
m.am << v["LinearAccelerationX"].get<double>(),v["LinearAccelerationY"].get<double>(),v["LinearAccelerationZ"].get<double>();app->feed_measurement_imu(m);
if(v["timestamp"].get<int64_t>()>stamp)break;}
ov_core::CameraData camera;camera.timestamp=stamp/1e9;
for(const auto& c:frame["cams"]){if(c["timestamp"].get<int64_t>()!=stamp)throw std::runtime_error("Unsynced stereo");
auto path=fs::canonical(input/c["filename"].get<std::string>());auto relative=path.lexically_relative(input);
if(relative.empty()||*relative.begin()=="..")throw std::runtime_error("Unsafe image path");
camera.sensor_ids.push_back(c["id"]);camera.images.push_back(image(path));camera.masks.push_back(cv::Mat::zeros(camera.images.back().size(),CV_8UC1));}
app->feed_measurement_camera(camera);Json row={{"stamp",stamp/1e9},{"initialized",app->initialized()}};
if(app->initialized()){
auto state=app->get_state();if(std::abs(state->_timestamp-camera.timestamp)>1e-7)throw std::runtime_error("Stale posterior state");
const Eigen::Matrix3d r=state->_imu->Rot().transpose();const Eigen::Vector3d p=state->_imu->pos()+r*t;
Eigen::Quaterniond q(r*imu_from_base);q.normalize();
std::vector<std::shared_ptr<ov_type::Type>> order{state->_imu->pose()->q(),state->_imu->pose()->p()};
Eigen::Matrix<double,6,6> marginal=ov_msckf::StateHelper::get_marginal_covariance(state,order);
const auto c=OpenVinsBaseCovariance(marginal,r,t);std::vector<double> packed;
for(int i=0;i<6;++i)for(int j=0;j<6;++j)packed.push_back(c(i,j));
row["position"]={p.x(),p.y(),p.z()};row["quaternion"]={q.x(),q.y(),q.z(),q.w()};row["covariance"]=packed;
row["gravity_mag"]=params.gravity_mag;row["use_fej"]=params.state_options.do_fej;
}out<<row.dump()<<'\n';}
std::cout<<"OpenVINS replay frames="<<frames.size()<<" IMU="<<index<<'\n';
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
