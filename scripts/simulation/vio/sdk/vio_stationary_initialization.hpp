#pragma once
// Static initialization evidence from stereo poses and original IMU samples.
// No truth, flight-controller state, output pose correction, or runtime ZUPT.
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>
#include <algorithm>

namespace uav_vio {
using Vec3 = std::array<double, 3>;
struct StaticVisualSample { int64_t stamp; Vec3 position; double angle; };
struct StaticImuSample { int64_t stamp; Vec3 acceleration_world; Vec3 gyro_body; };
struct StaticEstimate { Vec3 acceleration_world{}; Vec3 gyro_bias{}; };
struct StaticFeatureTrack { int camera; std::vector<std::array<double,2>> pixels; };
struct StaticFeatureEvidence { size_t left=0,right=0; double median=0,p95=0; bool stationary=false; };
inline StaticFeatureEvidence StationaryFeatures(const std::vector<StaticFeatureTrack>& tracks,size_t frames) {
  StaticFeatureEvidence evidence;
  if (frames<6) return evidence;
  std::vector<double> motion;
  for (const auto& track:tracks) {
    if (track.pixels.size()!=frames || (track.camera!=0 && track.camera!=1)) continue;
    double maximum=0; bool valid=true;
    for (const auto& p:track.pixels) {
      if (!std::isfinite(p[0]) || !std::isfinite(p[1])) { valid=false; break; }
      maximum=std::max(maximum,std::hypot(p[0]-track.pixels[0][0],p[1]-track.pixels[0][1]));
    }
    if (!valid) continue;
    if (track.camera==0) ++evidence.left; else ++evidence.right;
    motion.push_back(maximum);
  }
  if (evidence.left<30 || evidence.right<30) return evidence;
  std::sort(motion.begin(),motion.end());
  evidence.median=motion[motion.size()/2];
  evidence.p95=motion[size_t(.95*(motion.size()-1))];
  evidence.stationary=evidence.median<=.1 && evidence.p95<=.3;
  return evidence;
}
inline double norm(const Vec3& v) {
  return std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
}
inline bool finite(const Vec3& v) {
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}
inline bool EstimateStationary(const std::vector<StaticVisualSample>& vision,
                               const std::vector<StaticImuSample>& imu,
                               StaticEstimate& estimate, bool stereo_stationary=false) {
  if (vision.size()<6 || imu.size()<500) return false;
  if (vision.back().stamp-vision.front().stamp<5000000000LL ||
      imu.back().stamp-imu.front().stamp<4900000000LL) return false;
  if (imu.front().stamp<vision.front().stamp-20000000LL ||
      imu.back().stamp>vision.back().stamp+20000000LL ||
      imu.front().stamp>vision.front().stamp+20000000LL ||
      imu.back().stamp<vision.back().stamp-20000000LL) return false;
  Vec3 low=vision.front().position, high=low;
  for (size_t i=0; i<vision.size(); ++i) {
    const auto& v=vision[i];
    if (!finite(v.position) || !std::isfinite(v.angle) || std::abs(v.angle)>.002 ||
        (i && (v.stamp<=vision[i-1].stamp || v.stamp-vision[i-1].stamp>1500000000LL))) return false;
    for (int j=0;j<3;++j) { low[j]=std::fmin(low[j],v.position[j]); high[j]=std::fmax(high[j],v.position[j]); }
  }
  Vec3 span{};
  for (int j=0;j<3;++j) span[j]=high[j]-low[j];
  // Strict raw stereo stationarity can disambiguate drift in the pre-IMU
  // visual-only pose solve. Keep a gross metric consistency bound as well.
  if (norm(span)>(stereo_stationary ? .05 : .003)) return false;
  StaticEstimate candidate;
  for (size_t i=0;i<imu.size();++i) {
    const auto& m=imu[i];
    if (!finite(m.acceleration_world) || !finite(m.gyro_body) ||
        (i && (m.stamp<=imu[i-1].stamp || m.stamp-imu[i-1].stamp>20000000LL))) return false;
    for (int j=0;j<3;++j) {
      candidate.acceleration_world[j]+=m.acceleration_world[j]/imu.size();
      candidate.gyro_bias[j]+=m.gyro_body[j]/imu.size();
    }
  }
  if (std::abs(norm(candidate.acceleration_world)-9.81)>.15 || norm(candidate.gyro_bias)>.02) return false;
  double acc_variance=0,gyro_variance=0;
  for (const auto& m:imu) for (int j=0;j<3;++j) {
    acc_variance+=std::pow(m.acceleration_world[j]-candidate.acceleration_world[j],2)/imu.size();
    gyro_variance+=std::pow(m.gyro_body[j]-candidate.gyro_bias[j],2)/imu.size();
  }
  if (std::sqrt(acc_variance)>.03 || std::sqrt(gyro_variance)>.002) return false;
  estimate=candidate;
  return true;
}
}  // namespace uav_vio
