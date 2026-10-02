// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <Eigen/Core>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

// Constant-time conservative body-box queries. Observation alone does not
// imply free space: every covered voxel must have a finite positive distance.
class FreeVolume {
 public:
  void update(const std::array<uint32_t,3>& shape,
              const std::vector<float>& distance,
              const std::vector<uint8_t>& observed) {
    shape_=shape;
    prefix_.assign(size_t(shape[0]+1)*(shape[1]+1)*(shape[2]+1),0);
    for(uint32_t x=1;x<=shape[0];++x)
      for(uint32_t y=1;y<=shape[1];++y)
        for(uint32_t z=1;z<=shape[2];++z) {
          const size_t i=(size_t(x-1)*shape[1]+y-1)*shape[2]+z-1;
          const bool free=observed[i] && std::isfinite(distance[i]) && distance[i]>0;
          at(x,y,z)=free+sum(x-1,y,z)+sum(x,y-1,z)+sum(x,y,z-1)
            -sum(x-1,y-1,z)-sum(x-1,y,z-1)-sum(x,y-1,z-1)+sum(x-1,y-1,z-1);
        }
  }

  bool isFree(const Eigen::Vector3i& lo, const Eigen::Vector3i& hi) const {
    for(int a=0;a<3;++a)
      if(lo[a]<0 || hi[a]<lo[a] || hi[a]>=int(shape_[a])) return false;
    const auto h=(hi+Eigen::Vector3i::Ones()).eval();
    const int64_t count=sum(h.x(),h.y(),h.z())-sum(lo.x(),h.y(),h.z())
      -sum(h.x(),lo.y(),h.z())-sum(h.x(),h.y(),lo.z())
      +sum(lo.x(),lo.y(),h.z())+sum(lo.x(),h.y(),lo.z())
      +sum(h.x(),lo.y(),lo.z())-sum(lo.x(),lo.y(),lo.z());
    return count==int64_t((h-lo).prod());
  }

 private:
  uint32_t& at(int x,int y,int z) {
    return prefix_[(size_t(x)*(shape_[1]+1)+y)*(shape_[2]+1)+z];
  }
  int64_t sum(int x,int y,int z) const {
    return prefix_[(size_t(x)*(shape_[1]+1)+y)*(shape_[2]+1)+z];
  }
  std::array<uint32_t,3> shape_{};
  std::vector<uint32_t> prefix_;
};
