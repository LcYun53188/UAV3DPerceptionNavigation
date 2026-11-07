#pragma once
// Prefer the exact photometric zero-displacement solution only near the
// original prediction. This changes a solver seed, never its quality checks.
#ifdef __CUDACC__
#define UAV_FLOW_HD __host__ __device__
#else
#define UAV_FLOW_HD
#endif
namespace uav_vio {
UAV_FLOW_HD inline bool PreferZeroFlowSeed(float absolute_patch_error, float x, float y) {
  return absolute_patch_error==0.f && x>=-1.f && x<=1.f && y>=-1.f && y<=1.f;
}
}
#undef UAV_FLOW_HD
