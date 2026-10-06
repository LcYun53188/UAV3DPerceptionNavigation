// SPDX-License-Identifier: GPL-3.0-only
#include <gtest/gtest.h>
#include "../src/swept_segment.hpp"

TEST(SweptSegment, RefinesUnknownBoundaryWithoutReducingRadius) {
  auto collision=[](const Eigen::Vector3d& p,double r){return p.z()+r>=.4;};
  const Eigen::Vector3d start(0,0,.045), end(.3,0,.045);
  EXPECT_TRUE(collision(start,.375));
  EXPECT_FALSE(collision(start,.35));
  EXPECT_TRUE(sweptSegmentFree(start,end,.35,.1,collision));
  EXPECT_FALSE(sweptSegmentFree(start,end+Eigen::Vector3d(0,0,.1),.35,.1,collision));
}

TEST(SweptSegment, RejectsObstacleBetweenFreeEndpoints) {
  auto collision=[](const Eigen::Vector3d& p,double r){return std::abs(p.x())<=r+.005;};
  EXPECT_FALSE(sweptSegmentFree(Eigen::Vector3d(-.8,0,0),Eigen::Vector3d(.8,0,0),.3,.1,collision));
}

TEST(SweptSegment, RejectsUncertifiableIntervalsWithinBudget) {
  int checks=0;
  auto collision=[&checks](const Eigen::Vector3d&,double r){++checks;return r>.35;};
  EXPECT_FALSE(sweptSegmentFree(Eigen::Vector3d::Zero(),Eigen::Vector3d(.3,0,0),.35,.1,collision));
  EXPECT_LE(checks,4096);
}
