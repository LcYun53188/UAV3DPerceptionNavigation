// SPDX-License-Identifier: GPL-3.0-only
#include <gtest/gtest.h>
#include <bspline_opt/uniform_bspline.h>
#include "../src/safe_seed.hpp"

using V = Eigen::Vector3d;
using ego_planner::UniformBspline;

double limitedInterval(const Eigen::MatrixXd& controls) {
  UniformBspline derivative(controls, 3, 0.6);
  const double limits[] = {0.5, 1.0, 2.0};
  double scale = 1.0;
  for (int d = 1; d <= 3; ++d) {
    derivative = derivative.getDerivative();
    const double bound = derivative.getControlPoint().colwise().norm().maxCoeff();
    scale = std::max(scale, std::pow(bound/limits[d-1], 1.0/d));
  }
  return 0.6*scale*1.01;
}

TEST(SafeSeed, FasterThanRepeatedStraightStopsUnderSameDynamicLimits) {
  const double length = 4.3;
  const auto controls = safeSeedControls({V::Zero(), V(length, 0, 0)});
  Eigen::MatrixXd old_controls(3, 18);
  for (int i = 0; i <= 5; ++i)
    for (int k = 0; k < 3; ++k) old_controls.col(i*3+k) = V(length*i/5, 0, 0);
  UniformBspline curve(controls, 3, limitedInterval(controls));
  UniformBspline old_curve(old_controls, 3, limitedInterval(old_controls));
  EXPECT_LT(curve.getTimeSum(), old_curve.getTimeSum());
  auto derivative = curve;
  for (double limit : {0.5, 1.0, 2.0}) {
    derivative = derivative.getDerivative();
    EXPECT_LE(derivative.getControlPoint().colwise().norm().maxCoeff(), limit);
  }
  RecordProperty("old_duration_seconds", std::to_string(old_curve.getTimeSum()));
  RecordProperty("new_duration_seconds", std::to_string(curve.getTimeSum()));
}

TEST(SafeSeed, StraightTravelNeverStopsInsideAndEndsAtRest) {
  for (double length : {0.2, 1.0, 4.3}) {
    const auto controls = safeSeedControls({V::Zero(), V(length, 0, 0)});
    ASSERT_GE(controls.cols(), 7); // Python executor's representation contract.
    UniformBspline curve(controls, 3, 1.0);
    auto velocity = curve.getDerivative();
    auto acceleration = velocity.getDerivative();
    const double end = curve.getTimeSum();
    for (double t : {0.0, end}) {
      EXPECT_LT(velocity.evaluateDeBoorT(t).norm(), 1e-10);
      EXPECT_LT(acceleration.evaluateDeBoorT(t).norm(), 1e-10);
    }
    EXPECT_LT(curve.evaluateDeBoorT(0).norm(), 1e-10);
    EXPECT_LT((curve.evaluateDeBoorT(end)-V(length, 0, 0)).norm(), 1e-10);
    double previous = -1;
    for (int i = 1; i < 1000; ++i) {
      const double t = end*i/1000;
      const V p = curve.evaluateDeBoorT(t);
      EXPECT_GT(velocity.evaluateDeBoorT(t).x(), 0);
      EXPECT_GT(p.x(), previous);
      EXPECT_LE(p.x(), length);
      EXPECT_NEAR(p.y(), 0, 1e-10);
      EXPECT_NEAR(p.z(), 0, 1e-10);
      previous = p.x();
    }
  }
}

TEST(SafeSeed, RightAngleStaysOnCheckedEdgesAndStopsAtCorner) {
  const V corner(2, 0, 0), goal(2, 2, 0);
  const auto controls = safeSeedControls({V::Zero(), corner, goal});
  UniformBspline curve(controls, 3, 1.0);
  auto velocity = curve.getDerivative();
  auto acceleration = velocity.getDerivative();
  bool found_corner = false;
  for (double t = 0; t <= curve.getTimeSum(); t += 0.001) {
    const V p = curve.evaluateDeBoorT(t);
    EXPECT_TRUE(std::abs(p.y()) < 1e-9 || std::abs(p.x()-2) < 1e-9);
    EXPECT_GE(p.x(), -1e-9);
    EXPECT_LE(p.x(), 2+1e-9);
    EXPECT_GE(p.y(), -1e-9);
    EXPECT_LE(p.y(), 2+1e-9);
    if ((p-corner).norm() < 1e-12 && velocity.evaluateDeBoorT(t).norm() < 1e-9) {
      EXPECT_LT(acceleration.evaluateDeBoorT(t).norm(), 1e-8);
      found_corner = true;
    }
  }
  EXPECT_TRUE(found_corner);
}

TEST(SafeSeed, SmoothCornerKeepsMovingAndRequiresSweptCollisionCheck) {
  const auto controls=safeSeedControls({V::Zero(), V(2,0,0), V(2,2,0)},true);
  UniformBspline curve(controls,3,limitedInterval(controls));
  auto velocity=curve.getDerivative();
  bool cuts_corner=false;
  for(double t=.1;t<curve.getTimeSum()-.1;t+=.01) {
    const V p=curve.evaluateDeBoorT(t);
    EXPECT_GT(velocity.evaluateDeBoorT(t).norm(),1e-6);
    if(p.x()<1.99 && p.y()>.01) cuts_corner=true;
  }
  EXPECT_TRUE(cuts_corner); // Planner must validate this rounded region.
  EXPECT_LT(velocity.evaluateDeBoorT(curve.getTimeSum()).norm(),1e-10);
}

TEST(SafeSeed, MovingEndpointPreservesPositionVelocityAccelerationAfterRetiming) {
  const V p(1,2,3), v(.23,-.07,.03), a(-.04,.03,.01);
  for(double dt : {.3,.6,1.7,3.}) {
    auto controls=safeSeedControls({p,p+V(3,0,0)},true);
    setStartState(controls,dt,p,v,a);
    UniformBspline curve(controls,3,dt);
    EXPECT_LT((curve.evaluateDeBoorT(0)-p).norm(),1e-10);
    EXPECT_LT((curve.getDerivative().evaluateDeBoorT(0)-v).norm(),1e-10);
    EXPECT_LT((curve.getDerivative().getDerivative().evaluateDeBoorT(0)-a).norm(),1e-10);
    EXPECT_LT(curve.getDerivative().evaluateDeBoorT(curve.getTimeSum()).norm(),1e-10);
  }
}
