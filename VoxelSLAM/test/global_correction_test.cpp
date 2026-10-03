#include "global_correction.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <limits>
#include <string>

namespace {

CorrectionState makeState(double offset) {
  CorrectionState state;
  state.R = Eigen::AngleAxisd(0.2 + offset, Eigen::Vector3d::UnitX())
                .toRotationMatrix();
  state.p = Eigen::Vector3d(1.0 + offset, 2.0, 3.0);
  state.v = Eigen::Vector3d(4.0, 5.0 + offset, 6.0);
  state.bg = Eigen::Vector3d(0.1, 0.2, 0.3);
  state.ba = Eigen::Vector3d(0.4, 0.5, 0.6);
  state.g = Eigen::Vector3d(0.0, 0.0, -9.81);
  state.cov.setIdentity();
  state.cov(0, 3) = state.cov(3, 0) = 0.25;
  state.cov(3, 9) = state.cov(9, 3) = -0.1;
  state.cov(6, 12) = state.cov(12, 6) = 0.2;
  return state;
}

void expectStateNear(const CorrectionState &actual,
                     const CorrectionState &expected,
                     double tolerance = 1e-12) {
  EXPECT_TRUE(actual.R.isApprox(expected.R, tolerance));
  EXPECT_TRUE(actual.p.isApprox(expected.p, tolerance));
  EXPECT_TRUE(actual.v.isApprox(expected.v, tolerance));
  EXPECT_TRUE(actual.bg.isApprox(expected.bg, tolerance));
  EXPECT_TRUE(actual.ba.isApprox(expected.ba, tolerance));
  EXPECT_TRUE(actual.g.isApprox(expected.g, tolerance));
  EXPECT_TRUE(actual.cov.isApprox(expected.cov, tolerance));
}

TEST(GlobalCorrection, CorrectsAllWorldStateAtomicallyAndPreservesLocalPoints) {
  GlobalCorrectionTransform correction;
  correction.R =
      Eigen::AngleAxisd(1.5707963267948966, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  correction.t = Eigen::Vector3d(10.0, -2.0, 3.0);

  CorrectionTargets targets;
  targets.current_state = makeState(0.0);
  targets.window_states.push_back(makeState(1.0));
  targets.cached_states.push_back(makeState(2.0));
  targets.world_points.push_back(Eigen::Vector3d(1.0, 2.0, 3.0));
  targets.local_points.push_back(Eigen::Vector3d(4.0, 5.0, 6.0));
  const CorrectionTargets before = targets;
  std::string error;

  ASSERT_TRUE(
      apply_global_correction_once(correction, 7, targets, error)) << error;
  EXPECT_EQ(targets.last_applied_generation, 7U);
  EXPECT_TRUE(targets.has_applied_generation);

  Eigen::Matrix<double, 15, 15> adjoint =
      Eigen::Matrix<double, 15, 15>::Identity();
  adjoint.block<3, 3>(3, 3) = correction.R;
  adjoint.block<3, 3>(6, 6) = correction.R;
  auto expectedState = [&](const CorrectionState &input) {
    CorrectionState expected = input;
    expected.R = correction.R * input.R;
    expected.p = correction.R * input.p + correction.t;
    expected.v = correction.R * input.v;
    expected.g = correction.R * input.g;
    expected.cov = adjoint * input.cov * adjoint.transpose();
    return expected;
  };

  expectStateNear(targets.current_state, expectedState(before.current_state));
  expectStateNear(targets.window_states[0],
                  expectedState(before.window_states[0]));
  expectStateNear(targets.cached_states[0],
                  expectedState(before.cached_states[0]));
  EXPECT_TRUE(targets.world_points[0].isApprox(
      correction.R * before.world_points[0] + correction.t));
  EXPECT_TRUE(targets.local_points[0].isApprox(before.local_points[0]));
}

TEST(GlobalCorrection, RejectsDuplicateOrInvalidCorrectionWithoutMutation) {
  GlobalCorrectionTransform correction;
  correction.R = Eigen::Matrix3d::Identity();
  correction.t = Eigen::Vector3d(1.0, 2.0, 3.0);
  CorrectionTargets targets;
  targets.current_state = makeState(0.0);
  std::string error;

  ASSERT_TRUE(
      apply_global_correction_once(correction, 4, targets, error)) << error;
  const CorrectionTargets after_first = targets;
  EXPECT_FALSE(
      apply_global_correction_once(correction, 4, targets, error));
  EXPECT_NE(error.find("generation"), std::string::npos) << error;
  expectStateNear(targets.current_state, after_first.current_state);
  EXPECT_EQ(targets.last_applied_generation,
            after_first.last_applied_generation);

  GlobalCorrectionTransform invalid = correction;
  invalid.t.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(apply_global_correction_once(invalid, 5, targets, error));
  expectStateNear(targets.current_state, after_first.current_state);
  EXPECT_EQ(targets.last_applied_generation,
            after_first.last_applied_generation);
}

TEST(GlobalCorrection, CanLeaveGravityUnchangedForLegacyLoopUpdates) {
  GlobalCorrectionTransform correction;
  correction.R =
      Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitY()).toRotationMatrix();
  CorrectionTargets targets;
  targets.rotate_gravity = false;
  targets.current_state = makeState(0.0);
  const Eigen::Vector3d gravity = targets.current_state.g;
  std::string error;

  ASSERT_TRUE(
      apply_global_correction_once(correction, 1, targets, error)) << error;
  EXPECT_TRUE(targets.current_state.g.isApprox(gravity));
}

}  // namespace
