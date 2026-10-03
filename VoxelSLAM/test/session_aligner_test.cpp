#include "session_aligner.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <limits>
#include <string>
#include <vector>

namespace {

SavedPose pose(double x, double y, double z, double yaw, double stamp) {
  SavedPose result;
  result.stamp = stamp;
  result.R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                 .toRotationMatrix();
  result.t = Eigen::Vector3d(x, y, z);
  return result;
}

SavedPose compose(const SavedPose &left, const SavedPose &right) {
  SavedPose result;
  result.stamp = right.stamp;
  result.R = left.R * right.R;
  result.t = left.R * right.t + left.t;
  return result;
}

SavedPose inverse(const SavedPose &input) {
  SavedPose result;
  result.R = input.R.transpose();
  result.t = -result.R * input.t;
  return result;
}

void expectPoseNear(const SavedPose &actual, const SavedPose &expected,
                    double tolerance = 1e-7) {
  EXPECT_TRUE(actual.R.isApprox(expected.R, tolerance));
  EXPECT_TRUE(actual.t.isApprox(expected.t, tolerance));
}

std::vector<SavedPose> driftingSession() {
  std::vector<SavedPose> poses;
  for (int i = 0; i < 5; ++i) {
    poses.push_back(pose(1.1 * i, 0.05 * i, 0.0, 0.01 * i,
                         100.0 + i));
  }
  return poses;
}

TEST(SessionAligner, FixesHistoryAndPreservesCurrentOdometryStructure) {
  const SavedPose history = pose(10.0, 5.0, 1.0, 0.3, 10.0);
  const Eigen::Matrix3d R_HC =
      Eigen::AngleAxisd(1.5707963267948966, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  const Eigen::Vector3d t_HC(2.0, -1.0, 0.5);
  const std::vector<SavedPose> current = driftingSession();
  SessionAlignmentOptions options;
  options.max_translation_correction_m = 100.0;
  options.max_rotation_correction_deg = 180.0;
  SessionAlignmentResult result;
  std::string error;

  ASSERT_TRUE(align_current_session_to_map(
      history, 2, R_HC, t_HC, current, options, result, error)) << error;

  expectPoseNear(result.fixed_history, history);
  ASSERT_EQ(result.optimized_current.size(), current.size());
  SavedPose H_C;
  H_C.R = R_HC;
  H_C.t = t_HC;
  H_C.stamp = current[2].stamp;
  const SavedPose expected_anchor = compose(history, H_C);
  expectPoseNear(result.optimized_current[2], expected_anchor);

  for (std::size_t i = 1; i < current.size(); ++i) {
    const SavedPose odom_relative = compose(inverse(current[i - 1]), current[i]);
    const SavedPose map_relative = compose(
        inverse(result.optimized_current[i - 1]), result.optimized_current[i]);
    expectPoseNear(map_relative, odom_relative, 1e-6);
  }
  for (std::size_t i = 0; i < current.size(); ++i) {
    expectPoseNear(result.optimized_current[i],
                   compose(result.T_W_O, current[i]), 1e-6);
    EXPECT_DOUBLE_EQ(result.optimized_current[i].stamp, current[i].stamp);
  }
}

TEST(SessionAligner, RejectsInvalidInputsAndUnsafeCorrectionAtomically) {
  const SavedPose history = pose(10.0, 5.0, 1.0, 0.3, 10.0);
  const std::vector<SavedPose> current = driftingSession();
  const Eigen::Matrix3d R_HC = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d t_HC(2.0, -1.0, 0.5);
  SessionAlignmentOptions options;
  SessionAlignmentResult sentinel;
  sentinel.fixed_history = pose(999.0, 0.0, 0.0, 0.0, 0.0);
  SessionAlignmentResult result = sentinel;
  std::string error;

  std::vector<SavedPose> non_finite = current;
  non_finite[1].t.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(align_current_session_to_map(
      history, 2, R_HC, t_HC, non_finite, options, result, error));
  EXPECT_TRUE(result.fixed_history.t.isApprox(sentinel.fixed_history.t));

  EXPECT_FALSE(align_current_session_to_map(
      history, 9, R_HC, t_HC, current, options, result, error));
  EXPECT_NE(error.find("anchor"), std::string::npos) << error;

  SessionAlignmentOptions unsafe = options;
  unsafe.max_translation_correction_m = 0.1;
  EXPECT_FALSE(align_current_session_to_map(
      history, 2, R_HC, t_HC, current, unsafe, result, error));
  EXPECT_NE(error.find("safety"), std::string::npos) << error;
  EXPECT_TRUE(result.fixed_history.t.isApprox(sentinel.fixed_history.t));
}

TEST(SessionAligner, PinsHistoryToCurrentTransformDirection) {
  const SavedPose history = pose(2.0, 3.0, 0.0, 0.2, 1.0);
  const std::vector<SavedPose> current = driftingSession();
  SavedPose H_C;
  H_C.R = Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ())
              .toRotationMatrix();
  H_C.t = Eigen::Vector3d(3.0, -2.0, 0.5);
  const SavedPose expected = compose(history, H_C);
  SessionAlignmentOptions options;
  options.max_translation_correction_m = 100.0;
  SessionAlignmentResult correct;
  SessionAlignmentResult inverted;
  std::string error;

  ASSERT_TRUE(align_current_session_to_map(
      history, 2, H_C.R, H_C.t, current, options, correct, error)) << error;
  const SavedPose C_H = inverse(H_C);
  ASSERT_TRUE(align_current_session_to_map(
      history, 2, C_H.R, C_H.t, current, options, inverted, error)) << error;

  expectPoseNear(correct.optimized_current[2], expected, 1e-6);
  EXPECT_FALSE(inverted.optimized_current[2].R.isApprox(expected.R, 1e-3));
  EXPECT_FALSE(inverted.optimized_current[2].t.isApprox(expected.t, 1e-3));
}

}  // namespace
