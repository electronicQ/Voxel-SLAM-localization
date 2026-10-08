#include "localization_state.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <limits>
#include <string>

namespace {

Eigen::Matrix3d rpy(double roll, double pitch, double yaw) {
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

SavedKeyframe keyframe(int id, double x, double y, double z) {
  SavedKeyframe result;
  result.id = id;
  result.pose.R = Eigen::Matrix3d::Identity();
  result.pose.t = Eigen::Vector3d(x, y, z);
  result.cloud.reset(new pcl::PointCloud<PointType>());
  PointType point;
  point.x = point.y = point.z = 0.0F;
  point.intensity = 0.0F;
  point.normal_x = point.normal_y = point.normal_z = 1.0F;
  point.curvature = 0.0F;
  result.cloud->push_back(point);
  return result;
}

TEST(DirectPoseInitializer, UsesRvizXyYawImuRollPitchAndNearestMapHeight) {
  SavedMap map;
  map.keyframes.push_back(keyframe(7, 0.0, 0.0, 2.0));
  map.keyframes.push_back(keyframe(9, 10.0, 20.0, 6.5));
  PriorMapTracker tracker;
  std::string error;
  ASSERT_TRUE(tracker.initialize(map, error)) << error;

  PendingInitialPose request;
  request.valid = true;
  request.generation = 3;
  request.stamp_sec = 123.0;
  request.xy = Eigen::Vector2d(9.8, 20.2);
  request.yaw = 1.2;
  const Eigen::Matrix3d R_O_C = rpy(0.15, -0.25, 0.8);
  const Eigen::Vector3d p_O_C(4.0, -3.0, 1.0);
  DirectPoseSeed seed;

  ASSERT_TRUE(make_direct_pose_seed(request, true, R_O_C, p_O_C,
                                    tracker, seed, error)) << error;
  EXPECT_EQ(seed.request_generation, 3U);
  EXPECT_EQ(seed.height_keyframe_id, 9);
  EXPECT_TRUE(seed.T_W_C.t.isApprox(Eigen::Vector3d(9.8, 20.2, 6.5)));
  EXPECT_TRUE(seed.T_W_C.R.isApprox(rpy(0.15, -0.25, 1.2), 1e-12));

  const Eigen::Matrix3d recovered_R = seed.T_W_O.R * R_O_C;
  const Eigen::Vector3d recovered_t =
      seed.T_W_O.R * p_O_C + seed.T_W_O.t;
  EXPECT_TRUE(recovered_R.isApprox(seed.T_W_C.R, 1e-12));
  EXPECT_TRUE(recovered_t.isApprox(seed.T_W_C.t, 1e-12));
}

TEST(DirectPoseInitializer, RejectsMissingImuAndInvalidInputsAtomically) {
  SavedMap map;
  map.keyframes.push_back(keyframe(1, 0.0, 0.0, 3.0));
  PriorMapTracker tracker;
  std::string error;
  ASSERT_TRUE(tracker.initialize(map, error)) << error;
  PendingInitialPose request;
  request.valid = true;
  request.generation = 1;
  request.xy = Eigen::Vector2d::Zero();
  DirectPoseSeed sentinel;
  sentinel.height_keyframe_id = 99;
  DirectPoseSeed result = sentinel;

  EXPECT_FALSE(make_direct_pose_seed(
      request, false, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(),
      tracker, result, error));
  EXPECT_NE(error.find("IMU"), std::string::npos) << error;
  EXPECT_EQ(result.height_keyframe_id, 99);

  request.yaw = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(make_direct_pose_seed(
      request, true, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(),
      tracker, result, error));
  EXPECT_EQ(result.height_keyframe_id, 99);
}

TEST(DirectPoseInitializer, AcceptsMapFramesAndReplacesRequests) {
  InitialPoseRequestBuffer buffer;
  std::string error;
  std::uint64_t first = 0;
  std::uint64_t second = 0;
  std::uint64_t third = 0;
  std::uint64_t fourth = 0;

  EXPECT_FALSE(buffer.submit("odom", 1.0, Eigen::Vector2d(1.0, 2.0),
                             0.3, first, error));
  EXPECT_NE(error.find("camera_init"), std::string::npos) << error;
  EXPECT_NE(error.find("/camera_init"), std::string::npos) << error;
  EXPECT_NE(error.find("map"), std::string::npos) << error;
  EXPECT_NE(error.find("/map"), std::string::npos) << error;
  EXPECT_FALSE(buffer.snapshot().valid);

  ASSERT_TRUE(buffer.submit("camera_init", 2.0, Eigen::Vector2d(3.0, 4.0),
                            0.5, first, error)) << error;
  ASSERT_TRUE(buffer.submit("/camera_init", 3.0, Eigen::Vector2d(5.0, 6.0),
                            0.7, second, error)) << error;
  ASSERT_TRUE(buffer.submit("map", 4.0, Eigen::Vector2d(7.0, 8.0),
                            0.9, third, error)) << error;
  ASSERT_TRUE(buffer.submit("/map", 5.0, Eigen::Vector2d(9.0, 10.0),
                            1.1, fourth, error)) << error;
  EXPECT_GT(second, first);
  EXPECT_GT(third, second);
  EXPECT_GT(fourth, third);
  EXPECT_FALSE(buffer.is_current(first));
  EXPECT_TRUE(buffer.is_current(fourth));
  bool stale_action_ran = false;
  EXPECT_FALSE(buffer.run_if_current(first,
                                     [&]() { stale_action_ran = true; }));
  EXPECT_FALSE(stale_action_ran);
  bool current_action_ran = false;
  EXPECT_TRUE(buffer.run_if_current(fourth,
                                    [&]() { current_action_ran = true; }));
  EXPECT_TRUE(current_action_ran);
  const PendingInitialPose pending = buffer.snapshot();
  EXPECT_TRUE(pending.valid);
  EXPECT_EQ(pending.generation, fourth);
  EXPECT_TRUE(pending.xy.isApprox(Eigen::Vector2d(9.0, 10.0)));
  EXPECT_DOUBLE_EQ(pending.yaw, 1.1);
}

}  // namespace
