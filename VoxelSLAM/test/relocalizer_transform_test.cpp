#include "relocalizer.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <set>
#include <string>
#include <unordered_set>
#include <vector>

namespace {

SavedKeyframe keyframe(int id, double x, double y, double z) {
  SavedKeyframe result;
  result.id = id;
  result.pose.t = Eigen::Vector3d(x, y, z);
  result.cloud.reset(new pcl::PointCloud<PointType>());
  PointType point;
  point.x = 0.0F;
  point.y = 0.0F;
  point.z = 0.0F;
  point.intensity = 1.0F;
  point.normal_x = 0.0F;
  point.normal_y = 0.0F;
  point.normal_z = 1.0F;
  point.curvature = 0.0F;
  result.cloud->push_back(point);
  return result;
}

SavedBtcFrame btcFrame(int frame_id, int anchor_id) {
  SavedBtcFrame result;
  result.frame_id = frame_id;
  result.anchor_keyframe_id = anchor_id;
  result.plane_cloud.reset(
      new pcl::PointCloud<pcl::PointXYZINormal>());
  return result;
}

SavedMap makeMap() {
  SavedMap map;
  map.keyframes.push_back(keyframe(10, 0.0, 0.0, 1.0));
  map.keyframes.push_back(keyframe(20, 50.0, 0.0, 2.0));
  map.keyframes.push_back(keyframe(30, 150.0, 0.0, 3.0));
  map.btc_frames.push_back(btcFrame(2, 10));
  map.btc_frames.push_back(btcFrame(5, 20));
  map.btc_frames.push_back(btcFrame(9, 30));
  return map;
}

std::set<int> ordered(const std::unordered_set<int> &values) {
  return std::set<int>(values.begin(), values.end());
}

BtcRelocalizationOptions localOptions() {
  BtcRelocalizationOptions options;
  options.use_prior_position = true;
  options.prior_xy = Eigen::Vector2d::Zero();
  options.initial_radius = 30.0;
  options.expand_on_failure = true;
  options.maximum_radius = 100.0;
  options.fallback_to_global = true;
  return options;
}

TEST(BtcRelocalizer, ExpandsThenFallsBackGloballyWithExplicitMappings) {
  SavedMap map = makeMap();
  PriorMapTracker tracker;
  std::string error;
  ASSERT_TRUE(tracker.initialize(map, error)) << error;
  BtcRelocalizer relocalizer;
  ASSERT_TRUE(relocalizer.initialize(map, tracker, error)) << error;

  std::vector<std::set<int>> attempted_sets;
  const Eigen::Matrix3d expected_rotation =
      Eigen::AngleAxisd(1.5707963267948966, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  const Eigen::Vector3d expected_translation(4.0, -3.0, 2.0);
  BtcSearchFunction search =
      [&](const std::unordered_set<int> &allowed, BtcSearchMatch &match,
          std::string &reason) {
        attempted_sets.push_back(ordered(allowed));
        if (allowed.count(9) == 0U) {
          reason = "synthetic BTC miss";
          return false;
        }
        match.frame_id = 9;
        match.score = 0.9;
        match.R_HC = expected_rotation;
        match.t_HC = expected_translation;
        return true;
      };

  IcpVerificationFunction verify =
      [&](int frame_id, Eigen::Matrix3d &R_HC, Eigen::Vector3d &t_HC,
          double &observability, std::string &) {
        EXPECT_EQ(frame_id, 9);
        const std::vector<Eigen::Vector3d> current{
            Eigen::Vector3d(1.0, 2.0, 3.0),
            Eigen::Vector3d(-2.0, 0.5, 4.0),
            Eigen::Vector3d(0.25, -1.5, 2.0)};
        for (const Eigen::Vector3d &point : current) {
          const Eigen::Vector3d history =
              expected_rotation * point + expected_translation;
          EXPECT_TRUE((R_HC * point + t_HC).isApprox(history, 1e-12));
        }
        observability = 20.0;
        return true;
      };

  const RelocalizationResult result =
      relocalizer.relocalize(localOptions(), search, verify);
  ASSERT_TRUE(result.success) << result.failure_reason;
  EXPECT_EQ(result.btc_frame_id, 9);
  EXPECT_EQ(result.anchor_keyframe_id, 30);
  EXPECT_TRUE(result.R_HC.isApprox(expected_rotation, 1e-12));
  EXPECT_TRUE(result.t_HC.isApprox(expected_translation, 1e-12));
  EXPECT_TRUE(result.icp_observable);
  EXPECT_DOUBLE_EQ(result.icp_observability, 20.0);
  EXPECT_TRUE(result.global_fallback_tried);
  ASSERT_EQ(result.attempted_radii.size(), 3U);
  EXPECT_DOUBLE_EQ(result.attempted_radii[0], 30.0);
  EXPECT_DOUBLE_EQ(result.attempted_radii[1], 60.0);
  EXPECT_DOUBLE_EQ(result.attempted_radii[2], 100.0);

  ASSERT_EQ(attempted_sets.size(), 4U);
  EXPECT_EQ(attempted_sets[0], std::set<int>({2}));
  EXPECT_EQ(attempted_sets[1], std::set<int>({2, 5}));
  EXPECT_EQ(attempted_sets[2], std::set<int>({2, 5}));
  EXPECT_EQ(attempted_sets[3], std::set<int>({2, 5, 9}));
}

TEST(BtcRelocalizer, StopsAfterMaximumWhenGlobalFallbackIsDisabled) {
  SavedMap map = makeMap();
  PriorMapTracker tracker;
  std::string error;
  ASSERT_TRUE(tracker.initialize(map, error)) << error;
  BtcRelocalizer relocalizer;
  ASSERT_TRUE(relocalizer.initialize(map, tracker, error)) << error;
  BtcRelocalizationOptions options = localOptions();
  options.fallback_to_global = false;
  int search_count = 0;
  BtcSearchFunction search =
      [&](const std::unordered_set<int> &, BtcSearchMatch &,
          std::string &reason) {
        ++search_count;
        reason = "no match";
        return false;
      };
  IcpVerificationFunction unused_verify;

  const RelocalizationResult result =
      relocalizer.relocalize(options, search, unused_verify);
  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.global_fallback_tried);
  EXPECT_EQ(search_count, 3);
  EXPECT_NE(result.failure_reason.find("no match"), std::string::npos);
}

TEST(BtcRelocalizer, RejectsDegenerateGeometryAfterBtcMatch) {
  SavedMap map = makeMap();
  PriorMapTracker tracker;
  std::string error;
  ASSERT_TRUE(tracker.initialize(map, error)) << error;
  BtcRelocalizer relocalizer;
  ASSERT_TRUE(relocalizer.initialize(map, tracker, error)) << error;
  BtcRelocalizationOptions options = localOptions();
  options.use_prior_position = false;

  BtcSearchFunction search =
      [](const std::unordered_set<int> &, BtcSearchMatch &match,
         std::string &) {
        match.frame_id = 2;
        match.score = 0.8;
        match.R_HC = Eigen::Matrix3d::Identity();
        match.t_HC = Eigen::Vector3d::Zero();
        return true;
      };
  IcpVerificationFunction degenerate =
      [](int, Eigen::Matrix3d &, Eigen::Vector3d &, double &observability,
         std::string &reason) {
        observability = 0.0;
        reason = "degenerate plane geometry";
        return false;
      };

  const RelocalizationResult result =
      relocalizer.relocalize(options, search, degenerate);
  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.global_fallback_tried);
  EXPECT_FALSE(result.icp_observable);
  EXPECT_NE(result.failure_reason.find("degenerate"), std::string::npos);
}

TEST(BtcRelocalizer, RejectsInvalidAnchorMappingsAtomically) {
  SavedMap map = makeMap();
  PriorMapTracker tracker;
  std::string error;
  ASSERT_TRUE(tracker.initialize(map, error)) << error;
  BtcRelocalizer relocalizer;
  ASSERT_TRUE(relocalizer.initialize(map, tracker, error)) << error;

  SavedMap invalid = map;
  invalid.btc_frames[1].anchor_keyframe_id = 10;
  EXPECT_FALSE(relocalizer.initialize(invalid, tracker, error));
  EXPECT_NE(error.find("duplicate"), std::string::npos) << error;

  int calls = 0;
  BtcSearchFunction search =
      [&](const std::unordered_set<int> &, BtcSearchMatch &,
          std::string &reason) {
        ++calls;
        reason = "still using prior valid mapping";
        return false;
      };
  BtcRelocalizationOptions options = localOptions();
  options.expand_on_failure = false;
  options.fallback_to_global = false;
  EXPECT_FALSE(relocalizer.relocalize(options, search,
                                      IcpVerificationFunction()).success);
  EXPECT_EQ(calls, 1);
}

}  // namespace
