#include "prior_map_tracker.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <sstream>
#include <string>
#include <vector>

namespace {

SavedKeyframe keyframe(int id, double x, double yaw = 0.0) {
  SavedKeyframe result;
  result.id = id;
  result.pose.stamp = 100.0 + id;
  result.pose.R =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  result.pose.t = Eigen::Vector3d(x, 0.0, 1.0);
  result.cloud.reset(new pcl::PointCloud<PointType>());
  PointType point;
  point.x = 1.0F;
  point.y = 0.0F;
  point.z = 0.0F;
  point.intensity = static_cast<float>(id);
  point.normal_x = 1.0F;
  point.normal_y = 4.0F;
  point.normal_z = 9.0F;
  point.curvature = static_cast<float>(id + 1);
  result.cloud->push_back(point);
  return result;
}

SavedMap separatedMap() {
  SavedMap map;
  map.keyframes.push_back(keyframe(10, 0.0, 1.5707963267948966));
  map.keyframes.push_back(keyframe(11, 2.0));
  map.keyframes.push_back(keyframe(20, 20.0));
  map.keyframes.push_back(keyframe(21, 22.0));
  map.keyframes.push_back(keyframe(30, 40.0));
  map.keyframes.push_back(keyframe(31, 42.0));
  return map;
}

std::string fingerprint(const SavedMap &map) {
  std::ostringstream stream;
  stream.precision(17);
  for (const SavedKeyframe &keyframe : map.keyframes) {
    stream << keyframe.id << ':' << keyframe.pose.stamp << ':';
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        stream << keyframe.pose.R(row, column) << ',';
      }
      stream << keyframe.pose.t(row) << ',';
    }
    stream << keyframe.cloud.get() << ':' << keyframe.cloud->size();
    for (const PointType &point : keyframe.cloud->points) {
      stream << ':' << point.x << ',' << point.y << ',' << point.z << ','
             << point.intensity << ',' << point.normal_x << ','
             << point.normal_y << ',' << point.normal_z << ','
             << point.curvature;
    }
  }
  return stream.str();
}

TEST(PriorMapLoading, LoadsAtMostOnePerScanWithoutDuplicatesAndUnloads) {
  SavedMap map = separatedMap();
  const std::string before = fingerprint(map);
  PriorMapTracker tracker;
  PriorMapCacheUpdate update;
  std::string error;
  ASSERT_TRUE(tracker.initialize(map, error)) << error;

  ASSERT_TRUE(tracker.update_loaded_keyframes(
      Eigen::Vector3d(0.0, 0.0, 1.0), 3.0, 8.0, 1, update, error))
      << error;
  EXPECT_EQ(update.loaded_ids, std::vector<int>({10}));
  EXPECT_TRUE(update.unloaded_ids.empty());
  EXPECT_EQ(update.active_ids, std::vector<int>({10}));
  ASSERT_EQ(update.world_cloud->size(), 1U);
  EXPECT_NEAR(update.world_cloud->points[0].x, 0.0, 1e-6);
  EXPECT_NEAR(update.world_cloud->points[0].y, 1.0, 1e-6);
  EXPECT_NEAR(update.world_cloud->points[0].z, 1.0, 1e-6);
  EXPECT_NEAR(update.world_cloud->points[0].normal_x, 4.0, 1e-6);
  EXPECT_NEAR(update.world_cloud->points[0].normal_y, 1.0, 1e-6);

  ASSERT_TRUE(tracker.update_loaded_keyframes(
      Eigen::Vector3d(0.0, 0.0, 1.0), 3.0, 8.0, 1, update, error))
      << error;
  EXPECT_EQ(update.loaded_ids, std::vector<int>({11}));
  EXPECT_EQ(update.active_ids, std::vector<int>({10, 11}));

  ASSERT_TRUE(tracker.update_loaded_keyframes(
      Eigen::Vector3d(0.0, 0.0, 1.0), 3.0, 8.0, 1, update, error))
      << error;
  EXPECT_TRUE(update.loaded_ids.empty());
  EXPECT_TRUE(update.unloaded_ids.empty());
  EXPECT_EQ(update.active_ids, std::vector<int>({10, 11}));

  ASSERT_TRUE(tracker.update_loaded_keyframes(
      Eigen::Vector3d(20.0, 0.0, 1.0), 3.0, 8.0, 1, update, error))
      << error;
  EXPECT_EQ(update.loaded_ids, std::vector<int>({20}));
  EXPECT_EQ(update.unloaded_ids, std::vector<int>({10, 11}));
  EXPECT_EQ(update.active_ids, std::vector<int>({20}));
  EXPECT_EQ(fingerprint(map), before);
}

TEST(PriorMapLoading, RebuildsAroundANewCenterAndRejectsBadRequestsAtomically) {
  SavedMap map = separatedMap();
  PriorMapTracker tracker;
  PriorMapCacheUpdate update;
  std::string error;
  ASSERT_TRUE(tracker.initialize(map, error)) << error;
  ASSERT_TRUE(tracker.rebuild_local_map(Eigen::Vector3d(40.0, 0.0, 1.0),
                                        3.0, 2, update, error))
      << error;
  EXPECT_EQ(update.loaded_ids, std::vector<int>({30, 31}));
  EXPECT_EQ(update.active_ids, std::vector<int>({30, 31}));
  ASSERT_EQ(update.world_cloud->size(), 2U);

  const std::vector<int> active_before = update.active_ids;
  const pcl::PointCloud<PointType>::ConstPtr cloud_before =
      tracker.loaded_world_cloud();
  EXPECT_FALSE(tracker.update_loaded_keyframes(
      Eigen::Vector3d::Zero(), 9.0, 8.0, 1, update, error));
  EXPECT_NE(error.find("radius"), std::string::npos) << error;
  EXPECT_EQ(tracker.loaded_keyframe_ids(), active_before);
  EXPECT_EQ(tracker.loaded_world_cloud(), cloud_before);

  tracker.reset_runtime_cache();
  EXPECT_TRUE(tracker.loaded_keyframe_ids().empty());
  EXPECT_TRUE(tracker.loaded_world_cloud()->empty());
}

}  // namespace
