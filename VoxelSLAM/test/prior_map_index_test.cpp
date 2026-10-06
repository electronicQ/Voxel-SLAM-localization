#include "prior_map_tracker.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {

PointType makePoint(float seed) {
  PointType point;
  point.x = seed;
  point.y = seed + 1.0F;
  point.z = seed + 2.0F;
  point.intensity = seed + 3.0F;
  point.normal_x = 1.0F;
  point.normal_y = 0.0F;
  point.normal_z = 0.0F;
  point.curvature = seed + 4.0F;
  return point;
}

SavedKeyframe makeKeyframe(int id, const Eigen::Vector3d &translation) {
  SavedKeyframe keyframe;
  keyframe.id = id;
  keyframe.pose.stamp = 100.0 + id;
  keyframe.pose.R = Eigen::Matrix3d::Identity();
  keyframe.pose.t = translation;
  keyframe.cloud.reset(new pcl::PointCloud<PointType>());
  keyframe.cloud->push_back(makePoint(static_cast<float>(id)));
  return keyframe;
}

SavedMap makeShuffledMap() {
  SavedMap map;
  map.keyframes.push_back(
      makeKeyframe(99, Eigen::Vector3d(1.0, 0.0, 5.0)));
  map.keyframes.push_back(
      makeKeyframe(42, Eigen::Vector3d(0.0, 0.0, 100.0)));
  map.keyframes.push_back(
      makeKeyframe(7, Eigen::Vector3d(3.0, 4.0, -20.0)));
  return map;
}

std::string fingerprint(const SavedMap &map) {
  std::ostringstream stream;
  stream.precision(17);
  stream << map.keyframes.size() << ':' << map.btc_frames.size();
  for (const SavedKeyframe &keyframe : map.keyframes) {
    stream << '|' << keyframe.id << ',' << keyframe.pose.stamp;
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        stream << ',' << keyframe.pose.R(row, column);
      }
      stream << ',' << keyframe.pose.t(row);
    }
    stream << ',' << keyframe.cloud.get() << ',' << keyframe.cloud->size();
    for (const PointType &point : keyframe.cloud->points) {
      stream << ',' << point.x << ',' << point.y << ',' << point.z << ','
             << point.intensity << ',' << point.normal_x << ','
             << point.normal_y << ',' << point.normal_z << ','
             << point.curvature;
    }
  }
  return stream.str();
}

TEST(PriorMapTracker, UsesExplicitIdsAndIgnoresHeightForXyQueries) {
  SavedMap map = makeShuffledMap();
  const std::string before = fingerprint(map);
  PriorMapTracker tracker;
  std::string error;

  ASSERT_TRUE(tracker.initialize(map, error)) << error;
  EXPECT_EQ(tracker.source_map(), &map);

  const std::vector<int> ids =
      tracker.radius_search_xy(Eigen::Vector2d(0.0, 0.0), 1.1);
  ASSERT_EQ(ids.size(), 2U);
  EXPECT_EQ(ids[0], 42);
  EXPECT_EQ(ids[1], 99);

  double height = 0.0;
  int id = -1;
  ASSERT_TRUE(tracker.nearest_height_xy(Eigen::Vector2d(2.9, 4.1), height,
                                        id, error)) << error;
  EXPECT_EQ(id, 7);
  EXPECT_DOUBLE_EQ(height, -20.0);

  EXPECT_EQ(tracker.radius_search_xy(Eigen::Vector2d::Zero(), 1.1), ids);
  EXPECT_EQ(fingerprint(map), before);
}

TEST(PriorMapTracker, ReturnsDeterministicDistanceThenIdOrder) {
  SavedMap map;
  map.keyframes.push_back(makeKeyframe(20, Eigen::Vector3d(1.0, 0.0, 0.0)));
  map.keyframes.push_back(makeKeyframe(10, Eigen::Vector3d(-1.0, 0.0, 0.0)));
  map.keyframes.push_back(makeKeyframe(30, Eigen::Vector3d(0.0, 0.0, 0.0)));
  PriorMapTracker tracker;
  std::string error;
  ASSERT_TRUE(tracker.initialize(map, error)) << error;

  const std::vector<int> ids =
      tracker.radius_search_xy(Eigen::Vector2d::Zero(), 2.0);
  ASSERT_EQ(ids.size(), 3U);
  EXPECT_EQ(ids[0], 30);
  EXPECT_EQ(ids[1], 10);
  EXPECT_EQ(ids[2], 20);
}

TEST(PriorMapTracker, FailedReinitializationKeepsPreviousIndex) {
  SavedMap valid = makeShuffledMap();
  PriorMapTracker tracker;
  std::string error;
  ASSERT_TRUE(tracker.initialize(valid, error)) << error;

  SavedMap invalid = valid;
  invalid.keyframes[1].id = invalid.keyframes[0].id;
  EXPECT_FALSE(tracker.initialize(invalid, error));
  EXPECT_NE(error.find("duplicate"), std::string::npos) << error;
  EXPECT_EQ(tracker.source_map(), &valid);

  const std::vector<int> ids =
      tracker.radius_search_xy(Eigen::Vector2d::Zero(), 1.1);
  ASSERT_EQ(ids.size(), 2U);
  EXPECT_EQ(ids[0], 42);
  EXPECT_EQ(ids[1], 99);
}

TEST(PriorMapTracker, RejectsMalformedMapsAndQueries) {
  PriorMapTracker tracker;
  std::string error;
  SavedMap empty;
  EXPECT_FALSE(tracker.initialize(empty, error));
  EXPECT_NE(error.find("empty"), std::string::npos) << error;

  SavedMap null_cloud = makeShuffledMap();
  null_cloud.keyframes[0].cloud.reset();
  EXPECT_FALSE(tracker.initialize(null_cloud, error));
  EXPECT_NE(error.find("cloud"), std::string::npos) << error;

  SavedMap non_finite = makeShuffledMap();
  non_finite.keyframes[0].pose.t.x() =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(tracker.initialize(non_finite, error));
  EXPECT_NE(error.find("finite"), std::string::npos) << error;

  EXPECT_TRUE(tracker.radius_search_xy(Eigen::Vector2d::Zero(), -1.0).empty());
  double height = 0.0;
  int id = -1;
  EXPECT_FALSE(tracker.nearest_height_xy(
      Eigen::Vector2d(std::numeric_limits<double>::infinity(), 0.0), height,
      id, error));
  EXPECT_NE(error.find("finite"), std::string::npos) << error;
}

}  // namespace
