#include "spatial_map_builder.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <type_traits>

namespace {

SpatialMapPoint test_point(double x, double y, double z,
                           double covariance = 1e-4,
                           float intensity = 1.0F) {
  SpatialMapPoint point;
  point.position = Eigen::Vector3d(x, y, z);
  point.covariance = Eigen::Matrix3d::Identity() * covariance;
  point.intensity = intensity;
  return point;
}

SpatialMapPointVector horizontal_plane_points(double z = 0.05) {
  SpatialMapPointVector points;
  for (int x = 0; x < 4; ++x) {
    for (int y = 0; y < 4; ++y) {
      points.push_back(test_point(0.10 + 0.11 * x, 0.10 + 0.11 * y,
                                  z, 1e-5,
                                  static_cast<float>(4 * x + y)));
    }
  }
  return points;
}

bool points_equal(const SpatialMapPoint &left,
                  const SpatialMapPoint &right) {
  return (left.position.array() == right.position.array()).all() &&
         (left.covariance.array() == right.covariance.array()).all() &&
         left.intensity == right.intensity;
}

bool blocks_equal(const StaticMapBlock &left, const StaticMapBlock &right) {
  if (left.index != right.index || left.roots.size() != right.roots.size() ||
      left.nodes.size() != right.nodes.size() ||
      left.planes.size() != right.planes.size() ||
      left.display_points.size() != right.display_points.size()) {
    return false;
  }
  for (std::size_t i = 0; i < left.roots.size(); ++i) {
    if (left.roots[i].index != right.roots[i].index ||
        left.roots[i].root_node_index != right.roots[i].root_node_index) {
      return false;
    }
  }
  for (std::size_t i = 0; i < left.nodes.size(); ++i) {
    if (!(left.nodes[i].center.array() == right.nodes[i].center.array()).all() ||
        left.nodes[i].half_length != right.nodes[i].half_length ||
        left.nodes[i].children != right.nodes[i].children ||
        left.nodes[i].plane_index != right.nodes[i].plane_index) {
      return false;
    }
  }
  for (std::size_t i = 0; i < left.planes.size(); ++i) {
    if (!(left.planes[i].center.array() ==
          right.planes[i].center.array()).all() ||
        !(left.planes[i].normal.array() ==
          right.planes[i].normal.array()).all() ||
        !(left.planes[i].covariance.array() ==
          right.planes[i].covariance.array()).all() ||
        left.planes[i].radius != right.planes[i].radius) {
      return false;
    }
  }
  for (std::size_t i = 0; i < left.display_points.size(); ++i) {
    if (!points_equal(left.display_points[i], right.display_points[i])) {
      return false;
    }
  }
  return true;
}

TEST(SpatialMapBuilder, FusesRepeatedObservationsAcrossSources) {
  SpatialMapConfig config;
  SpatialMapPointVector points;
  points.push_back(test_point(1.01, 2.01, 0.01, 1e-4, 1.0F));
  points.push_back(test_point(1.04, 2.03, 0.04, 2e-4, 2.0F));
  points.push_back(test_point(1.08, 2.07, 0.08, 3e-4, 3.0F));

  SpatialMapBuildResult result;
  std::string error;
  ASSERT_TRUE(SpatialMapBuilder::build(points, config, result, error)) << error;
  EXPECT_EQ(result.source_point_count, 3U);
  EXPECT_EQ(result.fused_point_count, 1U);
}

TEST(SpatialMapBuilder, InputReorderingProducesIdenticalLogicalBlocks) {
  SpatialMapConfig config;
  config.save_display_cloud = true;
  SpatialMapPointVector points = horizontal_plane_points();
  points.push_back(test_point(0.101, 0.101, 0.051, 2e-5, 20.0F));
  points.push_back(test_point(0.102, 0.102, 0.052, 3e-5, 30.0F));

  SpatialMapBuildResult forward;
  SpatialMapBuildResult reversed;
  std::string error;
  ASSERT_TRUE(SpatialMapBuilder::build(points, config, forward, error)) << error;
  std::reverse(points.begin(), points.end());
  ASSERT_TRUE(SpatialMapBuilder::build(points, config, reversed, error)) << error;

  ASSERT_EQ(forward.source_point_count, reversed.source_point_count);
  ASSERT_EQ(forward.fused_point_count, reversed.fused_point_count);
  ASSERT_EQ(forward.blocks.size(), reversed.blocks.size());
  for (std::size_t i = 0; i < forward.blocks.size(); ++i) {
    EXPECT_TRUE(blocks_equal(forward.blocks[i], reversed.blocks[i]));
  }
  ASSERT_EQ(forward.blocks.size(), 1U);
  ASSERT_FALSE(forward.blocks[0].display_points.empty());
  const SpatialMapPoint &averaged = forward.blocks[0].display_points[0];
  EXPECT_NEAR(averaged.position.x(), (0.100 + 0.101 + 0.102) / 3.0,
              1e-15);
  EXPECT_NEAR(averaged.position.y(), (0.100 + 0.101 + 0.102) / 3.0,
              1e-15);
  EXPECT_NEAR(averaged.position.z(), (0.050 + 0.051 + 0.052) / 3.0,
              1e-15);
  EXPECT_NEAR(averaged.covariance(0, 0), 2e-5, 1e-15);
  EXPECT_FLOAT_EQ(averaged.intensity, 50.0F / 3.0F);
}

TEST(SpatialMapBuilder, AssignsNegativeCoordinatesThroughSharedIndices) {
  SpatialMapConfig config;
  SpatialMapPointVector points;
  for (int x = 0; x < 3; ++x) {
    for (int y = 0; y < 3; ++y) {
      points.push_back(test_point(-0.90 + 0.14 * x, -0.90 + 0.14 * y,
                                  -0.05));
    }
  }

  SpatialMapBuildResult result;
  std::string error;
  ASSERT_TRUE(SpatialMapBuilder::build(points, config, result, error)) << error;
  ASSERT_EQ(result.blocks.size(), 1U);
  EXPECT_EQ(result.blocks[0].index, BlockIndex(-1, -1, -1));
  ASSERT_EQ(result.blocks[0].roots.size(), 1U);
  EXPECT_EQ(result.blocks[0].roots[0].index, RootVoxelIndex(-1, -1, -1));
}

TEST(SpatialMapBuilder, EmptyAndNonFiniteInputsFailAtomically) {
  const SpatialMapConfig config;
  SpatialMapBuildResult output;
  output.source_point_count = 77U;
  output.fused_point_count = 33U;
  StaticMapBlock sentinel;
  sentinel.index = BlockIndex(9, 8, 7);
  output.blocks.push_back(sentinel);
  std::string error;

  const SpatialMapPointVector empty;
  EXPECT_FALSE(SpatialMapBuilder::build(empty, config, output, error));
  EXPECT_FALSE(error.empty());
  EXPECT_EQ(output.source_point_count, 77U);
  ASSERT_EQ(output.blocks.size(), 1U);
  EXPECT_EQ(output.blocks[0].index, BlockIndex(9, 8, 7));

  SpatialMapPointVector invalid{test_point(0.0, 0.0, 0.0)};
  invalid[0].position.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(SpatialMapBuilder::build(invalid, config, output, error));
  EXPECT_EQ(output.source_point_count, 77U);

  invalid[0] = test_point(0.0, 0.0, 0.0);
  invalid[0].covariance(0, 1) = 1.0;
  EXPECT_FALSE(SpatialMapBuilder::build(invalid, config, output, error));
  EXPECT_EQ(output.source_point_count, 77U);

  invalid[0] = test_point(0.0, 0.0, 0.0);
  invalid[0].covariance(1, 1) = -1.0;
  EXPECT_FALSE(SpatialMapBuilder::build(invalid, config, output, error));
  EXPECT_EQ(output.source_point_count, 77U);

  invalid[0] = test_point(0.0, 0.0, 0.0);
  invalid[0].intensity = std::numeric_limits<float>::infinity();
  EXPECT_FALSE(SpatialMapBuilder::build(invalid, config, output, error));
  EXPECT_EQ(output.source_point_count, 77U);
}

TEST(SpatialMapBuilder, StaticRepresentationUsesIndicesInsteadOfPointers) {
  static_assert(
      std::is_same<typename decltype(StaticOctreeNode{}.children)::value_type,
                   std::uint32_t>::value,
      "octree children must be serialized indices");
  static_assert(std::is_same<decltype(RootVoxelRecord{}.root_node_index),
                             std::uint32_t>::value,
                "root nodes must be serialized indices");
  SUCCEED();
}

}  // namespace
