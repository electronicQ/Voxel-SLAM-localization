#include "spatial_map_builder.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <initializer_list>
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

bool configs_equal(const SpatialMapConfig &left,
                   const SpatialMapConfig &right) {
  return left.size_x == right.size_x && left.size_y == right.size_y &&
         left.size_z == right.size_z &&
         left.fusion_leaf_size == right.fusion_leaf_size &&
         left.root_voxel_size == right.root_voxel_size &&
         left.max_layer == right.max_layer &&
         left.min_eigen_value == right.min_eigen_value &&
         left.min_points == right.min_points &&
         left.plane_ratio_threshold == right.plane_ratio_threshold &&
         left.save_display_cloud == right.save_display_cloud;
}

bool results_equal(const SpatialMapBuildResult &left,
                   const SpatialMapBuildResult &right) {
  if (!configs_equal(left.config, right.config) ||
      left.source_point_count != right.source_point_count ||
      left.fused_point_count != right.fused_point_count ||
      left.blocks.size() != right.blocks.size()) {
    return false;
  }
  for (std::size_t i = 0; i < left.blocks.size(); ++i) {
    if (!blocks_equal(left.blocks[i], right.blocks[i])) {
      return false;
    }
  }
  return true;
}

SpatialMapBuildResult full_sentinel_result() {
  SpatialMapBuildResult result;
  result.config.size_x = 40.0;
  result.config.size_y = 60.0;
  result.config.size_z = 30.0;
  result.config.fusion_leaf_size = 0.2;
  result.config.max_layer = 1;
  result.config.min_eigen_value = 0.001;
  result.config.min_points = {{2, 3, 4, 5}};
  result.config.plane_ratio_threshold = {{0.1, 0.2, 0.3, 0.4}};
  result.config.save_display_cloud = true;
  result.source_point_count = 77U;
  result.fused_point_count = 33U;

  StaticMapBlock block;
  block.index = BlockIndex(9, 8, 7);
  RootVoxelRecord root;
  root.index = RootVoxelIndex(180, 160, 70);
  root.root_node_index = 0U;
  block.roots.push_back(root);
  StaticOctreeNode node;
  node.center = Eigen::Vector3d(180.5, 160.5, 70.5);
  node.half_length = 0.5;
  node.children[3] = 0U;
  node.plane_index = 0U;
  block.nodes.push_back(node);
  StaticPlane plane;
  plane.center = Eigen::Vector3d(180.2, 160.3, 70.4);
  plane.normal = Eigen::Vector3d(0.0, 0.0, 1.0);
  plane.covariance = Eigen::Matrix<double, 6, 6>::Identity() * 0.125;
  plane.radius = 0.75;
  block.planes.push_back(plane);
  block.display_points.push_back(test_point(180.2, 160.3, 70.4,
                                            0.25, 42.0F));
  result.blocks.push_back(block);
  return result;
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

TEST(SpatialMapBuilder, SplitsNonPlanarRootAndHonorsMaxLayer) {
  SpatialMapPointVector points;
  for (const double z : {0.2, 0.8}) {
    for (int x = 0; x < 4; ++x) {
      for (int y = 0; y < 4; ++y) {
        points.push_back(test_point(0.10 + 0.11 * x,
                                    0.10 + 0.11 * y, z, 1e-5));
      }
    }
  }

  SpatialMapConfig split_config;
  split_config.max_layer = 1;
  SpatialMapBuildResult split_result;
  std::string error;
  ASSERT_TRUE(SpatialMapBuilder::build(points, split_config, split_result,
                                       error))
      << error;
  ASSERT_EQ(split_result.blocks.size(), 1U);
  const StaticMapBlock &block = split_result.blocks[0];
  ASSERT_EQ(block.roots.size(), 1U);
  ASSERT_EQ(block.nodes.size(), 3U);
  ASSERT_EQ(block.planes.size(), 2U);
  const StaticOctreeNode &root = block.nodes[block.roots[0].root_node_index];
  EXPECT_EQ(root.plane_index, kInvalidSpatialIndex);
  ASSERT_NE(root.children[0], kInvalidSpatialIndex);
  ASSERT_NE(root.children[1], kInvalidSpatialIndex);
  EXPECT_NE(block.nodes[root.children[0]].plane_index, kInvalidSpatialIndex);
  EXPECT_NE(block.nodes[root.children[1]].plane_index, kInvalidSpatialIndex);

  SpatialMapConfig capped_config = split_config;
  capped_config.max_layer = 0;
  SpatialMapBuildResult capped_result;
  ASSERT_TRUE(SpatialMapBuilder::build(points, capped_config, capped_result,
                                       error))
      << error;
  EXPECT_TRUE(capped_result.blocks.empty());
  EXPECT_EQ(capped_result.fused_point_count, points.size());
}

TEST(SpatialMapBuilder, StoresLambda2RadiusAndNumericPlaneCovariance) {
  const double point_variance = 1e-4;
  SpatialMapPointVector points;
  points.push_back(test_point(0.1, 0.1, 0.2, point_variance));
  points.push_back(test_point(0.3, 0.1, 0.2, point_variance));
  points.push_back(test_point(0.1, 0.3, 0.2, point_variance));
  points.push_back(test_point(0.3, 0.3, 0.2, point_variance));
  SpatialMapConfig config;
  config.max_layer = 0;
  config.min_points[0] = 3;

  SpatialMapBuildResult result;
  std::string error;
  ASSERT_TRUE(SpatialMapBuilder::build(points, config, result, error)) << error;
  ASSERT_EQ(result.blocks.size(), 1U);
  ASSERT_EQ(result.blocks[0].planes.size(), 1U);
  const StaticPlane &plane = result.blocks[0].planes[0];
  EXPECT_NEAR(plane.radius, 0.01, 1e-12);
  EXPECT_NEAR(plane.normal.x(), 0.0, 1e-12);
  EXPECT_NEAR(plane.normal.y(), 0.0, 1e-12);
  EXPECT_NEAR(plane.normal.z(), 1.0, 1e-12);

  Eigen::Matrix<double, 6, 6> expected =
      Eigen::Matrix<double, 6, 6>::Zero();
  expected(0, 0) = point_variance / (4.0 * 0.01);
  expected(1, 1) = point_variance / (4.0 * 0.01);
  expected.block<3, 3>(3, 3) =
      Eigen::Matrix3d::Identity() * point_variance / 4.0;
  EXPECT_TRUE(plane.covariance.isApprox(expected, 1e-10))
      << "actual:\n" << plane.covariance << "\nexpected:\n" << expected;
}

TEST(SpatialMapBuilder, EmptyAndNonFiniteInputsFailAtomically) {
  const SpatialMapConfig config;
  SpatialMapBuildResult output = full_sentinel_result();
  const SpatialMapBuildResult sentinel = output;
  std::string error;

  const SpatialMapPointVector empty;
  EXPECT_FALSE(SpatialMapBuilder::build(empty, config, output, error));
  EXPECT_FALSE(error.empty());
  EXPECT_TRUE(results_equal(output, sentinel));

  SpatialMapPointVector invalid{test_point(0.0, 0.0, 0.0)};
  invalid[0].position.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(SpatialMapBuilder::build(invalid, config, output, error));
  EXPECT_TRUE(results_equal(output, sentinel));

  invalid[0] = test_point(0.0, 0.0, 0.0);
  invalid[0].covariance(2, 2) =
      std::numeric_limits<double>::infinity();
  EXPECT_FALSE(SpatialMapBuilder::build(invalid, config, output, error));
  EXPECT_TRUE(results_equal(output, sentinel));

  invalid[0] = test_point(0.0, 0.0, 0.0);
  invalid[0].covariance(0, 1) = 1.0;
  EXPECT_FALSE(SpatialMapBuilder::build(invalid, config, output, error));
  EXPECT_TRUE(results_equal(output, sentinel));

  invalid[0] = test_point(0.0, 0.0, 0.0);
  invalid[0].covariance(1, 1) = -1.0;
  EXPECT_FALSE(SpatialMapBuilder::build(invalid, config, output, error));
  EXPECT_TRUE(results_equal(output, sentinel));

  invalid[0] = test_point(0.0, 0.0, 0.0);
  invalid[0].intensity = std::numeric_limits<float>::infinity();
  EXPECT_FALSE(SpatialMapBuilder::build(invalid, config, output, error));
  EXPECT_TRUE(results_equal(output, sentinel));

  SpatialMapConfig invalid_config = config;
  invalid_config.size_z = 10.5;
  invalid[0] = test_point(0.0, 0.0, 0.0);
  EXPECT_FALSE(
      SpatialMapBuilder::build(invalid, invalid_config, output, error));
  EXPECT_TRUE(results_equal(output, sentinel));
}

TEST(SpatialMapBuilder, DistantTiltedPlanesHaveSymmetricCovariance) {
  SpatialMapConfig config;
  config.fusion_leaf_size = 0.05;
  SpatialMapPointVector points;
  for (int x = 0; x < 6; ++x) {
    for (int y = 0; y < 6; ++y) {
      points.push_back(test_point(-359.9 + 0.13 * x,
                                  280.1 + 0.13 * y,
                                  90.15 + 0.017 * x + 0.029 * y,
                                  1e-4 * (1 + x + y)));
    }
  }
  SpatialMapBuildResult result;
  std::string error;
  ASSERT_TRUE(SpatialMapBuilder::build(points, config, result, error)) << error;
  std::size_t plane_count = 0;
  for (const auto &block : result.blocks) {
    for (const auto &plane : block.planes) {
      ++plane_count;
      EXPECT_NEAR(plane.normal.norm(), 1.0, 1e-6);
      EXPECT_TRUE(plane.covariance.isApprox(plane.covariance.transpose(), 1e-9));
      EXPECT_TRUE((plane.covariance.array() ==
                   plane.covariance.transpose().array()).all());
    }
  }
  EXPECT_GT(plane_count, 0U);
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
