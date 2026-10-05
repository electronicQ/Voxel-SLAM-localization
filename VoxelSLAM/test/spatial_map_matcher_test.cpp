#include "spatial_map_builder.hpp"
#include "spatial_map_matcher.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

namespace {

SpatialMapPoint plane_point(double x, double y, double z,
                            double covariance = 1e-6) {
  SpatialMapPoint point;
  point.position = Eigen::Vector3d(x, y, z);
  point.covariance = Eigen::Matrix3d::Identity() * covariance;
  return point;
}

SpatialMapBuildResult build_horizontal_plane(double covariance = 1e-6) {
  SpatialMapPointVector points;
  for (int x = 0; x < 4; ++x) {
    for (int y = 0; y < 4; ++y) {
      points.push_back(plane_point(0.10 + 0.11 * x, 0.10 + 0.11 * y,
                                   0.05, covariance));
    }
  }
  SpatialMapBuildResult result;
  std::string error;
  EXPECT_TRUE(SpatialMapBuilder::build(points, SpatialMapConfig{}, result,
                                       error))
      << error;
  return result;
}

StaticPlane test_plane(const Eigen::Vector3d &center, double radius = 1.0) {
  StaticPlane plane;
  plane.center = center;
  plane.normal = Eigen::Vector3d::UnitZ();
  plane.radius = radius;
  return plane;
}

StaticMapBlock single_root_block(const StaticPlane &plane) {
  StaticMapBlock block;
  block.index = BlockIndex(0, 0, 0);
  RootVoxelRecord root;
  root.index = RootVoxelIndex(0, 0, 0);
  root.root_node_index = 0U;
  block.roots.push_back(root);
  StaticOctreeNode node;
  node.center = Eigen::Vector3d(0.5, 0.5, 0.5);
  node.half_length = 0.5;
  node.plane_index = 0U;
  block.nodes.push_back(node);
  block.planes.push_back(plane);
  return block;
}

void expect_same_match(const StaticPlaneMatch &left,
                       const StaticPlaneMatch &right) {
  EXPECT_EQ(left.block_index, right.block_index);
  EXPECT_EQ(left.node_index, right.node_index);
  EXPECT_EQ(left.plane_index, right.plane_index);
  EXPECT_TRUE((left.center.array() == right.center.array()).all());
  EXPECT_TRUE((left.normal.array() == right.normal.array()).all());
  EXPECT_DOUBLE_EQ(left.sigma_d, right.sigma_d);
}

TEST(StaticMapMatcher, MatchesHorizontalPlaneWithPositiveFiniteSigma) {
  const SpatialMapBuildResult result = build_horizontal_plane();
  ASSERT_EQ(result.blocks.size(), 1U);
  StaticPlaneMatch match;
  const StaticMapMatcher matcher(result.config);
  ASSERT_TRUE(matcher.match(result.blocks[0],
                            Eigen::Vector3d(0.25, 0.25, 0.051),
                            Eigen::Matrix3d::Identity() * 1e-4, nullptr,
                            match));
  EXPECT_NEAR(std::abs(match.normal.z()), 1.0, 1e-6);
  EXPECT_TRUE(std::isfinite(match.sigma_d));
  EXPECT_GT(match.sigma_d, 0.0);
}

TEST(StaticMapMatcher, RejectsRadiusGateThreeSigmaGateAndMissingRoot) {
  const SpatialMapBuildResult result = build_horizontal_plane();
  ASSERT_EQ(result.blocks.size(), 1U);
  const StaticMapMatcher matcher(result.config);
  StaticPlaneMatch match;

  EXPECT_FALSE(matcher.match(result.blocks[0],
                             Eigen::Vector3d(0.95, 0.95, 0.05),
                             Eigen::Matrix3d::Identity() * 1e-4, nullptr,
                             match));
  EXPECT_FALSE(matcher.match(result.blocks[0],
                             Eigen::Vector3d(0.25, 0.25, 0.25),
                             Eigen::Matrix3d::Identity() * 1e-12, nullptr,
                             match));
  EXPECT_FALSE(matcher.match(result.blocks[0],
                             Eigen::Vector3d(1.25, 0.25, 0.05),
                             Eigen::Matrix3d::Identity() * 1e-4, nullptr,
                             match));
}

TEST(StaticMapMatcher, ValidHintAndColdLookupReturnIdenticalMatch) {
  const SpatialMapBuildResult result = build_horizontal_plane();
  ASSERT_EQ(result.blocks.size(), 1U);
  const StaticMapMatcher matcher(result.config);
  const Eigen::Vector3d query(0.25, 0.25, 0.051);
  const Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity() * 1e-4;
  StaticPlaneMatch cold;
  ASSERT_TRUE(matcher.match(result.blocks[0], query, covariance, nullptr,
                            cold));

  StaticMapNodeHint hint;
  hint.block_index = cold.block_index;
  hint.node_index = cold.node_index;
  StaticPlaneMatch hinted;
  ASSERT_TRUE(matcher.match(result.blocks[0], query, covariance, &hint,
                            hinted));
  expect_same_match(cold, hinted);
}

TEST(StaticMapMatcher, SplitBoundarySiblingHintCannotChangeColdPath) {
  StaticMapBlock block;
  block.index = BlockIndex(0, 0, 0);
  RootVoxelRecord root;
  root.index = RootVoxelIndex(0, 0, 0);
  root.root_node_index = 0U;
  block.roots.push_back(root);

  StaticOctreeNode root_node;
  root_node.center = Eigen::Vector3d(0.5, 0.5, 0.5);
  root_node.half_length = 0.5;
  root_node.children[0] = 1U;
  root_node.children[4] = 2U;
  block.nodes.push_back(root_node);

  StaticOctreeNode cold_child;
  cold_child.center = Eigen::Vector3d(0.25, 0.25, 0.25);
  cold_child.half_length = 0.25;
  cold_child.plane_index = 0U;
  block.nodes.push_back(cold_child);

  StaticOctreeNode sibling_child;
  sibling_child.center = Eigen::Vector3d(0.75, 0.25, 0.25);
  sibling_child.half_length = 0.25;
  sibling_child.plane_index = 1U;
  block.nodes.push_back(sibling_child);
  block.planes.push_back(test_plane(cold_child.center));
  block.planes.push_back(test_plane(sibling_child.center));

  const Eigen::Vector3d query(0.5, 0.25, 0.25);
  const Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity() * 1e-4;
  const StaticMapMatcher matcher(SpatialMapConfig{});
  StaticPlaneMatch cold;
  ASSERT_TRUE(matcher.match(block, query, covariance, nullptr, cold));
  ASSERT_EQ(cold.node_index, 1U);

  StaticMapNodeHint sibling_hint;
  sibling_hint.block_index = block.index;
  sibling_hint.node_index = 2U;
  StaticPlaneMatch hinted;
  ASSERT_TRUE(matcher.match(block, query, covariance, &sibling_hint,
                            hinted));
  expect_same_match(cold, hinted);

  StaticMapNodeHint path_hint;
  path_hint.block_index = block.index;
  path_hint.node_index = 1U;
  ASSERT_TRUE(matcher.match(block, query, covariance, &path_hint, hinted));
  expect_same_match(cold, hinted);
}

TEST(StaticMapMatcher, RootBoundaryHintCannotBypassMissingOwningRoot) {
  const StaticMapBlock block =
      single_root_block(test_plane(Eigen::Vector3d(0.5, 0.5, 0.5)));
  const Eigen::Vector3d query(1.0, 0.5, 0.5);
  const Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity() * 1e-4;
  const StaticMapMatcher matcher(SpatialMapConfig{});
  StaticPlaneMatch match;
  EXPECT_FALSE(matcher.match(block, query, covariance, nullptr, match));

  StaticMapNodeHint preceding_root_hint;
  preceding_root_hint.block_index = block.index;
  preceding_root_hint.node_index = 0U;
  EXPECT_FALSE(matcher.match(block, query, covariance,
                             &preceding_root_hint, match));
}

TEST(StaticMapMatcher, SigmaDIsThePointToPlaneDistanceVariance) {
  StaticPlane plane = test_plane(Eigen::Vector3d::Zero());
  plane.covariance(0, 0) = 4e-4;
  plane.covariance(1, 1) = 9e-4;
  plane.covariance(2, 2) = 16e-4;
  plane.covariance(5, 5) = 25e-4;
  plane.covariance(0, 5) = 2e-4;
  plane.covariance(5, 0) = 2e-4;
  const StaticMapBlock block = single_root_block(plane);
  const Eigen::Vector3d query(0.2, 0.3, 0.001);
  Eigen::Matrix3d point_covariance = Eigen::Matrix3d::Zero();
  point_covariance.diagonal() << 1e-4, 2e-4, 3e-4;

  const StaticMapMatcher matcher(SpatialMapConfig{});
  StaticPlaneMatch match;
  ASSERT_TRUE(matcher.match(block, query, point_covariance, nullptr, match));

  Eigen::Matrix<double, 1, 6> jacobian;
  jacobian << 0.2, 0.3, 0.001, 0.0, 0.0, -1.0;
  const double expected_plane_variance =
      (jacobian * plane.covariance * jacobian.transpose())(0, 0);
  const double expected_point_variance =
      (plane.normal.transpose() * point_covariance * plane.normal)(0, 0);
  EXPECT_NEAR(match.sigma_d,
              expected_plane_variance + expected_point_variance, 1e-15);
  EXPECT_NEAR(match.sigma_d, 0.0028170016, 1e-15);
}

TEST(StaticMapMatcher, InvalidHintFallsBackAndCannotBypassMissingRoot) {
  const SpatialMapBuildResult result = build_horizontal_plane();
  ASSERT_EQ(result.blocks.size(), 1U);
  const StaticMapMatcher matcher(result.config);
  const Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity() * 1e-4;

  StaticMapNodeHint invalid_hint;
  invalid_hint.block_index = result.blocks[0].index;
  invalid_hint.node_index = std::numeric_limits<std::uint32_t>::max() - 1U;
  StaticPlaneMatch cold;
  StaticPlaneMatch fallback;
  ASSERT_TRUE(matcher.match(result.blocks[0],
                            Eigen::Vector3d(0.25, 0.25, 0.051), covariance,
                            nullptr, cold));
  ASSERT_TRUE(matcher.match(result.blocks[0],
                            Eigen::Vector3d(0.25, 0.25, 0.051), covariance,
                            &invalid_hint, fallback));
  expect_same_match(cold, fallback);

  invalid_hint.block_index = BlockIndex(1, 0, 0);
  invalid_hint.node_index = cold.node_index;
  ASSERT_TRUE(matcher.match(result.blocks[0],
                            Eigen::Vector3d(0.25, 0.25, 0.051), covariance,
                            &invalid_hint, fallback));
  expect_same_match(cold, fallback);

  invalid_hint.block_index = result.blocks[0].index;
  invalid_hint.node_index = cold.node_index;
  EXPECT_FALSE(matcher.match(result.blocks[0],
                             Eigen::Vector3d(1.25, 0.25, 0.05), covariance,
                             &invalid_hint, fallback));
}

TEST(StaticMapMatcher, RejectsNonFiniteInputsAndNonPositiveSigma) {
  const SpatialMapBuildResult result = build_horizontal_plane(0.0);
  ASSERT_EQ(result.blocks.size(), 1U);
  const StaticMapMatcher matcher(result.config);
  StaticPlaneMatch match;
  const Eigen::Vector3d query(0.25, 0.25, 0.05);

  EXPECT_FALSE(matcher.match(result.blocks[0], query,
                             Eigen::Matrix3d::Zero(), nullptr, match));

  Eigen::Matrix3d non_finite = Eigen::Matrix3d::Identity();
  non_finite(0, 0) = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(matcher.match(result.blocks[0], query, non_finite, nullptr,
                             match));
  EXPECT_FALSE(matcher.match(
      result.blocks[0],
      Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.25, 0.05),
      Eigen::Matrix3d::Identity(), nullptr, match));
}

TEST(StaticMapMatcher, RejectsReachableOutOfRangeSerializedIndices) {
  const Eigen::Vector3d query(0.25, 0.25, 0.25);
  const Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity() * 1e-4;
  const StaticMapMatcher matcher(SpatialMapConfig{});
  StaticPlaneMatch match;

  StaticMapBlock bad_root =
      single_root_block(test_plane(Eigen::Vector3d(0.25, 0.25, 0.25)));
  bad_root.roots[0].root_node_index = 7U;
  EXPECT_FALSE(matcher.match(bad_root, query, covariance, nullptr, match));

  StaticMapBlock bad_child = bad_root;
  bad_child.roots[0].root_node_index = 0U;
  bad_child.nodes[0].plane_index = kInvalidSpatialIndex;
  bad_child.nodes[0].children[0] = 7U;
  EXPECT_FALSE(matcher.match(bad_child, query, covariance, nullptr, match));

  StaticMapBlock bad_plane = bad_root;
  bad_plane.roots[0].root_node_index = 0U;
  bad_plane.nodes[0].plane_index = 7U;
  EXPECT_FALSE(matcher.match(bad_plane, query, covariance, nullptr, match));
}

}  // namespace
