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

}  // namespace
