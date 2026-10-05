#include "spatial_map_types.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_set>
#include <vector>

namespace {

TEST(SpatialMapGeometry, FloorsNegativeCoordinatesAndKeepsExactBoundaries) {
  EXPECT_EQ(floor_grid_index(0.0, 20.0), 0);
  EXPECT_EQ(floor_grid_index(19.999, 20.0), 0);
  EXPECT_EQ(floor_grid_index(20.0, 20.0), 1);
  EXPECT_EQ(floor_grid_index(-0.001, 20.0), -1);
  EXPECT_EQ(floor_grid_index(-20.0, 20.0), -1);
  EXPECT_EQ(floor_grid_index(-20.001, 20.0), -2);
}

TEST(SpatialMapGeometry, RejectsInvalidResolutionAndNonFiniteCoordinates) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();

  EXPECT_THROW(floor_grid_index(0.0, 0.0), std::invalid_argument);
  EXPECT_THROW(floor_grid_index(0.0, -1.0), std::invalid_argument);
  EXPECT_THROW(floor_grid_index(0.0, nan), std::invalid_argument);
  EXPECT_THROW(floor_grid_index(0.0, infinity), std::invalid_argument);
  EXPECT_THROW(floor_grid_index(nan, 1.0), std::invalid_argument);
  EXPECT_THROW(floor_grid_index(infinity, 1.0), std::invalid_argument);
  EXPECT_THROW(floor_grid_index(std::ldexp(1.0, 63), 1.0),
               std::overflow_error);
}

TEST(SpatialMapGeometry, RejectsMisalignedOrNonFiniteConfiguration) {
  SpatialMapConfig config;
  std::string error;
  EXPECT_TRUE(config.validate(error)) << error;
  EXPECT_TRUE(error.empty());

  config.size_z = 10.5;
  EXPECT_FALSE(config.validate(error));
  EXPECT_NE(error.find("integer multiple"), std::string::npos);

  config = SpatialMapConfig{};
  config.size_x = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(config.validate(error));
  EXPECT_NE(error.find("finite"), std::string::npos);

  config = SpatialMapConfig{};
  config.fusion_leaf_size =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(config.validate(error));
  EXPECT_NE(error.find("finite"), std::string::npos);
}

TEST(SpatialMapGeometry, UsesExactSpatialMapDefaults) {
  const SpatialMapConfig config;

  EXPECT_DOUBLE_EQ(config.size_x, 20.0);
  EXPECT_DOUBLE_EQ(config.size_y, 20.0);
  EXPECT_DOUBLE_EQ(config.size_z, 10.0);
  EXPECT_DOUBLE_EQ(config.fusion_leaf_size, 0.1);
  EXPECT_DOUBLE_EQ(config.root_voxel_size, 1.0);
  EXPECT_EQ(config.max_layer, 2);
  EXPECT_DOUBLE_EQ(config.min_eigen_value, 0.0025);
  EXPECT_EQ(config.min_points, (std::array<int, 4>{{5, 5, 5, 5}}));
  EXPECT_EQ(config.plane_ratio_threshold,
            (std::array<double, 4>{{0.25, 0.25, 0.25, 0.25}}));
  EXPECT_FALSE(config.save_display_cloud);
}

TEST(SpatialMapGeometry, RejectsInvalidLayerPointAndPlaneParameters) {
  SpatialMapConfig config;
  std::string error;

  config.max_layer = 4;
  EXPECT_FALSE(config.validate(error));
  EXPECT_NE(error.find("max_layer"), std::string::npos);

  config = SpatialMapConfig{};
  config.min_points[2] = 0;
  EXPECT_FALSE(config.validate(error));
  EXPECT_NE(error.find("min_points"), std::string::npos);

  config = SpatialMapConfig{};
  config.plane_ratio_threshold[1] = -0.01;
  EXPECT_FALSE(config.validate(error));
  EXPECT_NE(error.find("plane_ratio_threshold"), std::string::npos);

  config = SpatialMapConfig{};
  config.min_eigen_value = -0.01;
  EXPECT_FALSE(config.validate(error));
  EXPECT_NE(error.find("min_eigen_value"), std::string::npos);
}

TEST(SpatialMapGeometry, RejectsRootVoxelCountOutsideSigned64BitRange) {
  SpatialMapConfig config;
  std::string error;
  config.size_x = std::ldexp(1.0, 63);

  EXPECT_FALSE(config.validate(error));
  EXPECT_NE(error.find("integer multiple"), std::string::npos);
}

TEST(SpatialMapGeometry, RejectsLargeNonIntegralRootVoxelRatio) {
  SpatialMapConfig config;
  std::string error;
  config.size_x = 1000000000000.25;

  EXPECT_FALSE(config.validate(error));
  EXPECT_NE(error.find("integer multiple"), std::string::npos);
}

TEST(SpatialMapGeometry, MapsWorldAndNegativeRootVoxelsToBlocks) {
  const SpatialMapConfig config;

  EXPECT_EQ(world_to_block(Eigen::Vector3d(20.0, -0.001, -10.0), config),
            BlockIndex(1, -1, -1));
  EXPECT_EQ(world_to_root_voxel(
                Eigen::Vector3d(-0.001, -20.0, -20.001), config),
            RootVoxelIndex(-1, -20, -21));
  EXPECT_EQ(root_voxel_to_block(RootVoxelIndex(-1, -1, -1), config),
            BlockIndex(-1, -1, -1));
  EXPECT_EQ(root_voxel_to_block(RootVoxelIndex(-20, -20, -10), config),
            BlockIndex(-1, -1, -1));
  EXPECT_EQ(root_voxel_to_block(RootVoxelIndex(-21, -21, -11), config),
            BlockIndex(-2, -2, -2));
}

TEST(SpatialMapGeometry, WorldAndRootVoxelMappingsAgreeAtFloatingBoundary) {
  SpatialMapConfig config;
  config.size_x = 0.3;
  config.size_y = 0.6;
  config.size_z = 0.3;
  config.root_voxel_size = 0.1;
  std::string error;
  ASSERT_TRUE(config.validate(error)) << error;

  const double boundary = 0.3;
  const std::vector<double> coordinates{
      std::nextafter(boundary, -std::numeric_limits<double>::infinity()),
      boundary,
      std::nextafter(boundary, std::numeric_limits<double>::infinity()),
      -boundary};
  for (const double coordinate : coordinates) {
    const Eigen::Vector3d point(coordinate, coordinate, coordinate);
    EXPECT_EQ(world_to_block(point, config),
              root_voxel_to_block(world_to_root_voxel(point, config),
                                  config));
  }
}

TEST(SpatialMapGeometry, SphereIntersectionIncludesSurfaceAndZeroRadius) {
  const SpatialMapConfig config;
  const BlockAabb aabb = block_aabb(BlockIndex(0, 0, 0), config);

  EXPECT_TRUE(sphere_intersects_aabb(
      Eigen::Vector3d(25.0, 5.0, 5.0), 5.0, aabb));
  EXPECT_FALSE(sphere_intersects_aabb(
      Eigen::Vector3d(25.0, 5.0, 5.0), 4.999, aabb));
  EXPECT_TRUE(sphere_intersects_aabb(
      Eigen::Vector3d(20.0, 5.0, 5.0), 0.0, aabb));
  EXPECT_FALSE(sphere_intersects_aabb(
      Eigen::Vector3d(20.001, 5.0, 5.0), 0.0, aabb));
}

TEST(SpatialMapGeometry, RejectsAabbWithCollapsedDoubleBoundaries) {
  const SpatialMapConfig config;

  EXPECT_THROW(
      block_aabb(BlockIndex(std::numeric_limits<std::int64_t>::max(), 0, 0),
                 config),
      std::overflow_error);
}

TEST(SpatialMapGeometry, EnumeratesOnlyBlocksWhoseAabbTouchesSphere) {
  SpatialMapConfig config;
  const Eigen::Vector3d center(45.0, 12.0, 3.0);
  const auto blocks = blocks_intersecting_sphere(center, 30.0, config);
  EXPECT_NE(std::find(blocks.begin(), blocks.end(), BlockIndex{2, 0, 0}),
            blocks.end());
  for (const BlockIndex &index : blocks) {
    EXPECT_TRUE(sphere_intersects_aabb(center, 30.0,
                                       block_aabb(index, config)));
  }
}

TEST(SpatialMapGeometry, EnumeratesCompleteSortedUniqueMultiAxisRange) {
  const SpatialMapConfig config;
  const std::vector<BlockIndex> blocks = blocks_intersecting_sphere(
      Eigen::Vector3d(19.0, -1.0, 9.0), 2.0, config);
  const std::vector<BlockIndex> expected{
      BlockIndex(0, -1, 0), BlockIndex(0, -1, 1),
      BlockIndex(0, 0, 0),  BlockIndex(0, 0, 1),
      BlockIndex(1, -1, 0), BlockIndex(1, -1, 1),
      BlockIndex(1, 0, 0),  BlockIndex(1, 0, 1)};

  EXPECT_EQ(blocks, expected);
  EXPECT_TRUE(std::is_sorted(blocks.begin(), blocks.end()));
  EXPECT_EQ(std::adjacent_find(blocks.begin(), blocks.end()), blocks.end());
}

TEST(SpatialMapGeometry, IncludesAllBlocksTouchingBoundaryPoint) {
  const SpatialMapConfig config;
  const std::vector<BlockIndex> blocks = blocks_intersecting_sphere(
      Eigen::Vector3d(20.0, 5.0, 5.0), 0.0, config);

  ASSERT_EQ(blocks.size(), 2U);
  EXPECT_EQ(blocks[0], BlockIndex(0, 0, 0));
  EXPECT_EQ(blocks[1], BlockIndex(1, 0, 0));
}

TEST(SpatialMapGeometry, IndexOrderingAndHashesAreDeterministicAndUnique) {
  static_assert(
      std::is_same<decltype(BlockIndex{}.x), std::int64_t>::value,
      "BlockIndex must use signed 64-bit coordinates");
  static_assert(
      std::is_same<decltype(RootVoxelIndex{}.x), std::int64_t>::value,
      "RootVoxelIndex must use signed 64-bit coordinates");

  const std::vector<BlockIndex> input{
      BlockIndex(1, 0, 0), BlockIndex(-1, 2, 3), BlockIndex(1, 0, 0),
      BlockIndex(-1, 2, 2), BlockIndex(0, 0, 0)};
  std::unordered_set<BlockIndex, BlockIndexHash> unique(input.begin(),
                                                        input.end());
  EXPECT_EQ(unique.size(), 4U);
  EXPECT_EQ(BlockIndexHash{}(BlockIndex(-7, 11, -13)),
            BlockIndexHash{}(BlockIndex(-7, 11, -13)));
  if (sizeof(std::size_t) == sizeof(std::uint64_t)) {
    EXPECT_EQ(BlockIndexHash{}(BlockIndex(-7, 11, -13)),
              static_cast<std::size_t>(UINT64_C(17488281428513000436)));
  }

  std::vector<BlockIndex> sorted(input.begin(), input.end());
  std::sort(sorted.begin(), sorted.end());
  sorted.erase(std::unique(sorted.begin(), sorted.end()), sorted.end());
  const std::vector<BlockIndex> expected{
      BlockIndex(-1, 2, 2), BlockIndex(-1, 2, 3), BlockIndex(0, 0, 0),
      BlockIndex(1, 0, 0)};
  EXPECT_EQ(sorted, expected);

  std::unordered_set<RootVoxelIndex, RootVoxelIndexHash> root_unique;
  root_unique.insert(RootVoxelIndex(-1, -2, -3));
  root_unique.insert(RootVoxelIndex(-1, -2, -3));
  root_unique.insert(RootVoxelIndex(-1, -2, -2));
  EXPECT_EQ(root_unique.size(), 2U);
  EXPECT_EQ(RootVoxelIndexHash{}(RootVoxelIndex(-7, 11, -13)),
            BlockIndexHash{}(BlockIndex(-7, 11, -13)));
}

TEST(SpatialMapGeometry, RejectsInvalidSphereInputs) {
  const SpatialMapConfig config;
  const BlockAabb aabb = block_aabb(BlockIndex(0, 0, 0), config);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();

  EXPECT_THROW(sphere_intersects_aabb(Eigen::Vector3d(nan, 0.0, 0.0),
                                      1.0, aabb),
               std::invalid_argument);
  EXPECT_THROW(sphere_intersects_aabb(Eigen::Vector3d::Zero(), -1.0, aabb),
               std::invalid_argument);
  EXPECT_THROW(blocks_intersecting_sphere(Eigen::Vector3d::Zero(), infinity,
                                          config),
               std::invalid_argument);
}

}  // namespace
