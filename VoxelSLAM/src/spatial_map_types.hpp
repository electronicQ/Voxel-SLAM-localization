#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

constexpr std::uint32_t kInvalidSpatialIndex =
    std::numeric_limits<std::uint32_t>::max();

struct BlockIndex {
  std::int64_t x = 0;
  std::int64_t y = 0;
  std::int64_t z = 0;

  BlockIndex() = default;
  BlockIndex(std::int64_t x_value, std::int64_t y_value,
             std::int64_t z_value)
      : x(x_value), y(y_value), z(z_value) {}
};

bool operator==(const BlockIndex &left, const BlockIndex &right);
bool operator!=(const BlockIndex &left, const BlockIndex &right);
bool operator<(const BlockIndex &left, const BlockIndex &right);

struct BlockIndexHash {
  std::size_t operator()(const BlockIndex &index) const noexcept;
};

struct RootVoxelIndex {
  std::int64_t x = 0;
  std::int64_t y = 0;
  std::int64_t z = 0;

  RootVoxelIndex() = default;
  RootVoxelIndex(std::int64_t x_value, std::int64_t y_value,
                 std::int64_t z_value)
      : x(x_value), y(y_value), z(z_value) {}
};

bool operator==(const RootVoxelIndex &left, const RootVoxelIndex &right);
bool operator!=(const RootVoxelIndex &left, const RootVoxelIndex &right);
bool operator<(const RootVoxelIndex &left, const RootVoxelIndex &right);

struct RootVoxelIndexHash {
  std::size_t operator()(const RootVoxelIndex &index) const noexcept;
};

struct BlockAabb {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d minimum = Eigen::Vector3d::Zero();
  Eigen::Vector3d maximum = Eigen::Vector3d::Zero();
};

struct SpatialMapConfig {
  double size_x = 20.0;
  double size_y = 20.0;
  double size_z = 10.0;
  double fusion_leaf_size = 0.1;
  double root_voxel_size = 1.0;
  int max_layer = 2;
  double min_eigen_value = 0.0025;
  std::array<int, 4> min_points{{5, 5, 5, 5}};
  std::array<double, 4> plane_ratio_threshold{{0.25, 0.25, 0.25, 0.25}};
  bool save_display_cloud = false;

  bool validate(std::string &error) const;
};

struct SpatialMapPoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  float intensity = 0.0F;
};

using SpatialMapPointVector =
    std::vector<SpatialMapPoint, Eigen::aligned_allocator<SpatialMapPoint>>;

struct StaticPlane {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 6, 6> covariance =
      Eigen::Matrix<double, 6, 6>::Zero();
  double radius = 0.0;
};

struct StaticOctreeNode {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  double half_length = 0.0;
  std::array<std::uint32_t, 8> children{{
      kInvalidSpatialIndex, kInvalidSpatialIndex,
      kInvalidSpatialIndex, kInvalidSpatialIndex,
      kInvalidSpatialIndex, kInvalidSpatialIndex,
      kInvalidSpatialIndex, kInvalidSpatialIndex}};
  std::uint32_t plane_index = kInvalidSpatialIndex;
};

struct RootVoxelRecord {
  RootVoxelIndex index;
  std::uint32_t root_node_index = kInvalidSpatialIndex;
};

struct StaticMapBlock {
  BlockIndex index;
  std::vector<RootVoxelRecord> roots;
  std::vector<StaticOctreeNode,
              Eigen::aligned_allocator<StaticOctreeNode>> nodes;
  std::vector<StaticPlane, Eigen::aligned_allocator<StaticPlane>> planes;
  SpatialMapPointVector display_points;
};

struct SpatialMapBuildResult {
  SpatialMapConfig config;
  std::size_t source_point_count = 0;
  std::size_t fused_point_count = 0;
  std::vector<StaticMapBlock> blocks;
};

std::int64_t floor_grid_index(double coordinate, double resolution);

BlockIndex world_to_block(const Eigen::Vector3d &point,
                          const SpatialMapConfig &config);

RootVoxelIndex world_to_root_voxel(const Eigen::Vector3d &point,
                                   const SpatialMapConfig &config);

BlockIndex root_voxel_to_block(const RootVoxelIndex &index,
                               const SpatialMapConfig &config);

BlockAabb block_aabb(const BlockIndex &index,
                     const SpatialMapConfig &config);

bool sphere_intersects_aabb(const Eigen::Vector3d &center, double radius,
                            const BlockAabb &aabb);

std::vector<BlockIndex> blocks_intersecting_sphere(
    const Eigen::Vector3d &center, double radius,
    const SpatialMapConfig &config);
