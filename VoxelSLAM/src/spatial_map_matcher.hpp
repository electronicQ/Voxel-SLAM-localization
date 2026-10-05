#pragma once

#include "spatial_map_types.hpp"

#include <cstdint>

struct StaticMapNodeHint {
  BlockIndex block_index;
  std::uint32_t node_index = kInvalidSpatialIndex;
};

struct StaticPlaneMatch {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  // Point-to-plane distance variance in length^2, not standard deviation.
  double sigma_d = 0.0;
  BlockIndex block_index;
  std::uint32_t node_index = kInvalidSpatialIndex;
  std::uint32_t plane_index = kInvalidSpatialIndex;
};

class StaticMapMatcher {
 public:
  explicit StaticMapMatcher(const SpatialMapConfig &config);

  bool match(const StaticMapBlock &block,
             const Eigen::Vector3d &world_point,
             const Eigen::Matrix3d &world_covariance,
             const StaticMapNodeHint *hint,
             StaticPlaneMatch &match) const;

 private:
  SpatialMapConfig config_;
  bool config_valid_ = false;
};
