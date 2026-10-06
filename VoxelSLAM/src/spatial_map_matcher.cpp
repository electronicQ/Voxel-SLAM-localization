#include "spatial_map_matcher.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <string>

namespace {

bool vector_is_finite(const Eigen::Vector3d &value) {
  return std::isfinite(value.x()) && std::isfinite(value.y()) &&
         std::isfinite(value.z());
}

template <int Rows, int Columns>
bool matrix_is_finite(const Eigen::Matrix<double, Rows, Columns> &value) {
  for (int row = 0; row < Rows; ++row) {
    for (int column = 0; column < Columns; ++column) {
      if (!std::isfinite(value(row, column))) {
        return false;
      }
    }
  }
  return true;
}

bool node_contains(const StaticOctreeNode &node,
                   const Eigen::Vector3d &point) {
  if (!vector_is_finite(node.center) ||
      !std::isfinite(node.half_length) || node.half_length <= 0.0) {
    return false;
  }
  const Eigen::Vector3d offset = point - node.center;
  return vector_is_finite(offset) &&
         (offset.array().abs() <= node.half_length).all();
}

std::uint32_t find_root_node(const StaticMapBlock &block,
                             const RootVoxelIndex &root_index) {
  const auto record = std::lower_bound(
      block.roots.begin(), block.roots.end(), root_index,
      [](const RootVoxelRecord &left, const RootVoxelIndex &right) {
        return left.index < right;
      });
  if (record == block.roots.end() || record->index != root_index) {
    return kInvalidSpatialIndex;
  }
  return record->root_node_index;
}

int child_for_point(const StaticOctreeNode &node,
                    const Eigen::Vector3d &point) {
  return (point.x() > node.center.x() ? 4 : 0) +
         (point.y() > node.center.y() ? 2 : 0) +
         (point.z() > node.center.z() ? 1 : 0);
}

bool query_path_reaches_hint(const StaticMapBlock &block,
                             std::uint32_t root_node_index,
                             const Eigen::Vector3d &world_point,
                             std::uint32_t hint_node_index) {
  std::uint32_t node_index = root_node_index;
  for (std::size_t visited = 0; visited < block.nodes.size(); ++visited) {
    if (node_index >= block.nodes.size()) {
      return false;
    }
    const StaticOctreeNode &node = block.nodes[node_index];
    if (!node_contains(node, world_point)) {
      return false;
    }
    if (node_index == hint_node_index) {
      return true;
    }
    if (node.plane_index != kInvalidSpatialIndex) {
      return false;
    }
    const std::uint32_t child_index =
        node.children[child_for_point(node, world_point)];
    if (child_index == kInvalidSpatialIndex) {
      return false;
    }
    node_index = child_index;
  }
  return false;
}

bool evaluate_plane(const StaticMapBlock &block,
                    std::uint32_t node_index,
                    const StaticPlane &plane,
                    std::uint32_t plane_index,
                    const Eigen::Vector3d &world_point,
                    const Eigen::Matrix3d &world_covariance,
                    StaticPlaneMatch &match) {
  if (!vector_is_finite(plane.center) ||
      !vector_is_finite(plane.normal) ||
      !matrix_is_finite(plane.covariance) ||
      std::abs(plane.normal.norm() - 1.0) > 1e-6 ||
      !std::isfinite(plane.radius) || plane.radius < 0.0) {
    return false;
  }

  const Eigen::Vector3d offset = world_point - plane.center;
  if (!vector_is_finite(offset)) {
    return false;
  }
  const double plane_distance = std::abs(plane.normal.dot(offset));
  const double center_distance_squared = offset.squaredNorm();
  const double radial_distance_squared =
      center_distance_squared - plane_distance * plane_distance;
  const double radius_gate = 9.0 * plane.radius;
  if (!std::isfinite(plane_distance) ||
      !std::isfinite(center_distance_squared) ||
      !std::isfinite(radial_distance_squared) ||
      !std::isfinite(radius_gate) ||
      radial_distance_squared > radius_gate) {
    return false;
  }

  Eigen::Matrix<double, 1, 6> jacobian;
  jacobian.head<3>() = offset;
  jacobian.tail<3>() = -plane.normal;
  double sigma =
      (jacobian * plane.covariance * jacobian.transpose())(0, 0);
  sigma +=
      (plane.normal.transpose() * world_covariance * plane.normal)(0, 0);
  if (!std::isfinite(sigma) || sigma <= 0.0) {
    return false;
  }
  const double gate = 3.0 * std::sqrt(sigma);
  if (!std::isfinite(gate) || !(plane_distance < gate)) {
    return false;
  }

  StaticPlaneMatch candidate;
  candidate.center = plane.center;
  candidate.normal = plane.normal;
  candidate.sigma_d = sigma;
  candidate.block_index = block.index;
  candidate.node_index = node_index;
  candidate.plane_index = plane_index;
  match = candidate;
  return true;
}

}  // namespace

StaticMapMatcher::StaticMapMatcher(const SpatialMapConfig &config)
    : config_(config) {
  std::string error;
  config_valid_ = config_.validate(error);
}

bool StaticMapMatcher::match(
    const StaticMapBlock &block,
    const Eigen::Vector3d &world_point,
    const Eigen::Matrix3d &world_covariance,
    const StaticMapNodeHint *hint,
    StaticPlaneMatch &match_result) const {
  if (!config_valid_ || !vector_is_finite(world_point) ||
      !matrix_is_finite(world_covariance)) {
    return false;
  }

  try {
    if (world_to_block(world_point, config_) != block.index) {
      return false;
    }
    const RootVoxelIndex root_index =
        world_to_root_voxel(world_point, config_);

    const std::uint32_t root_node_index =
        find_root_node(block, root_index);
    if (root_node_index == kInvalidSpatialIndex ||
        root_node_index >= block.nodes.size()) {
      return false;
    }

    std::uint32_t node_index = root_node_index;
    if (hint != nullptr && hint->block_index == block.index &&
        hint->node_index < block.nodes.size() &&
        query_path_reaches_hint(block, root_node_index, world_point,
                                hint->node_index)) {
      node_index = hint->node_index;
    }

    for (std::size_t visited = 0; visited < block.nodes.size(); ++visited) {
      if (node_index >= block.nodes.size()) {
        return false;
      }
      const StaticOctreeNode &node = block.nodes[node_index];
      if (!node_contains(node, world_point)) {
        return false;
      }
      if (node.plane_index != kInvalidSpatialIndex) {
        if (node.plane_index >= block.planes.size()) {
          return false;
        }
        return evaluate_plane(block, node_index,
                              block.planes[node.plane_index],
                              node.plane_index, world_point,
                              world_covariance, match_result);
      }

      const std::uint32_t child_index =
          node.children[child_for_point(node, world_point)];
      if (child_index == kInvalidSpatialIndex) {
        return false;
      }
      node_index = child_index;
    }
  } catch (const std::exception &) {
    return false;
  }
  return false;
}
