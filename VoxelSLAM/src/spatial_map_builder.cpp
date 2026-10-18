#include "spatial_map_builder.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <map>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

namespace {

struct FusionKey {
  std::int64_t x = 0;
  std::int64_t y = 0;
  std::int64_t z = 0;
};

bool operator<(const FusionKey &left, const FusionKey &right) {
  if (left.x != right.x) {
    return left.x < right.x;
  }
  if (left.y != right.y) {
    return left.y < right.y;
  }
  return left.z < right.z;
}

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

bool covariance_is_symmetric(const Eigen::Matrix3d &covariance) {
  for (int row = 0; row < 3; ++row) {
    for (int column = row + 1; column < 3; ++column) {
      const double scale = std::max(
          1.0, std::max(std::abs(covariance(row, column)),
                        std::abs(covariance(column, row))));
      if (std::abs(covariance(row, column) -
                   covariance(column, row)) > 1e-12 * scale) {
        return false;
      }
    }
  }
  return true;
}

bool validate_point(const SpatialMapPoint &point, std::size_t index,
                    std::string &error) {
  if (!vector_is_finite(point.position)) {
    error = "spatial map point " + std::to_string(index) +
            " has a non-finite position";
    return false;
  }
  if (!matrix_is_finite(point.covariance)) {
    error = "spatial map point " + std::to_string(index) +
            " has a non-finite covariance";
    return false;
  }
  if (!covariance_is_symmetric(point.covariance)) {
    error = "spatial map point " + std::to_string(index) +
            " has a non-symmetric covariance";
    return false;
  }
  if ((point.covariance.diagonal().array() < 0.0).any()) {
    error = "spatial map point " + std::to_string(index) +
            " has a negative covariance diagonal";
    return false;
  }
  if (!std::isfinite(static_cast<double>(point.intensity))) {
    error = "spatial map point " + std::to_string(index) +
            " has a non-finite intensity";
    return false;
  }
  return true;
}

template <typename Scalar>
int compare_scalar(Scalar left, Scalar right) {
  if (left < right) {
    return -1;
  }
  if (right < left) {
    return 1;
  }
  if (std::signbit(left) != std::signbit(right)) {
    return std::signbit(left) ? -1 : 1;
  }
  return 0;
}

bool point_less(const SpatialMapPoint &left, const SpatialMapPoint &right) {
  for (int axis = 0; axis < 3; ++axis) {
    const int comparison =
        compare_scalar(left.position[axis], right.position[axis]);
    if (comparison != 0) {
      return comparison < 0;
    }
  }
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      const int comparison = compare_scalar(left.covariance(row, column),
                                            right.covariance(row, column));
      if (comparison != 0) {
        return comparison < 0;
      }
    }
  }
  return compare_scalar(left.intensity, right.intensity) < 0;
}

FusionKey fusion_key(const SpatialMapPoint &point,
                     const SpatialMapConfig &config) {
  return FusionKey{
      floor_grid_index(point.position.x(), config.fusion_leaf_size),
      floor_grid_index(point.position.y(), config.fusion_leaf_size),
      floor_grid_index(point.position.z(), config.fusion_leaf_size)};
}

bool average_bucket(SpatialMapPointVector &bucket,
                    SpatialMapPoint &representative,
                    std::string &error) {
  std::sort(bucket.begin(), bucket.end(), point_less);
  Eigen::Matrix<long double, 3, 1> position_sum =
      Eigen::Matrix<long double, 3, 1>::Zero();
  Eigen::Matrix<long double, 3, 3> covariance_sum =
      Eigen::Matrix<long double, 3, 3>::Zero();
  long double intensity_sum = 0.0L;
  for (const SpatialMapPoint &point : bucket) {
    position_sum += point.position.cast<long double>();
    covariance_sum += point.covariance.cast<long double>();
    intensity_sum += static_cast<long double>(point.intensity);
  }

  const long double count = static_cast<long double>(bucket.size());
  representative.position = (position_sum / count).cast<double>();
  representative.covariance = (covariance_sum / count).cast<double>();
  representative.intensity =
      static_cast<float>(intensity_sum / count);
  if (!vector_is_finite(representative.position) ||
      !matrix_is_finite(representative.covariance) ||
      !std::isfinite(static_cast<double>(representative.intensity))) {
    error = "fusion average overflowed to a non-finite value";
    return false;
  }
  return true;
}

void accumulate_bf_variance(const SpatialMapPoint &point,
                            Eigen::Matrix<double, 9, 9> &sum) {
  const Eigen::Vector3d &position = point.position;
  Eigen::Matrix<double, 6, 3> bi;
  bi << 2.0 * position.x(), 0.0, 0.0,
        position.y(), position.x(), 0.0,
        position.z(), 0.0, position.x(),
        0.0, 2.0 * position.y(), 0.0,
        0.0, position.z(), position.y(),
        0.0, 0.0, 2.0 * position.z();
  const Eigen::Matrix<double, 6, 3> bi_covariance =
      bi * point.covariance;
  Eigen::Matrix<double, 9, 9> contribution =
      Eigen::Matrix<double, 9, 9>::Zero();
  contribution.block<6, 6>(0, 0) = bi_covariance * bi.transpose();
  contribution.block<6, 3>(0, 6) = bi_covariance;
  contribution.block<3, 6>(6, 0) = bi_covariance.transpose();
  contribution.block<3, 3>(6, 6) = point.covariance;
  sum += contribution;
}

void canonicalize_eigenvectors(Eigen::Matrix3d &eigenvectors) {
  for (int column = 0; column < 3; ++column) {
    int dominant_axis = 0;
    for (int axis = 1; axis < 3; ++axis) {
      if (std::abs(eigenvectors(axis, column)) >
          std::abs(eigenvectors(dominant_axis, column))) {
        dominant_axis = axis;
      }
    }
    if (eigenvectors(dominant_axis, column) < 0.0) {
      eigenvectors.col(column) *= -1.0;
    }
  }
}

bool make_static_plane(const Eigen::Vector3d &center,
                       const Eigen::Vector3d &eigenvalues,
                       const Eigen::Matrix3d &eigenvectors,
                       const Eigen::Matrix<double, 9, 9> &covariance_sum,
                       std::size_t point_count,
                       StaticPlane &plane) {
  const double inverse_count = 1.0 / static_cast<double>(point_count);
  Eigen::Matrix<double, 3, 9> normal_jacobian =
      Eigen::Matrix<double, 3, 9>::Zero();
  const Eigen::Vector3d normal = eigenvectors.col(0);
  for (int k = 1; k < 3; ++k) {
    const double denominator = eigenvalues[0] - eigenvalues[k];
    if (!std::isfinite(denominator) || denominator == 0.0) {
      return false;
    }
    const Eigen::Vector3d other = eigenvectors.col(k);
    const Eigen::Matrix3d outer = other * normal.transpose();
    Eigen::Matrix<double, 1, 9> derivative;
    derivative.head<6>() <<
        outer(0, 0), outer(1, 0) + outer(0, 1),
        outer(2, 0) + outer(0, 2), outer(1, 1),
        outer(1, 2) + outer(2, 1), outer(2, 2);
    derivative.tail<3>() =
        -(other.dot(center) * normal + normal.dot(center) * other);
    normal_jacobian += inverse_count / denominator * other * derivative;
  }

  const Eigen::Matrix<double, 3, 9> propagated =
      normal_jacobian * covariance_sum;
  plane.center = center;
  plane.normal = normal;
  plane.radius = eigenvalues[2];
  plane.covariance.setZero();
  plane.covariance.block<3, 3>(0, 0) =
      propagated * normal_jacobian.transpose();
  const Eigen::Matrix3d cross =
      inverse_count * propagated.block<3, 3>(0, 6);
  plane.covariance.block<3, 3>(0, 3) = cross;
  plane.covariance.block<3, 3>(3, 0) = cross.transpose();
  plane.covariance.block<3, 3>(3, 3) =
      inverse_count * inverse_count *
      covariance_sum.block<3, 3>(6, 6);
  // Covariance propagation is symmetric analytically; remove roundoff skew.
  plane.covariance =
      (0.5 * (plane.covariance + plane.covariance.transpose())).eval();
  return vector_is_finite(plane.center) && vector_is_finite(plane.normal) &&
         matrix_is_finite(plane.covariance) &&
         std::isfinite(plane.radius) && plane.radius >= 0.0;
}

class StaticTreeBuilder {
 public:
  StaticTreeBuilder(const SpatialMapConfig &config,
                    const SpatialMapPointVector &points,
                    StaticMapBlock &block)
      : config_(config), points_(points), block_(block) {}

  std::uint32_t build_root(const RootVoxelIndex &root_index) {
    Eigen::Vector3d center;
    const std::int64_t indices[] = {
        root_index.x, root_index.y, root_index.z};
    for (int axis = 0; axis < 3; ++axis) {
      const long double coordinate =
          (static_cast<long double>(indices[axis]) + 0.5L) *
          static_cast<long double>(config_.root_voxel_size);
      center[axis] = static_cast<double>(coordinate);
    }
    if (!vector_is_finite(center)) {
      fail("root voxel center overflowed");
      return kInvalidSpatialIndex;
    }

    std::vector<std::size_t> point_indices(points_.size());
    std::iota(point_indices.begin(), point_indices.end(), 0U);
    return build_node(point_indices, center,
                      0.5 * config_.root_voxel_size, 0);
  }

  bool failed() const { return failed_; }
  const std::string &error() const { return error_; }

 private:
  void fail(const std::string &message) {
    if (!failed_) {
      failed_ = true;
      error_ = message;
    }
  }

  bool can_append(std::size_t size, const char *kind) {
    if (size >= static_cast<std::size_t>(kInvalidSpatialIndex)) {
      fail(std::string("too many static ") + kind + " entries");
      return false;
    }
    return true;
  }

  std::uint32_t build_node(const std::vector<std::size_t> &point_indices,
                           const Eigen::Vector3d &center,
                           double half_length,
                           int layer) {
    if (failed_ ||
        point_indices.size() <=
            static_cast<std::size_t>(config_.min_points[layer])) {
      return kInvalidSpatialIndex;
    }

    Eigen::Matrix<long double, 3, 1> position_sum =
        Eigen::Matrix<long double, 3, 1>::Zero();
    Eigen::Matrix<long double, 3, 3> outer_sum =
        Eigen::Matrix<long double, 3, 3>::Zero();
    Eigen::Matrix<double, 9, 9> uncertainty_sum =
        Eigen::Matrix<double, 9, 9>::Zero();
    for (const std::size_t point_index : point_indices) {
      const SpatialMapPoint &point = points_[point_index];
      const Eigen::Matrix<long double, 3, 1> position =
          point.position.cast<long double>();
      position_sum += position;
      outer_sum += position * position.transpose();
      accumulate_bf_variance(point, uncertainty_sum);
    }
    if (!matrix_is_finite(uncertainty_sum)) {
      fail("octree uncertainty accumulation overflowed");
      return kInvalidSpatialIndex;
    }

    const long double inverse_count =
        1.0L / static_cast<long double>(point_indices.size());
    const Eigen::Matrix<long double, 3, 1> mean_long =
        position_sum * inverse_count;
    const Eigen::Matrix<long double, 3, 3> covariance_long =
        outer_sum * inverse_count - mean_long * mean_long.transpose();
    const Eigen::Vector3d mean = mean_long.cast<double>();
    const Eigen::Matrix3d covariance = covariance_long.cast<double>();
    if (!vector_is_finite(mean) || !matrix_is_finite(covariance)) {
      fail("octree statistics overflowed");
      return kInvalidSpatialIndex;
    }

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success ||
        !vector_is_finite(solver.eigenvalues())) {
      fail("octree covariance eigendecomposition failed");
      return kInvalidSpatialIndex;
    }
    const Eigen::Vector3d eigenvalues = solver.eigenvalues();
    Eigen::Matrix3d eigenvectors = solver.eigenvectors();
    canonicalize_eigenvectors(eigenvectors);
    const double ratio = eigenvalues[0] / eigenvalues[2];
    const bool planar =
        std::isfinite(ratio) &&
        eigenvalues[0] < config_.min_eigen_value &&
        ratio < config_.plane_ratio_threshold[layer];

    if (planar) {
      StaticPlane plane;
      if (make_static_plane(mean, eigenvalues, eigenvectors,
                            uncertainty_sum, point_indices.size(), plane)) {
        if (!can_append(block_.nodes.size(), "node") ||
            !can_append(block_.planes.size(), "plane")) {
          return kInvalidSpatialIndex;
        }
        StaticOctreeNode node;
        node.center = center;
        node.half_length = half_length;
        node.plane_index =
            static_cast<std::uint32_t>(block_.planes.size());
        const std::uint32_t node_index =
            static_cast<std::uint32_t>(block_.nodes.size());
        block_.nodes.push_back(node);
        block_.planes.push_back(plane);
        return node_index;
      }
    }

    if (layer >= config_.max_layer) {
      return kInvalidSpatialIndex;
    }
    if (!can_append(block_.nodes.size(), "node")) {
      return kInvalidSpatialIndex;
    }

    const std::size_t node_start = block_.nodes.size();
    const std::size_t plane_start = block_.planes.size();
    StaticOctreeNode node;
    node.center = center;
    node.half_length = half_length;
    block_.nodes.push_back(node);
    const std::uint32_t node_index = static_cast<std::uint32_t>(node_start);

    std::array<std::vector<std::size_t>, 8> child_points;
    for (const std::size_t point_index : point_indices) {
      const Eigen::Vector3d &position = points_[point_index].position;
      const int child = (position.x() > center.x() ? 4 : 0) +
                        (position.y() > center.y() ? 2 : 0) +
                        (position.z() > center.z() ? 1 : 0);
      child_points[child].push_back(point_index);
    }

    bool has_child = false;
    const double child_half_length = half_length * 0.5;
    for (int child = 0; child < 8; ++child) {
      if (child_points[child].empty()) {
        continue;
      }
      const Eigen::Vector3d child_center =
          center + Eigen::Vector3d((child & 4) ? child_half_length
                                               : -child_half_length,
                                   (child & 2) ? child_half_length
                                               : -child_half_length,
                                   (child & 1) ? child_half_length
                                               : -child_half_length);
      const std::uint32_t child_index =
          build_node(child_points[child], child_center,
                     child_half_length, layer + 1);
      if (failed_) {
        block_.nodes.resize(node_start);
        block_.planes.resize(plane_start);
        return kInvalidSpatialIndex;
      }
      if (child_index != kInvalidSpatialIndex) {
        block_.nodes[node_index].children[child] = child_index;
        has_child = true;
      }
    }

    if (!has_child) {
      block_.nodes.resize(node_start);
      block_.planes.resize(plane_start);
      return kInvalidSpatialIndex;
    }
    return node_index;
  }

  const SpatialMapConfig &config_;
  const SpatialMapPointVector &points_;
  StaticMapBlock &block_;
  bool failed_ = false;
  std::string error_;
};

}  // namespace

bool SpatialMapBuilder::build(const SpatialMapPointVector &points,
                              const SpatialMapConfig &config,
                              SpatialMapBuildResult &result,
                              std::string &error) {
  error.clear();
  std::string config_error;
  if (!config.validate(config_error)) {
    error = "invalid spatial map config: " + config_error;
    return false;
  }
  if (points.empty()) {
    error = "spatial map input must not be empty";
    return false;
  }
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (!validate_point(points[i], i, error)) {
      return false;
    }
  }

  try {
    std::map<FusionKey, SpatialMapPointVector> fusion_buckets;
    for (const SpatialMapPoint &point : points) {
      fusion_buckets[fusion_key(point, config)].push_back(point);
    }

    SpatialMapPointVector fused_points;
    fused_points.reserve(fusion_buckets.size());
    for (auto &entry : fusion_buckets) {
      SpatialMapPoint representative;
      if (!average_bucket(entry.second, representative, error)) {
        return false;
      }
      fused_points.push_back(representative);
    }

    using RootGroups =
        std::map<RootVoxelIndex, SpatialMapPointVector>;
    std::map<BlockIndex, RootGroups> grouped_points;
    for (const SpatialMapPoint &point : fused_points) {
      const RootVoxelIndex root =
          world_to_root_voxel(point.position, config);
      const BlockIndex block = root_voxel_to_block(root, config);
      grouped_points[block][root].push_back(point);
    }

    SpatialMapBuildResult candidate;
    candidate.config = config;
    candidate.source_point_count = points.size();
    candidate.fused_point_count = fused_points.size();
    candidate.blocks.reserve(grouped_points.size());
    for (const auto &block_entry : grouped_points) {
      StaticMapBlock block;
      block.index = block_entry.first;
      for (const auto &root_entry : block_entry.second) {
        StaticTreeBuilder tree_builder(config, root_entry.second, block);
        const std::uint32_t root_node_index =
            tree_builder.build_root(root_entry.first);
        if (tree_builder.failed()) {
          error = tree_builder.error();
          return false;
        }
        if (root_node_index == kInvalidSpatialIndex) {
          continue;
        }
        RootVoxelRecord root_record;
        root_record.index = root_entry.first;
        root_record.root_node_index = root_node_index;
        block.roots.push_back(root_record);
        if (config.save_display_cloud) {
          block.display_points.insert(block.display_points.end(),
                                      root_entry.second.begin(),
                                      root_entry.second.end());
        }
      }
      if (!block.roots.empty()) {
        candidate.blocks.push_back(std::move(block));
      }
    }

    result = std::move(candidate);
    return true;
  } catch (const std::exception &exception) {
    error = std::string("failed to build spatial map: ") + exception.what();
    return false;
  }
}
