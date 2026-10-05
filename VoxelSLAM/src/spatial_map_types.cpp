#include "spatial_map_types.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace {

bool vector_is_finite(const Eigen::Vector3d &value) {
  return std::isfinite(value.x()) && std::isfinite(value.y()) &&
         std::isfinite(value.z());
}

bool valid_positive(double value) {
  return std::isfinite(value) && value > 0.0;
}

constexpr double kMaxSafeRootVoxelsPerBlock = 9007199254740992.0;

double bounded_ulp_tolerance(double value) {
  const double lower = std::nextafter(
      value, -std::numeric_limits<double>::infinity());
  const double upper = std::nextafter(
      value, std::numeric_limits<double>::infinity());
  const double ulp = std::max(value - lower, upper - value);
  return std::min(1e-9, 4.0 * ulp);
}

bool matches_with_bounded_ulp(double left, double right) {
  if (!std::isfinite(left) || !std::isfinite(right)) {
    return false;
  }
  const double tolerance =
      std::max(bounded_ulp_tolerance(left),
               bounded_ulp_tolerance(right));
  return std::abs(left - right) <= tolerance;
}

enum class RootVoxelRatioStatus { kValid, kNotIntegral, kUnsafeCount };

RootVoxelRatioStatus classify_root_voxel_ratio(
    double size, double root_voxel_size, std::int64_t &count) {
  const double quotient = size / root_voxel_size;
  if (!std::isfinite(quotient) || quotient < 1.0) {
    return RootVoxelRatioStatus::kNotIntegral;
  }
  if (quotient > kMaxSafeRootVoxelsPerBlock) {
    return RootVoxelRatioStatus::kUnsafeCount;
  }

  const double nearest = std::round(quotient);
  if (!matches_with_bounded_ulp(quotient, nearest)) {
    return RootVoxelRatioStatus::kNotIntegral;
  }

  count = static_cast<std::int64_t>(nearest);
  const double canonical_size =
      static_cast<double>(count) * root_voxel_size;
  if (!matches_with_bounded_ulp(canonical_size, size)) {
    return RootVoxelRatioStatus::kNotIntegral;
  }
  return RootVoxelRatioStatus::kValid;
}

void require_valid_config(const SpatialMapConfig &config) {
  std::string error;
  if (!config.validate(error)) {
    throw std::invalid_argument("invalid spatial map config: " + error);
  }
}

std::int64_t root_voxels_per_block(double block_size,
                                   double root_voxel_size) {
  std::int64_t count = 0;
  if (classify_root_voxel_ratio(block_size, root_voxel_size, count) !=
      RootVoxelRatioStatus::kValid) {
    throw std::invalid_argument("invalid block-to-root-voxel ratio");
  }
  return count;
}

double canonical_block_size(double block_size, double root_voxel_size) {
  return static_cast<double>(
             root_voxels_per_block(block_size, root_voxel_size)) *
         root_voxel_size;
}

Eigen::Vector3d canonical_block_sizes(const SpatialMapConfig &config) {
  return Eigen::Vector3d(
      canonical_block_size(config.size_x, config.root_voxel_size),
      canonical_block_size(config.size_y, config.root_voxel_size),
      canonical_block_size(config.size_z, config.root_voxel_size));
}

std::int64_t floor_integer_divide(std::int64_t value,
                                  std::int64_t divisor) {
  std::int64_t quotient = value / divisor;
  if (value % divisor < 0) {
    --quotient;
  }
  return quotient;
}

std::size_t stable_index_hash(std::int64_t x, std::int64_t y,
                              std::int64_t z) noexcept {
  std::uint64_t hash = UINT64_C(14695981039346656037);
  const std::int64_t values[] = {x, y, z};
  for (const std::int64_t value : values) {
    const std::uint64_t bits = static_cast<std::uint64_t>(value);
    for (unsigned int shift = 0; shift < 64; shift += 8) {
      hash ^= (bits >> shift) & UINT64_C(0xff);
      hash *= UINT64_C(1099511628211);
    }
  }
  return static_cast<std::size_t>(hash);
}

void require_valid_sphere(const Eigen::Vector3d &center, double radius) {
  if (!vector_is_finite(center)) {
    throw std::invalid_argument("sphere center coordinates must be finite");
  }
  if (!std::isfinite(radius) || radius < 0.0) {
    throw std::invalid_argument(
        "sphere radius must be finite and non-negative");
  }
}

void require_valid_aabb(const BlockAabb &aabb) {
  if (!vector_is_finite(aabb.minimum) ||
      !vector_is_finite(aabb.maximum)) {
    throw std::invalid_argument("AABB coordinates must be finite");
  }
  if ((aabb.minimum.array() > aabb.maximum.array()).any()) {
    throw std::invalid_argument("AABB minimum must not exceed maximum");
  }
}

double checked_offset(double coordinate, double offset) {
  const double result = coordinate + offset;
  if (!std::isfinite(result)) {
    throw std::overflow_error("sphere bounding coordinates overflow");
  }
  return result;
}

std::int64_t preceding_index(std::int64_t index) {
  if (index == std::numeric_limits<std::int64_t>::min()) {
    return index;
  }
  return index - 1;
}

}  // namespace

bool operator==(const BlockIndex &left, const BlockIndex &right) {
  return left.x == right.x && left.y == right.y && left.z == right.z;
}

bool operator!=(const BlockIndex &left, const BlockIndex &right) {
  return !(left == right);
}

bool operator<(const BlockIndex &left, const BlockIndex &right) {
  if (left.x != right.x) {
    return left.x < right.x;
  }
  if (left.y != right.y) {
    return left.y < right.y;
  }
  return left.z < right.z;
}

std::size_t BlockIndexHash::operator()(const BlockIndex &index) const noexcept {
  return stable_index_hash(index.x, index.y, index.z);
}

bool operator==(const RootVoxelIndex &left, const RootVoxelIndex &right) {
  return left.x == right.x && left.y == right.y && left.z == right.z;
}

bool operator!=(const RootVoxelIndex &left, const RootVoxelIndex &right) {
  return !(left == right);
}

bool operator<(const RootVoxelIndex &left, const RootVoxelIndex &right) {
  if (left.x != right.x) {
    return left.x < right.x;
  }
  if (left.y != right.y) {
    return left.y < right.y;
  }
  return left.z < right.z;
}

std::size_t RootVoxelIndexHash::operator()(
    const RootVoxelIndex &index) const noexcept {
  return stable_index_hash(index.x, index.y, index.z);
}

bool SpatialMapConfig::validate(std::string &error) const {
  error.clear();
  const struct {
    const char *name;
    double value;
  } positive_values[] = {
      {"size_x", size_x},
      {"size_y", size_y},
      {"size_z", size_z},
      {"fusion_leaf_size", fusion_leaf_size},
      {"root_voxel_size", root_voxel_size},
  };

  for (const auto &entry : positive_values) {
    if (!std::isfinite(entry.value)) {
      error = std::string(entry.name) + " must be finite";
      return false;
    }
    if (entry.value <= 0.0) {
      error = std::string(entry.name) + " must be positive";
      return false;
    }
  }

  const double block_sizes[] = {size_x, size_y, size_z};
  for (const double block_size : block_sizes) {
    std::int64_t root_voxel_count = 0;
    const RootVoxelRatioStatus status = classify_root_voxel_ratio(
        block_size, root_voxel_size, root_voxel_count);
    if (status == RootVoxelRatioStatus::kUnsafeCount) {
      error = "each block size must be an integer multiple of "
              "root_voxel_size with no more than 2^53 root voxels";
      return false;
    }
    if (status != RootVoxelRatioStatus::kValid) {
      error =
          "each block size must be an integer multiple of root_voxel_size";
      return false;
    }
  }

  if (max_layer < 0 ||
      max_layer >= static_cast<int>(min_points.size()) ||
      max_layer >= static_cast<int>(plane_ratio_threshold.size())) {
    error = "max_layer must be within the per-layer array range";
    return false;
  }

  if (!std::isfinite(min_eigen_value)) {
    error = "min_eigen_value must be finite";
    return false;
  }
  if (min_eigen_value < 0.0) {
    error = "min_eigen_value must be non-negative";
    return false;
  }

  for (std::size_t i = 0; i < min_points.size(); ++i) {
    if (min_points[i] <= 0) {
      error = "min_points values must be positive";
      return false;
    }
  }

  for (std::size_t i = 0; i < plane_ratio_threshold.size(); ++i) {
    if (!std::isfinite(plane_ratio_threshold[i])) {
      error = "plane_ratio_threshold values must be finite";
      return false;
    }
    if (plane_ratio_threshold[i] < 0.0) {
      error = "plane_ratio_threshold values must be non-negative";
      return false;
    }
  }

  return true;
}

std::int64_t floor_grid_index(double coordinate, double resolution) {
  if (!std::isfinite(coordinate)) {
    throw std::invalid_argument("grid coordinate must be finite");
  }
  if (!valid_positive(resolution)) {
    throw std::invalid_argument(
        "grid resolution must be finite and positive");
  }

  const long double quotient =
      static_cast<long double>(coordinate) /
      static_cast<long double>(resolution);
  const long double floored = std::floor(quotient);
  const long double int64_limit = std::ldexp(1.0L, 63);
  if (floored < -int64_limit || floored >= int64_limit) {
    throw std::overflow_error("grid index exceeds signed 64-bit range");
  }
  return static_cast<std::int64_t>(floored);
}

BlockIndex world_to_block(const Eigen::Vector3d &point,
                          const SpatialMapConfig &config) {
  return root_voxel_to_block(world_to_root_voxel(point, config), config);
}

RootVoxelIndex world_to_root_voxel(const Eigen::Vector3d &point,
                                   const SpatialMapConfig &config) {
  require_valid_config(config);
  if (!vector_is_finite(point)) {
    throw std::invalid_argument("world coordinates must be finite");
  }
  return RootVoxelIndex(
      floor_grid_index(point.x(), config.root_voxel_size),
      floor_grid_index(point.y(), config.root_voxel_size),
      floor_grid_index(point.z(), config.root_voxel_size));
}

BlockIndex root_voxel_to_block(const RootVoxelIndex &index,
                               const SpatialMapConfig &config) {
  require_valid_config(config);
  const std::int64_t count_x =
      root_voxels_per_block(config.size_x, config.root_voxel_size);
  const std::int64_t count_y =
      root_voxels_per_block(config.size_y, config.root_voxel_size);
  const std::int64_t count_z =
      root_voxels_per_block(config.size_z, config.root_voxel_size);
  return BlockIndex(floor_integer_divide(index.x, count_x),
                    floor_integer_divide(index.y, count_y),
                    floor_integer_divide(index.z, count_z));
}

BlockAabb block_aabb(const BlockIndex &index,
                     const SpatialMapConfig &config) {
  require_valid_config(config);
  const Eigen::Vector3d block_sizes = canonical_block_sizes(config);
  BlockAabb result;
  result.minimum = Eigen::Vector3d(
      static_cast<double>(index.x) * block_sizes.x(),
      static_cast<double>(index.y) * block_sizes.y(),
      static_cast<double>(index.z) * block_sizes.z());
  result.maximum = result.minimum + block_sizes;
  if (!vector_is_finite(result.minimum) ||
      !vector_is_finite(result.maximum)) {
    throw std::overflow_error("block AABB coordinates overflow");
  }
  if ((result.maximum.array() <= result.minimum.array()).any()) {
    throw std::overflow_error(
        "block AABB cannot represent distinct adjacent boundaries");
  }
  return result;
}

bool sphere_intersects_aabb(const Eigen::Vector3d &center, double radius,
                            const BlockAabb &aabb) {
  require_valid_sphere(center, radius);
  require_valid_aabb(aabb);

  Eigen::Vector3d distance = Eigen::Vector3d::Zero();
  for (int axis = 0; axis < 3; ++axis) {
    if (center[axis] < aabb.minimum[axis]) {
      distance[axis] = aabb.minimum[axis] - center[axis];
    } else if (center[axis] > aabb.maximum[axis]) {
      distance[axis] = center[axis] - aabb.maximum[axis];
    }
  }
  return std::hypot(std::hypot(distance.x(), distance.y()), distance.z()) <=
         radius;
}

std::vector<BlockIndex> blocks_intersecting_sphere(
    const Eigen::Vector3d &center, double radius,
    const SpatialMapConfig &config) {
  require_valid_config(config);
  require_valid_sphere(center, radius);
  const Eigen::Vector3d block_sizes = canonical_block_sizes(config);

  const Eigen::Vector3d lower(
      checked_offset(center.x(), -radius),
      checked_offset(center.y(), -radius),
      checked_offset(center.z(), -radius));
  const Eigen::Vector3d upper(
      checked_offset(center.x(), radius),
      checked_offset(center.y(), radius),
      checked_offset(center.z(), radius));

  const BlockIndex first(
      preceding_index(floor_grid_index(lower.x(), block_sizes.x())),
      preceding_index(floor_grid_index(lower.y(), block_sizes.y())),
      preceding_index(floor_grid_index(lower.z(), block_sizes.z())));
  const BlockIndex last(floor_grid_index(upper.x(), block_sizes.x()),
                        floor_grid_index(upper.y(), block_sizes.y()),
                        floor_grid_index(upper.z(), block_sizes.z()));

  std::vector<BlockIndex> result;
  for (std::int64_t x = first.x;; ++x) {
    for (std::int64_t y = first.y;; ++y) {
      for (std::int64_t z = first.z;; ++z) {
        const BlockIndex candidate(x, y, z);
        if (sphere_intersects_aabb(center, radius,
                                   block_aabb(candidate, config))) {
          result.push_back(candidate);
        }
        if (z == last.z) {
          break;
        }
      }
      if (y == last.y) {
        break;
      }
    }
    if (x == last.x) {
      break;
    }
  }

  std::sort(result.begin(), result.end());
  return result;
}
