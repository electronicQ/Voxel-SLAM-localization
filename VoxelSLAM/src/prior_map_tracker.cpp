#include "prior_map_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <limits>
#include <new>
#include <utility>

namespace {

bool fail(std::string &error, const std::string &message) {
  error = message;
  return false;
}

bool finitePose(const SavedPose &pose) {
  return std::isfinite(pose.stamp) && pose.R.array().isFinite().all() &&
         pose.t.array().isFinite().all();
}

bool representableAsFloat(const Eigen::Vector3d &translation) {
  const double limit = static_cast<double>(std::numeric_limits<float>::max());
  return (translation.array().abs() <= limit).all();
}

}  // namespace

PriorMapTracker::IndexState::IndexState()
    : xy_points(new pcl::PointCloud<pcl::PointXYZ>()),
      xyz_points(new pcl::PointCloud<pcl::PointXYZ>()) {}

bool PriorMapTracker::initialize(const SavedMap &map, std::string &error) {
  error.clear();
  if (map.keyframes.empty()) {
    return fail(error, "cannot index an empty saved map");
  }
  if (generation_ == std::numeric_limits<std::uint64_t>::max()) {
    return fail(error, "prior-map index generation is exhausted");
  }

  try {
    std::shared_ptr<IndexState> candidate(new IndexState());
    candidate->source_map = &map;
    candidate->xy_points->reserve(map.keyframes.size());
    candidate->xyz_points->reserve(map.keyframes.size());
    candidate->xy_index_to_id.reserve(map.keyframes.size());
    candidate->xyz_index_to_id.reserve(map.keyframes.size());
    candidate->xy_index_to_height.reserve(map.keyframes.size());
    std::unordered_set<int> ids;
    ids.reserve(map.keyframes.size());

    for (const SavedKeyframe &keyframe : map.keyframes) {
      if (keyframe.id < 0) {
        return fail(error, "saved keyframe ID must be non-negative");
      }
      if (!ids.insert(keyframe.id).second) {
        return fail(error, "saved map contains a duplicate keyframe ID");
      }
      if (!finitePose(keyframe.pose) ||
          !representableAsFloat(keyframe.pose.t)) {
        return fail(error, "saved keyframe pose must be finite and indexable");
      }
      if (!keyframe.cloud) {
        return fail(error, "saved keyframe cloud is null");
      }

      pcl::PointXYZ xy;
      xy.x = static_cast<float>(keyframe.pose.t.x());
      xy.y = static_cast<float>(keyframe.pose.t.y());
      xy.z = 0.0F;
      candidate->xy_points->push_back(xy);

      pcl::PointXYZ xyz = xy;
      xyz.z = static_cast<float>(keyframe.pose.t.z());
      candidate->xyz_points->push_back(xyz);

      candidate->xy_index_to_id.push_back(keyframe.id);
      candidate->xyz_index_to_id.push_back(keyframe.id);
      candidate->xy_index_to_height.push_back(keyframe.pose.t.z());
    }

    candidate->xy_points->width =
        static_cast<std::uint32_t>(candidate->xy_points->size());
    candidate->xy_points->height = 1;
    candidate->xy_points->is_dense = true;
    candidate->xyz_points->width =
        static_cast<std::uint32_t>(candidate->xyz_points->size());
    candidate->xyz_points->height = 1;
    candidate->xyz_points->is_dense = true;
    candidate->xy_index.setInputCloud(candidate->xy_points);
    candidate->xyz_index.setInputCloud(candidate->xyz_points);

    index_state_ = candidate;
    ++generation_;
    reset_runtime_cache();
    return true;
  } catch (const std::bad_alloc &) {
    return fail(error, "out of memory while building prior-map indexes");
  } catch (const std::exception &exception) {
    return fail(error, std::string("failed to build prior-map indexes: ") +
                           exception.what());
  } catch (...) {
    return fail(error, "unknown failure while building prior-map indexes");
  }
}

std::vector<int> PriorMapTracker::radius_search_xy(
    const Eigen::Vector2d &center, double radius) const {
  std::vector<int> result;
  if (!index_state_ || !center.array().isFinite().all() ||
      !std::isfinite(radius) || radius < 0.0) {
    return result;
  }

  pcl::PointXYZ query;
  query.x = static_cast<float>(center.x());
  query.y = static_cast<float>(center.y());
  query.z = 0.0F;
  if (!std::isfinite(query.x) || !std::isfinite(query.y)) {
    return result;
  }

  std::vector<int> indices;
  std::vector<float> squared_distances;
  index_state_->xy_index.radiusSearch(query, radius, indices,
                                      squared_distances);
  std::vector<std::pair<std::pair<float, int>, int>> ordered;
  ordered.reserve(indices.size());
  for (std::size_t i = 0; i < indices.size(); ++i) {
    if (indices[i] < 0 ||
        static_cast<std::size_t>(indices[i]) >=
            index_state_->xy_index_to_id.size()) {
      return std::vector<int>();
    }
    const int id = index_state_->xy_index_to_id[indices[i]];
    ordered.push_back(std::make_pair(
        std::make_pair(squared_distances[i], id), id));
  }
  std::sort(ordered.begin(), ordered.end());
  result.reserve(ordered.size());
  for (const auto &entry : ordered) {
    result.push_back(entry.second);
  }
  return result;
}

bool PriorMapTracker::nearest_height_xy(const Eigen::Vector2d &center,
                                        double &height, int &keyframe_id,
                                        std::string &error) const {
  error.clear();
  if (!index_state_) {
    return fail(error, "prior-map indexes are not initialized");
  }
  if (!center.array().isFinite().all()) {
    return fail(error, "nearest-height XY query must be finite");
  }

  pcl::PointXYZ query;
  query.x = static_cast<float>(center.x());
  query.y = static_cast<float>(center.y());
  query.z = 0.0F;
  if (!std::isfinite(query.x) || !std::isfinite(query.y)) {
    return fail(error, "nearest-height XY query is outside index range");
  }

  std::vector<int> indices(1);
  std::vector<float> squared_distances(1);
  if (index_state_->xy_index.nearestKSearch(query, 1, indices,
                                            squared_distances) != 1) {
    return fail(error, "nearest-height XY query found no keyframe");
  }
  if (indices[0] < 0 ||
      static_cast<std::size_t>(indices[0]) >=
          index_state_->xy_index_to_id.size()) {
    return fail(error, "nearest-height XY index returned an invalid result");
  }

  const std::size_t index = static_cast<std::size_t>(indices[0]);
  keyframe_id = index_state_->xy_index_to_id[index];
  height = index_state_->xy_index_to_height[index];
  return true;
}

void PriorMapTracker::reset_runtime_cache() {
  loaded_keyframe_ids_.clear();
}

const SavedMap *PriorMapTracker::source_map() const {
  return index_state_ ? index_state_->source_map : nullptr;
}

std::uint64_t PriorMapTracker::generation() const { return generation_; }
