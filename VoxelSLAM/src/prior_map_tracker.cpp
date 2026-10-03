#include "prior_map_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <limits>
#include <new>
#include <unordered_set>
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

bool finitePoint(const PointType &point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z) && std::isfinite(point.intensity) &&
         std::isfinite(point.normal_x) && std::isfinite(point.normal_y) &&
         std::isfinite(point.normal_z) && std::isfinite(point.curvature);
}

bool buildWorldCloud(
    const SavedMap &map,
    const std::unordered_map<int, std::size_t> &id_to_keyframe_index,
    const std::unordered_set<int> &loaded_ids,
    pcl::PointCloud<PointType>::Ptr &cloud, std::string &error) {
  std::vector<int> ordered_ids(loaded_ids.begin(), loaded_ids.end());
  std::sort(ordered_ids.begin(), ordered_ids.end());

  pcl::PointCloud<PointType>::Ptr candidate(
      new pcl::PointCloud<PointType>());
  std::size_t point_count = 0;
  for (int id : ordered_ids) {
    const auto index = id_to_keyframe_index.find(id);
    if (index == id_to_keyframe_index.end() ||
        index->second >= map.keyframes.size()) {
      return fail(error, "loaded prior-map keyframe ID is invalid");
    }
    const SavedKeyframe &keyframe = map.keyframes[index->second];
    if (!keyframe.cloud) {
      return fail(error, "loaded prior-map keyframe cloud is null");
    }
    if (point_count > std::numeric_limits<std::size_t>::max() -
                          keyframe.cloud->size()) {
      return fail(error, "loaded prior-map point count overflowed");
    }
    point_count += keyframe.cloud->size();
  }
  candidate->reserve(point_count);

  for (int id : ordered_ids) {
    const SavedKeyframe &keyframe =
        map.keyframes.at(id_to_keyframe_index.at(id));
    for (const PointType &local_point : keyframe.cloud->points) {
      if (!finitePoint(local_point)) {
        return fail(error,
                    "loaded prior-map keyframe contains a non-finite point");
      }
      const Eigen::Vector3d local(local_point.x, local_point.y,
                                  local_point.z);
      const Eigen::Vector3d world =
          keyframe.pose.R * local + keyframe.pose.t;
      const Eigen::Matrix3d local_variance =
          Eigen::Vector3d(local_point.normal_x, local_point.normal_y,
                          local_point.normal_z)
              .asDiagonal();
      const Eigen::Matrix3d world_variance =
          keyframe.pose.R * local_variance * keyframe.pose.R.transpose();
      if (!world.array().isFinite().all() ||
          !world_variance.array().isFinite().all() ||
          !representableAsFloat(world)) {
        return fail(error,
                    "prior-map keyframe transform produced an invalid point");
      }

      PointType transformed = local_point;
      transformed.x = static_cast<float>(world.x());
      transformed.y = static_cast<float>(world.y());
      transformed.z = static_cast<float>(world.z());
      transformed.normal_x = static_cast<float>(world_variance(0, 0));
      transformed.normal_y = static_cast<float>(world_variance(1, 1));
      transformed.normal_z = static_cast<float>(world_variance(2, 2));
      candidate->push_back(transformed);
    }
  }
  candidate->width = static_cast<std::uint32_t>(candidate->size());
  candidate->height = 1;
  candidate->is_dense = true;
  cloud = candidate;
  return true;
}

void sortIds(std::vector<int> &ids) {
  std::sort(ids.begin(), ids.end());
}

}  // namespace

PriorMapTracker::PriorMapTracker()
    : loaded_world_cloud_(new pcl::PointCloud<PointType>()) {}

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
    candidate->id_to_keyframe_index.reserve(map.keyframes.size());
    std::unordered_set<int> ids;
    ids.reserve(map.keyframes.size());

    for (std::size_t keyframe_index = 0;
         keyframe_index < map.keyframes.size(); ++keyframe_index) {
      const SavedKeyframe &keyframe = map.keyframes[keyframe_index];
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
      candidate->id_to_keyframe_index.emplace(keyframe.id, keyframe_index);
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

    pcl::PointCloud<PointType>::Ptr empty_cache(
        new pcl::PointCloud<PointType>());
    empty_cache->width = 0;
    empty_cache->height = 1;
    index_state_ = candidate;
    loaded_keyframe_ids_.clear();
    loaded_world_cloud_ = empty_cache;
    ++generation_;
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
  pcl::PointCloud<PointType>::Ptr empty_cache(
      new pcl::PointCloud<PointType>());
  empty_cache->width = 0;
  empty_cache->height = 1;
  loaded_keyframe_ids_.clear();
  loaded_world_cloud_ = empty_cache;
}

const SavedMap *PriorMapTracker::source_map() const {
  return index_state_ ? index_state_->source_map : nullptr;
}

std::uint64_t PriorMapTracker::generation() const { return generation_; }

bool PriorMapTracker::rebuild_local_map(
    const Eigen::Vector3d &center, double radius, int preload_limit,
    PriorMapCacheUpdate &update, std::string &error) {
  error.clear();
  if (!index_state_) {
    return fail(error, "prior-map indexes are not initialized");
  }
  if (!center.array().isFinite().all() || !std::isfinite(radius) ||
      radius <= 0.0 || preload_limit <= 0) {
    return fail(error, "prior-map rebuild radius or preload limit is invalid");
  }

  try {
    pcl::PointXYZ query;
    query.x = static_cast<float>(center.x());
    query.y = static_cast<float>(center.y());
    query.z = static_cast<float>(center.z());
    if (!std::isfinite(query.x) || !std::isfinite(query.y) ||
        !std::isfinite(query.z)) {
      return fail(error, "prior-map rebuild center is outside index range");
    }

    std::vector<int> indices;
    std::vector<float> squared_distances;
    index_state_->xyz_index.radiusSearch(query, radius, indices,
                                         squared_distances);
    std::vector<std::pair<std::pair<float, int>, int>> ordered;
    ordered.reserve(indices.size());
    for (std::size_t i = 0; i < indices.size(); ++i) {
      if (indices[i] < 0 ||
          static_cast<std::size_t>(indices[i]) >=
              index_state_->xyz_index_to_id.size()) {
        return fail(error, "prior-map XYZ index returned an invalid result");
      }
      const int id = index_state_->xyz_index_to_id[indices[i]];
      ordered.push_back(
          std::make_pair(std::make_pair(squared_distances[i], id), id));
    }
    std::sort(ordered.begin(), ordered.end());

    std::unordered_set<int> candidate_ids;
    const std::size_t load_count = std::min(
        ordered.size(), static_cast<std::size_t>(preload_limit));
    for (std::size_t i = 0; i < load_count; ++i) {
      candidate_ids.insert(ordered[i].second);
    }

    pcl::PointCloud<PointType>::Ptr candidate_cloud;
    if (!buildWorldCloud(*index_state_->source_map,
                         index_state_->id_to_keyframe_index, candidate_ids,
                         candidate_cloud, error)) {
      return false;
    }

    PriorMapCacheUpdate candidate_update;
    for (int id : candidate_ids) {
      if (loaded_keyframe_ids_.find(id) == loaded_keyframe_ids_.end()) {
        candidate_update.loaded_ids.push_back(id);
      }
    }
    for (int id : loaded_keyframe_ids_) {
      if (candidate_ids.find(id) == candidate_ids.end()) {
        candidate_update.unloaded_ids.push_back(id);
      }
    }
    candidate_update.active_ids.assign(candidate_ids.begin(),
                                       candidate_ids.end());
    sortIds(candidate_update.loaded_ids);
    sortIds(candidate_update.unloaded_ids);
    sortIds(candidate_update.active_ids);
    candidate_update.world_cloud = candidate_cloud;

    loaded_keyframe_ids_ = std::move(candidate_ids);
    loaded_world_cloud_ = std::move(candidate_cloud);
    update = std::move(candidate_update);
    return true;
  } catch (const std::bad_alloc &) {
    return fail(error, "out of memory while rebuilding prior local map");
  } catch (const std::exception &exception) {
    return fail(error, std::string("failed to rebuild prior local map: ") +
                           exception.what());
  } catch (...) {
    return fail(error, "unknown failure while rebuilding prior local map");
  }
}

bool PriorMapTracker::update_loaded_keyframes(
    const Eigen::Vector3d &center, double load_radius, double unload_radius,
    int max_load_per_scan, PriorMapCacheUpdate &update, std::string &error) {
  error.clear();
  if (!index_state_) {
    return fail(error, "prior-map indexes are not initialized");
  }
  if (!center.array().isFinite().all() || !std::isfinite(load_radius) ||
      !std::isfinite(unload_radius) || load_radius <= 0.0 ||
      unload_radius < load_radius || max_load_per_scan < 0) {
    return fail(error, "prior-map load/unload radius or load limit is invalid");
  }

  try {
    pcl::PointXYZ query;
    query.x = static_cast<float>(center.x());
    query.y = static_cast<float>(center.y());
    query.z = static_cast<float>(center.z());
    if (!std::isfinite(query.x) || !std::isfinite(query.y) ||
        !std::isfinite(query.z)) {
      return fail(error, "prior-map cache center is outside index range");
    }

    std::unordered_set<int> candidate_ids = loaded_keyframe_ids_;
    std::vector<int> unloaded;
    const double unload_squared = unload_radius * unload_radius;
    for (auto iterator = candidate_ids.begin();
         iterator != candidate_ids.end();) {
      const auto index = index_state_->id_to_keyframe_index.find(*iterator);
      if (index == index_state_->id_to_keyframe_index.end()) {
        return fail(error, "loaded prior-map keyframe ID is invalid");
      }
      const Eigen::Vector3d delta =
          index_state_->source_map->keyframes[index->second].pose.t - center;
      if (delta.squaredNorm() > unload_squared) {
        unloaded.push_back(*iterator);
        iterator = candidate_ids.erase(iterator);
      } else {
        ++iterator;
      }
    }

    std::vector<int> indices;
    std::vector<float> squared_distances;
    index_state_->xyz_index.radiusSearch(query, load_radius, indices,
                                         squared_distances);
    std::vector<std::pair<std::pair<float, int>, int>> ordered;
    ordered.reserve(indices.size());
    for (std::size_t i = 0; i < indices.size(); ++i) {
      if (indices[i] < 0 ||
          static_cast<std::size_t>(indices[i]) >=
              index_state_->xyz_index_to_id.size()) {
        return fail(error, "prior-map XYZ index returned an invalid result");
      }
      const int id = index_state_->xyz_index_to_id[indices[i]];
      ordered.push_back(
          std::make_pair(std::make_pair(squared_distances[i], id), id));
    }
    std::sort(ordered.begin(), ordered.end());

    std::vector<int> loaded;
    for (const auto &entry : ordered) {
      if (loaded.size() >= static_cast<std::size_t>(max_load_per_scan)) {
        break;
      }
      if (candidate_ids.insert(entry.second).second) {
        loaded.push_back(entry.second);
      }
    }

    const bool changed = !loaded.empty() || !unloaded.empty();
    pcl::PointCloud<PointType>::Ptr candidate_cloud = loaded_world_cloud_;
    if (changed &&
        !buildWorldCloud(*index_state_->source_map,
                         index_state_->id_to_keyframe_index, candidate_ids,
                         candidate_cloud, error)) {
      return false;
    }

    PriorMapCacheUpdate candidate_update;
    candidate_update.loaded_ids = std::move(loaded);
    candidate_update.unloaded_ids = std::move(unloaded);
    candidate_update.active_ids.assign(candidate_ids.begin(),
                                       candidate_ids.end());
    sortIds(candidate_update.loaded_ids);
    sortIds(candidate_update.unloaded_ids);
    sortIds(candidate_update.active_ids);
    candidate_update.world_cloud = candidate_cloud;

    loaded_keyframe_ids_ = std::move(candidate_ids);
    loaded_world_cloud_ = std::move(candidate_cloud);
    update = std::move(candidate_update);
    return true;
  } catch (const std::bad_alloc &) {
    return fail(error, "out of memory while updating prior local map");
  } catch (const std::exception &exception) {
    return fail(error, std::string("failed to update prior local map: ") +
                           exception.what());
  } catch (...) {
    return fail(error, "unknown failure while updating prior local map");
  }
}

std::vector<int> PriorMapTracker::loaded_keyframe_ids() const {
  std::vector<int> ids(loaded_keyframe_ids_.begin(),
                       loaded_keyframe_ids_.end());
  sortIds(ids);
  return ids;
}

pcl::PointCloud<PointType>::ConstPtr PriorMapTracker::loaded_world_cloud()
    const {
  return loaded_world_cloud_;
}
