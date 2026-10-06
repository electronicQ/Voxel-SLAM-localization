#include "spatial_map_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace {

bool finiteNonNegative(double value) {
  return std::isfinite(value) && value >= 0.0;
}

double distanceToAabb(const Eigen::Vector3d &point,
                      const BlockAabb &bounds) {
  double squared = 0.0;
  for (int axis = 0; axis < 3; ++axis) {
    double delta = 0.0;
    if (point[axis] < bounds.minimum[axis]) {
      delta = bounds.minimum[axis] - point[axis];
    } else if (point[axis] > bounds.maximum[axis]) {
      delta = point[axis] - bounds.maximum[axis];
    }
    squared += delta * delta;
  }
  return std::sqrt(squared);
}

}  // namespace

bool SpatialMapRuntimeConfig::validate(std::string &error) const {
  error.clear();
  if (!finiteNonNegative(preload_radius) ||
      !finiteNonNegative(load_radius) ||
      !finiteNonNegative(unload_radius) ||
      !finiteNonNegative(prefetch_time_sec) ||
      !finiteNonNegative(map_gap_timeout_sec) ||
      !finiteNonNegative(preload_timeout_sec)) {
    error = "spatial map runtime values must be finite and non-negative";
    return false;
  }
  if (preload_radius > load_radius || load_radius >= unload_radius) {
    error = "spatial map radii must satisfy preload <= load < unload";
    return false;
  }
  if (max_background_loads <= 0) {
    error = "max_background_loads must be positive";
    return false;
  }
  return true;
}

FileSpatialBlockSource::FileSpatialBlockSource(
    std::string directory, SpatialMapCatalog catalog)
    : directory_(std::move(directory)), catalog_(std::move(catalog)) {}

const SpatialMapCatalog &FileSpatialBlockSource::catalog() const {
  return catalog_;
}

bool FileSpatialBlockSource::load(const BlockIndex &index,
                                  StaticMapBlock &block,
                                  std::string &error) {
  return SpatialMapIO::load_block(directory_, catalog_, index, block, error);
}

SpatialMapManager::SpatialMapManager(
    std::shared_ptr<SpatialBlockSource> source,
    const SpatialMapRuntimeConfig &config)
    : source_(std::move(source)), config_(config) {
  if (!source_) {
    throw std::invalid_argument("spatial block source is null");
  }
  std::string error;
  if (!config_.validate(error)) {
    throw std::invalid_argument(error);
  }
  std::string catalog_error;
  if (!source_->catalog().config.validate(catalog_error)) {
    throw std::invalid_argument("invalid spatial block catalog: " +
                                catalog_error);
  }
  std::shared_ptr<SpatialMapSnapshot> initial(new SpatialMapSnapshot());
  snapshot_ = initial;
  workers_.reserve(static_cast<std::size_t>(config_.max_background_loads));
  for (int i = 0; i < config_.max_background_loads; ++i) {
    workers_.emplace_back(&SpatialMapManager::worker_loop, this);
  }
}

SpatialMapManager::~SpatialMapManager() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
    queue_.clear();
    queued_.clear();
  }
  work_cv_.notify_all();
  for (std::thread &worker : workers_) {
    if (worker.joinable()) {
      worker.join();
    }
  }
}

std::uint64_t SpatialMapManager::begin_session() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (generation_ == std::numeric_limits<std::uint64_t>::max()) {
    throw std::overflow_error("spatial map session generation exhausted");
  }
  ++generation_;
  queue_.clear();
  queued_.clear();
  core_required_.clear();
  desired_.clear();
  failed_.clear();
  staging_.clear();
  status_ = SpatialMapStatus();
  status_.generation = generation_;
  status_.load_sequence = load_sequence_;
  publish_locked();
  update_status_locked();
  idle_cv_.notify_all();
  return generation_;
}

bool SpatialMapManager::request(const Eigen::Vector3d &position,
                                const Eigen::Vector3d &velocity,
                                std::uint64_t generation,
                                std::string *error) {
  if (error != nullptr) {
    error->clear();
  }
  if (!position.array().isFinite().all() ||
      !velocity.array().isFinite().all()) {
    if (error != nullptr) {
      *error = "spatial map request pose or velocity is non-finite";
    }
    return false;
  }
  const Eigen::Vector3d predicted =
      position + velocity * config_.prefetch_time_sec;
  if (!predicted.array().isFinite().all()) {
    if (error != nullptr) {
      *error = "spatial map predicted position is non-finite";
    }
    return false;
  }

  try {
    const SpatialMapCatalog &catalog = source_->catalog();
    const std::vector<BlockIndex> core_candidates =
        blocks_intersecting_sphere(position, config_.preload_radius,
                                   catalog.config);
    const std::vector<BlockIndex> current_candidates =
        blocks_intersecting_sphere(position, config_.load_radius,
                                   catalog.config);
    const std::vector<BlockIndex> predicted_candidates =
        blocks_intersecting_sphere(predicted, config_.load_radius,
                                   catalog.config);

    std::unordered_set<BlockIndex, BlockIndexHash> core;
    std::unordered_set<BlockIndex, BlockIndexHash> desired;
    auto add_if_present = [&](const BlockIndex &index,
                              std::unordered_set<BlockIndex, BlockIndexHash>
                                  &target) {
      if (catalog.lookup.find(index) != catalog.lookup.end()) {
        target.insert(index);
      }
    };
    for (const BlockIndex &index : core_candidates) {
      add_if_present(index, core);
      add_if_present(index, desired);
    }
    for (const BlockIndex &index : current_candidates) {
      add_if_present(index, desired);
    }
    for (const BlockIndex &index : predicted_candidates) {
      add_if_present(index, desired);
    }

    std::vector<LoadRequest> pending;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (generation != generation_) {
        if (error != nullptr) {
          *error = "spatial map request generation is stale";
        }
        return false;
      }
      core_required_ = std::move(core);
      desired_ = std::move(desired);
      queue_.clear();
      queued_.clear();

      bool removed = false;
      for (auto it = staging_.begin(); it != staging_.end();) {
        const BlockAabb bounds = block_aabb(it->first, catalog.config);
        if (desired_.count(it->first) == 0U &&
            !sphere_intersects_aabb(position, config_.unload_radius,
                                    bounds)) {
          it = staging_.erase(it);
          removed = true;
        } else {
          ++it;
        }
      }

      pending.reserve(desired_.size());
      for (const BlockIndex &index : desired_) {
        if (staging_.find(index) != staging_.end()) {
          continue;
        }
        if (failed_.count(index) != 0U) {
          continue;
        }
        const auto active = active_requests_.find(index);
        if (active != active_requests_.end() &&
            active->second == generation_) {
          continue;
        }
        const auto catalog_entry = catalog.lookup.find(index);
        if (catalog_entry == catalog.lookup.end() ||
            catalog_entry->second >= catalog.entries.size()) {
          continue;
        }
        LoadRequest request;
        request.index = index;
        request.generation = generation_;
        request.core = core_required_.count(index) != 0U;
        request.distance = distanceToAabb(
            position, catalog.entries[catalog_entry->second].bounds);
        pending.push_back(request);
      }
      std::sort(pending.begin(), pending.end(),
                [](const LoadRequest &left, const LoadRequest &right) {
                  if (left.core != right.core) {
                    return left.core && !right.core;
                  }
                  if (left.distance != right.distance) {
                    return left.distance < right.distance;
                  }
                  return left.index < right.index;
                });
      for (const LoadRequest &request : pending) {
        queue_.push_back(request);
        queued_.insert(request.index);
      }
      if (removed) {
        publish_locked();
      }
      update_status_locked();
      if (queue_.empty() && active_loads_ == 0U) {
        idle_cv_.notify_all();
      }
    }
    work_cv_.notify_all();
    return true;
  } catch (const std::exception &exception) {
    if (error != nullptr) {
      *error = std::string("spatial map request failed: ") + exception.what();
    }
    return false;
  }
}

std::shared_ptr<const SpatialMapSnapshot> SpatialMapManager::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_;
}

SpatialMapStatus SpatialMapManager::status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return status_;
}

bool SpatialMapManager::wait_until_idle(std::chrono::milliseconds timeout) {
  std::unique_lock<std::mutex> lock(mutex_);
  return idle_cv_.wait_for(lock, timeout, [&] {
    return queue_.empty() && active_loads_ == 0U;
  });
}

void SpatialMapManager::worker_loop() {
  while (true) {
    LoadRequest request;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      work_cv_.wait(lock, [&] { return stop_ || !queue_.empty(); });
      if (stop_) {
        return;
      }
      request = queue_.front();
      queue_.pop_front();
      queued_.erase(request.index);
      active_requests_[request.index] = request.generation;
      ++active_loads_;
      update_status_locked();
    }

    const std::chrono::steady_clock::time_point load_started =
        std::chrono::steady_clock::now();
    StaticMapBlock loaded;
    std::string load_error;
    bool success = source_->load(request.index, loaded, load_error);
    const double load_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - load_started).count();
    if (success && loaded.index != request.index) {
      success = false;
      load_error = "block source returned a different block index";
    }
    std::shared_ptr<const StaticMapBlock> immutable;
    if (success) {
      immutable.reset(new StaticMapBlock(std::move(loaded)));
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (active_loads_ != 0U) {
        --active_loads_;
      }
      const auto active = active_requests_.find(request.index);
      if (active != active_requests_.end() &&
          active->second == request.generation) {
        active_requests_.erase(active);
      }
      if (request.generation == generation_ &&
          desired_.count(request.index) != 0U) {
        if (load_sequence_ !=
            std::numeric_limits<std::uint64_t>::max()) {
          ++load_sequence_;
        }
        status_.load_sequence = load_sequence_;
        status_.last_load_ms = load_ms;
        if (success) {
          staging_[request.index] = std::move(immutable);
          publish_locked();
        } else {
          failed_.insert(request.index);
          status_.last_error = "failed loading block " +
                               std::to_string(request.index.x) + "," +
                               std::to_string(request.index.y) + "," +
                               std::to_string(request.index.z) + ": " +
                               load_error;
        }
      }
      update_status_locked();
      if (queue_.empty() && active_loads_ == 0U) {
        idle_cv_.notify_all();
      }
    }
  }
}

void SpatialMapManager::publish_locked() {
  std::shared_ptr<SpatialMapSnapshot> next(new SpatialMapSnapshot());
  next->generation = generation_;
  next->blocks = staging_;
  snapshot_ = next;
}

void SpatialMapManager::update_status_locked() {
  status_.generation = generation_;
  status_.core_block_count = core_required_.size();
  status_.loaded_block_count = staging_.size();
  status_.failed_block_count = failed_.size();
  status_.failed_core_block_count = 0U;
  for (const BlockIndex &index : core_required_) {
    if (failed_.count(index) != 0U) {
      ++status_.failed_core_block_count;
    }
  }
  std::size_t current_active_loads = 0U;
  for (const auto &active : active_requests_) {
    if (active.second == generation_) {
      ++current_active_loads;
    }
  }
  status_.pending_block_count = queue_.size() + current_active_loads;
  status_.idle = status_.pending_block_count == 0U;
  status_.core_ready = !core_required_.empty();
  for (const BlockIndex &index : core_required_) {
    if (staging_.find(index) == staging_.end()) {
      status_.core_ready = false;
      break;
    }
  }
  status_.map_gap = !status_.core_ready && status_.idle;
}
