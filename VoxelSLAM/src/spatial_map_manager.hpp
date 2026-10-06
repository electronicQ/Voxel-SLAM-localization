#pragma once

#include "spatial_map_io.hpp"

#include <Eigen/Core>

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

struct SpatialMapRuntimeConfig {
  double preload_radius = 15.0;
  double load_radius = 30.0;
  double unload_radius = 40.0;
  double prefetch_time_sec = 2.0;
  double map_gap_timeout_sec = 1.0;
  double preload_timeout_sec = 10.0;
  int max_background_loads = 2;

  bool validate(std::string &error) const;
};

struct SpatialMapSnapshot {
  std::uint64_t generation = 0U;
  std::unordered_map<BlockIndex, std::shared_ptr<const StaticMapBlock>,
                     BlockIndexHash>
      blocks;
};

struct SpatialMapStatus {
  std::uint64_t generation = 0U;
  bool core_ready = false;
  bool idle = true;
  bool map_gap = false;
  std::size_t core_block_count = 0U;
  std::size_t loaded_block_count = 0U;
  std::size_t pending_block_count = 0U;
  std::size_t failed_block_count = 0U;
  std::size_t failed_core_block_count = 0U;
  std::uint64_t load_sequence = 0U;
  double last_load_ms = 0.0;
  std::string last_error;
};

class SpatialBlockSource {
 public:
  virtual ~SpatialBlockSource() = default;
  virtual const SpatialMapCatalog &catalog() const = 0;
  virtual bool load(const BlockIndex &index, StaticMapBlock &block,
                    std::string &error) = 0;
};

class FileSpatialBlockSource : public SpatialBlockSource {
 public:
  FileSpatialBlockSource(std::string directory, SpatialMapCatalog catalog);
  const SpatialMapCatalog &catalog() const override;
  bool load(const BlockIndex &index, StaticMapBlock &block,
            std::string &error) override;

 private:
  std::string directory_;
  SpatialMapCatalog catalog_;
};

class SpatialMapManager {
 public:
  SpatialMapManager(std::shared_ptr<SpatialBlockSource> source,
                    const SpatialMapRuntimeConfig &config);
  ~SpatialMapManager();

  SpatialMapManager(const SpatialMapManager &) = delete;
  SpatialMapManager &operator=(const SpatialMapManager &) = delete;

  std::uint64_t begin_session();
  bool request(const Eigen::Vector3d &position,
               const Eigen::Vector3d &velocity,
               std::uint64_t generation, std::string *error = nullptr);

  std::shared_ptr<const SpatialMapSnapshot> snapshot() const;
  SpatialMapStatus status() const;
  bool wait_until_idle(std::chrono::milliseconds timeout);

 private:
  struct LoadRequest {
    BlockIndex index;
    std::uint64_t generation = 0U;
    bool core = false;
    double distance = 0.0;
  };

  void worker_loop();
  void publish_locked();
  void update_status_locked();

  std::shared_ptr<SpatialBlockSource> source_;
  SpatialMapRuntimeConfig config_;
  mutable std::mutex mutex_;
  std::condition_variable work_cv_;
  std::condition_variable idle_cv_;
  bool stop_ = false;
  std::size_t active_loads_ = 0U;
  std::uint64_t generation_ = 0U;
  std::uint64_t load_sequence_ = 0U;
  std::deque<LoadRequest> queue_;
  std::unordered_set<BlockIndex, BlockIndexHash> queued_;
  std::unordered_map<BlockIndex, std::uint64_t, BlockIndexHash>
      active_requests_;
  std::unordered_set<BlockIndex, BlockIndexHash> core_required_;
  std::unordered_set<BlockIndex, BlockIndexHash> desired_;
  std::unordered_set<BlockIndex, BlockIndexHash> failed_;
  std::unordered_map<BlockIndex, std::shared_ptr<const StaticMapBlock>,
                     BlockIndexHash>
      staging_;
  std::shared_ptr<const SpatialMapSnapshot> snapshot_;
  SpatialMapStatus status_;
  std::vector<std::thread> workers_;
};
