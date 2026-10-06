#include "spatial_map_manager.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <map>
#include <mutex>
#include <set>
#include <thread>

namespace {

class ControllableBlockSource : public SpatialBlockSource {
 public:
  explicit ControllableBlockSource(const SpatialMapCatalog &catalog)
      : catalog_(catalog) {}

  const SpatialMapCatalog &catalog() const override { return catalog_; }

  bool load(const BlockIndex &index, StaticMapBlock &block,
            std::string &error) override {
    std::unique_lock<std::mutex> lock(mutex_);
    requested_.insert(index);
    ++load_counts_[index];
    requested_cv_.notify_all();
    release_cv_.wait(lock, [&] { return released_ || stopped_; });
    if (stopped_) {
      error = "source stopped";
      return false;
    }
    if (failed_.count(index) != 0U) {
      error = "injected corrupt block";
      return false;
    }
    block.index = index;
    RootVoxelRecord root;
    root.index = RootVoxelIndex(index.x * 20, index.y * 20, index.z * 10);
    root.root_node_index = 0U;
    block.roots.push_back(root);
    StaticOctreeNode node;
    node.center = Eigen::Vector3d(root.index.x + 0.5,
                                  root.index.y + 0.5,
                                  root.index.z + 0.5);
    node.half_length = 0.5;
    block.nodes.push_back(node);
    return true;
  }

  void release_all() {
    std::lock_guard<std::mutex> lock(mutex_);
    released_ = true;
    release_cv_.notify_all();
  }

  void fail(const BlockIndex &index) { failed_.insert(index); }

  void stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    stopped_ = true;
    release_cv_.notify_all();
  }

  int load_count(const BlockIndex &index) {
    std::lock_guard<std::mutex> lock(mutex_);
    return load_counts_[index];
  }

 private:
  SpatialMapCatalog catalog_;
  std::mutex mutex_;
  std::condition_variable requested_cv_;
  std::condition_variable release_cv_;
  std::set<BlockIndex> requested_;
  std::map<BlockIndex, int> load_counts_;
  std::set<BlockIndex> failed_;
  bool released_ = false;
  bool stopped_ = false;
};

SpatialMapCatalog catalogFixture() {
  SpatialMapCatalog catalog;
  for (int x = -1; x <= 6; ++x) {
    SpatialBlockIndexEntry entry;
    entry.index = BlockIndex(x, 0, 0);
    entry.bounds = block_aabb(entry.index, catalog.config);
    catalog.lookup.emplace(entry.index, catalog.entries.size());
    catalog.entries.push_back(entry);
  }
  return catalog;
}

SpatialMapRuntimeConfig runtimeConfig() {
  SpatialMapRuntimeConfig config;
  config.preload_radius = 15.0;
  config.load_radius = 30.0;
  config.unload_radius = 40.0;
  config.prefetch_time_sec = 2.0;
  config.max_background_loads = 2;
  return config;
}

const std::chrono::milliseconds kTimeout(2000);

TEST(SpatialMapManager, LoadsCoreThenPrefetchesAndUnloadsWithHysteresis) {
  auto source = std::make_shared<ControllableBlockSource>(catalogFixture());
  SpatialMapManager manager(source, runtimeConfig());
  const std::uint64_t generation = manager.begin_session();
  manager.request(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                  generation);
  source->release_all();
  ASSERT_TRUE(manager.wait_until_idle(kTimeout));
  EXPECT_TRUE(manager.status().core_ready);
  const auto first = manager.snapshot();
  ASSERT_FALSE(first->blocks.empty());

  manager.request(Eigen::Vector3d(100.0, 0.0, 0.0),
                  Eigen::Vector3d::Zero(), generation);
  ASSERT_TRUE(manager.wait_until_idle(kTimeout));
  const auto second = manager.snapshot();
  EXPECT_NE(second.get(), first.get());
  EXPECT_EQ(second->blocks.count(BlockIndex(0, 0, 0)), 0U);
  EXPECT_GT(second->blocks.count(BlockIndex(5, 0, 0)), 0U);
}

TEST(SpatialMapManager, DiscardsLoadsFromOldSessionGeneration) {
  auto source = std::make_shared<ControllableBlockSource>(catalogFixture());
  SpatialMapManager manager(source, runtimeConfig());
  const std::uint64_t old_generation = manager.begin_session();
  manager.request(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                  old_generation);
  const std::uint64_t new_generation = manager.begin_session();
  source->release_all();
  ASSERT_TRUE(manager.wait_until_idle(kTimeout));
  EXPECT_EQ(manager.status().generation, new_generation);
  EXPECT_TRUE(manager.snapshot()->blocks.empty());
}

TEST(SpatialMapManager, ReportsLoadFailureWithoutPublishingCorruptBlock) {
  auto source = std::make_shared<ControllableBlockSource>(catalogFixture());
  source->fail(BlockIndex(0, 0, 0));
  source->release_all();
  SpatialMapManager manager(source, runtimeConfig());
  const std::uint64_t generation = manager.begin_session();
  manager.request(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                  generation);
  ASSERT_TRUE(manager.wait_until_idle(kTimeout));
  const SpatialMapStatus status = manager.status();
  EXPECT_FALSE(status.last_error.empty());
  EXPECT_FALSE(status.core_ready);
  EXPECT_EQ(manager.snapshot()->blocks.count(BlockIndex(0, 0, 0)), 0U);
  EXPECT_GT(status.failed_core_block_count, 0U);
  ASSERT_TRUE(manager.request(Eigen::Vector3d::Zero(),
                              Eigen::Vector3d::Zero(), generation));
  ASSERT_TRUE(manager.wait_until_idle(kTimeout));
  EXPECT_EQ(source->load_count(BlockIndex(0, 0, 0)), 1);
}

TEST(SpatialMapManager, ConcurrentSnapshotsRemainImmutable) {
  auto source = std::make_shared<ControllableBlockSource>(catalogFixture());
  source->release_all();
  SpatialMapManager manager(source, runtimeConfig());
  const std::uint64_t generation = manager.begin_session();
  const auto before = manager.snapshot();
  std::thread requester([&] {
    manager.request(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                    generation);
  });
  for (int i = 0; i < 100; ++i) {
    const auto current = manager.snapshot();
    ASSERT_TRUE(current);
    EXPECT_EQ(before->blocks.size(), 0U);
  }
  requester.join();
  ASSERT_TRUE(manager.wait_until_idle(kTimeout));
  EXPECT_FALSE(manager.snapshot()->blocks.empty());
  EXPECT_TRUE(before->blocks.empty());
}

TEST(SpatialMapManager, RepeatedRequestsDoNotDuplicateActiveLoads) {
  auto source = std::make_shared<ControllableBlockSource>(catalogFixture());
  SpatialMapManager manager(source, runtimeConfig());
  const std::uint64_t generation = manager.begin_session();
  ASSERT_TRUE(manager.request(Eigen::Vector3d::Zero(),
                              Eigen::Vector3d::Zero(), generation));
  for (int i = 0; i < 20; ++i) {
    ASSERT_TRUE(manager.request(Eigen::Vector3d::Zero(),
                                Eigen::Vector3d::Zero(), generation));
  }
  source->release_all();
  ASSERT_TRUE(manager.wait_until_idle(kTimeout));
  for (const auto &entry : manager.snapshot()->blocks) {
    EXPECT_EQ(source->load_count(entry.first), 1);
  }
}

TEST(SpatialMapManager, KeepsDesiredPrefetchBlocksOutsideCurrentUnloadRadius) {
  auto source = std::make_shared<ControllableBlockSource>(catalogFixture());
  source->release_all();
  SpatialMapManager manager(source, runtimeConfig());
  const std::uint64_t generation = manager.begin_session();
  const Eigen::Vector3d velocity(50.0, 0.0, 0.0);
  ASSERT_TRUE(manager.request(Eigen::Vector3d::Zero(), velocity, generation));
  ASSERT_TRUE(manager.wait_until_idle(kTimeout));
  ASSERT_GT(manager.snapshot()->blocks.count(BlockIndex(5, 0, 0)), 0U);
  ASSERT_TRUE(manager.request(Eigen::Vector3d::Zero(), velocity, generation));
  ASSERT_TRUE(manager.wait_until_idle(kTimeout));
  EXPECT_EQ(source->load_count(BlockIndex(5, 0, 0)), 1);
}

TEST(SpatialMapRuntimeConfig, RejectsInvalidRadiusOrdering) {
  SpatialMapRuntimeConfig config = runtimeConfig();
  config.unload_radius = config.load_radius - 1.0;
  std::string error;
  EXPECT_FALSE(config.validate(error));
}

}  // namespace
