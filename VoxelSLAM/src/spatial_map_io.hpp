#pragma once

#include "spatial_map_types.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

struct SpatialBlockIndexEntry {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  BlockIndex index;
  BlockAabb bounds;
  std::uint64_t offset = 0;
  std::uint64_t length = 0;
  std::uint32_t checksum = 0;
  std::uint32_t root_count = 0;
  std::uint32_t node_count = 0;
  std::uint32_t plane_count = 0;
  std::uint32_t display_point_count = 0;
};

using SpatialBlockIndexEntries =
    std::vector<SpatialBlockIndexEntry,
                Eigen::aligned_allocator<SpatialBlockIndexEntry>>;

struct SpatialMapCatalog {
  SpatialMapConfig config;
  SpatialBlockIndexEntries entries;
  std::unordered_map<BlockIndex, std::size_t, BlockIndexHash> lookup;
};

class SpatialMapIO {
 public:
  static bool save(const std::string &directory,
                   const std::vector<StaticMapBlock> &blocks,
                   const SpatialMapConfig &config,
                   SpatialMapCatalog &catalog, std::string &error);

  static bool load_catalog(const std::string &directory,
                           SpatialMapCatalog &catalog,
                           std::string &error);

  static bool load_block(const std::string &directory,
                         const SpatialMapCatalog &catalog,
                         const BlockIndex &index,
                         StaticMapBlock &block,
                         std::string &error);

  static bool validate_all_blocks(const std::string &directory,
                                  const SpatialMapCatalog &catalog,
                                  std::string &error);
};
