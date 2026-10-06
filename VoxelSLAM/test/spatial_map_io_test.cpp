#include "spatial_map_io.hpp"

#include <gtest/gtest.h>

#include <boost/filesystem.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <fstream>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

namespace {

namespace fs = boost::filesystem;

constexpr std::size_t kIndexHeaderSize = 144U;
constexpr std::size_t kIndexRecordSize = 108U;
constexpr std::size_t kIndexOffsetField = 72U;
constexpr std::size_t kIndexLengthField = 80U;
constexpr std::size_t kIndexChecksumField = 88U;
constexpr std::size_t kIndexRootCountField = 92U;
constexpr std::size_t kDataHeaderSize = 24U;
constexpr std::size_t kPayloadRootCountField = 24U;
constexpr std::size_t kPayloadRoots = 40U;
constexpr std::size_t kRootRecordSize = 28U;
constexpr std::size_t kNodeChildrenField = 32U;
constexpr std::size_t kNodePlaneField = 64U;

class TemporaryDirectory {
 public:
  TemporaryDirectory()
      : path_(fs::temp_directory_path() /
              fs::unique_path("voxel-slam-spatial-io-%%%%-%%%%-%%%%")) {
    fs::create_directories(path_);
  }

  ~TemporaryDirectory() {
    boost::system::error_code ignored;
    fs::remove_all(path_, ignored);
  }

  std::string path() const { return path_.string(); }
  fs::path index_path() const { return path_ / "spatial_blocks.idx"; }
  fs::path data_path() const { return path_ / "spatial_blocks.bin"; }

 private:
  fs::path path_;
};

SpatialMapPoint display_point(double x, double y, double z,
                              float intensity) {
  SpatialMapPoint point;
  point.position = Eigen::Vector3d(x, y, z);
  point.covariance = Eigen::Matrix3d::Identity() * 0.0125;
  point.covariance(0, 1) = 0.001;
  point.covariance(1, 0) = 0.001;
  point.intensity = intensity;
  return point;
}

StaticMapBlock make_block(const BlockIndex &block_index,
                          const RootVoxelIndex &root_index,
                          bool with_display) {
  StaticMapBlock block;
  block.index = block_index;

  RootVoxelRecord root;
  root.index = root_index;
  root.root_node_index = 0U;
  block.roots.push_back(root);

  StaticOctreeNode node;
  node.center = Eigen::Vector3d(
      static_cast<double>(root_index.x) + 0.5,
      static_cast<double>(root_index.y) + 0.5,
      static_cast<double>(root_index.z) + 0.5);
  node.half_length = 0.5;
  node.plane_index = 0U;
  block.nodes.push_back(node);

  StaticPlane plane;
  plane.center = node.center + Eigen::Vector3d(0.1, -0.2, 0.05);
  plane.normal = Eigen::Vector3d(0.0, 0.0, 1.0);
  plane.covariance = Eigen::Matrix<double, 6, 6>::Identity() * 0.025;
  plane.covariance(0, 5) = -0.002;
  plane.covariance(5, 0) = -0.002;
  plane.radius = 0.75;
  block.planes.push_back(plane);

  if (with_display) {
    block.display_points.push_back(display_point(
        node.center.x(), node.center.y(), node.center.z(), 17.5F));
    block.display_points.push_back(display_point(
        node.center.x() + 0.1, node.center.y() + 0.2,
        node.center.z() + 0.3, -2.25F));
  }
  return block;
}

std::vector<StaticMapBlock> two_block_fixture() {
  std::vector<StaticMapBlock> blocks;
  blocks.push_back(make_block(BlockIndex(1, -2, 0),
                              RootVoxelIndex(20, -40, 0), true));
  blocks.push_back(make_block(BlockIndex(-1, 0, -1),
                              RootVoxelIndex(-1, 0, -1), false));
  return blocks;
}

SpatialMapConfig test_config() {
  SpatialMapConfig config;
  config.fusion_leaf_size = 0.2;
  config.max_layer = 3;
  config.min_eigen_value = 0.00125;
  config.min_points = {{3, 4, 5, 6}};
  config.plane_ratio_threshold = {{0.1, 0.2, 0.3, 0.4}};
  config.save_display_cloud = true;
  return config;
}

bool points_equal(const SpatialMapPoint &left,
                  const SpatialMapPoint &right) {
  return (left.position.array() == right.position.array()).all() &&
         (left.covariance.array() == right.covariance.array()).all() &&
         left.intensity == right.intensity;
}

bool blocks_equal(const StaticMapBlock &left,
                  const StaticMapBlock &right) {
  if (left.index != right.index || left.roots.size() != right.roots.size() ||
      left.nodes.size() != right.nodes.size() ||
      left.planes.size() != right.planes.size() ||
      left.display_points.size() != right.display_points.size()) {
    return false;
  }
  for (std::size_t i = 0; i < left.roots.size(); ++i) {
    if (left.roots[i].index != right.roots[i].index ||
        left.roots[i].root_node_index != right.roots[i].root_node_index) {
      return false;
    }
  }
  for (std::size_t i = 0; i < left.nodes.size(); ++i) {
    if (!(left.nodes[i].center.array() ==
          right.nodes[i].center.array()).all() ||
        left.nodes[i].half_length != right.nodes[i].half_length ||
        left.nodes[i].children != right.nodes[i].children ||
        left.nodes[i].plane_index != right.nodes[i].plane_index) {
      return false;
    }
  }
  for (std::size_t i = 0; i < left.planes.size(); ++i) {
    if (!(left.planes[i].center.array() ==
          right.planes[i].center.array()).all() ||
        !(left.planes[i].normal.array() ==
          right.planes[i].normal.array()).all() ||
        !(left.planes[i].covariance.array() ==
          right.planes[i].covariance.array()).all() ||
        left.planes[i].radius != right.planes[i].radius) {
      return false;
    }
  }
  for (std::size_t i = 0; i < left.display_points.size(); ++i) {
    if (!points_equal(left.display_points[i], right.display_points[i])) {
      return false;
    }
  }
  return true;
}

bool configs_equal(const SpatialMapConfig &left,
                   const SpatialMapConfig &right) {
  return left.size_x == right.size_x && left.size_y == right.size_y &&
         left.size_z == right.size_z &&
         left.fusion_leaf_size == right.fusion_leaf_size &&
         left.root_voxel_size == right.root_voxel_size &&
         left.max_layer == right.max_layer &&
         left.min_eigen_value == right.min_eigen_value &&
         left.min_points == right.min_points &&
         left.plane_ratio_threshold == right.plane_ratio_threshold &&
         left.save_display_cloud == right.save_display_cloud;
}

std::vector<std::uint8_t> read_bytes(const fs::path &path) {
  std::ifstream input(path.string(), std::ios::binary);
  return std::vector<std::uint8_t>(std::istreambuf_iterator<char>(input),
                                   std::istreambuf_iterator<char>());
}

void write_bytes(const fs::path &path,
                 const std::vector<std::uint8_t> &bytes) {
  std::ofstream output(path.string(), std::ios::binary | std::ios::trunc);
  ASSERT_TRUE(output.good());
  if (!bytes.empty()) {
    output.write(reinterpret_cast<const char *>(bytes.data()),
                 static_cast<std::streamsize>(bytes.size()));
  }
  ASSERT_TRUE(output.good());
}

std::uint32_t read_u32(const std::vector<std::uint8_t> &bytes,
                       std::size_t offset) {
  std::uint32_t value = 0U;
  for (unsigned int shift = 0; shift < 32; shift += 8) {
    value |= static_cast<std::uint32_t>(bytes[offset + shift / 8U]) << shift;
  }
  return value;
}

std::uint64_t read_u64(const std::vector<std::uint8_t> &bytes,
                       std::size_t offset) {
  std::uint64_t value = 0U;
  for (unsigned int shift = 0; shift < 64; shift += 8) {
    value |= static_cast<std::uint64_t>(bytes[offset + shift / 8U]) << shift;
  }
  return value;
}

void write_u32(std::vector<std::uint8_t> &bytes, std::size_t offset,
               std::uint32_t value) {
  for (unsigned int shift = 0; shift < 32; shift += 8) {
    bytes[offset + shift / 8U] =
        static_cast<std::uint8_t>((value >> shift) & 0xffU);
  }
}

void write_u64(std::vector<std::uint8_t> &bytes, std::size_t offset,
               std::uint64_t value) {
  for (unsigned int shift = 0; shift < 64; shift += 8) {
    bytes[offset + shift / 8U] =
        static_cast<std::uint8_t>((value >> shift) & UINT64_C(0xff));
  }
}

std::uint32_t crc32(const std::uint8_t *data, std::size_t size) {
  std::uint32_t crc = UINT32_C(0xffffffff);
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      const std::uint32_t mask =
          static_cast<std::uint32_t>(-
              static_cast<std::int32_t>(crc & UINT32_C(1)));
      crc = (crc >> 1U) ^ (UINT32_C(0xedb88320) & mask);
    }
  }
  return ~crc;
}

void refresh_first_checksum(std::vector<std::uint8_t> &index_bytes,
                            const std::vector<std::uint8_t> &data_bytes) {
  const std::uint64_t offset =
      read_u64(index_bytes, kIndexHeaderSize + kIndexOffsetField);
  const std::uint64_t length =
      read_u64(index_bytes, kIndexHeaderSize + kIndexLengthField);
  ASSERT_LE(offset, data_bytes.size());
  ASSERT_LE(length, data_bytes.size() - static_cast<std::size_t>(offset));
  write_u32(index_bytes, kIndexHeaderSize + kIndexChecksumField,
            crc32(data_bytes.data() + static_cast<std::size_t>(offset),
                  static_cast<std::size_t>(length)));
}

SpatialMapCatalog sentinel_catalog() {
  SpatialMapCatalog catalog;
  catalog.config.size_x = 40.0;
  SpatialBlockIndexEntry entry;
  entry.index = BlockIndex(9, 8, 7);
  entry.offset = 123U;
  entry.length = 456U;
  catalog.entries.push_back(entry);
  catalog.lookup.emplace(entry.index, 0U);
  return catalog;
}

StaticMapBlock sentinel_block() {
  return make_block(BlockIndex(9, 8, 7), RootVoxelIndex(180, 160, 70),
                    true);
}

TEST(SpatialMapIO, RoundTripsCatalogAndRandomAccessBlocksExactly) {
  TemporaryDirectory temp;
  const std::vector<StaticMapBlock> blocks = two_block_fixture();
  const SpatialMapConfig config = test_config();
  SpatialMapCatalog written;
  std::string error;
  ASSERT_TRUE(SpatialMapIO::save(temp.path(), blocks, config, written, error))
      << error;
  ASSERT_EQ(written.entries.size(), 2U);
  EXPECT_EQ(written.lookup.size(), 2U);
  EXPECT_TRUE(configs_equal(written.config, config));

  SpatialMapCatalog loaded;
  ASSERT_TRUE(SpatialMapIO::load_catalog(temp.path(), loaded, error)) << error;
  ASSERT_EQ(loaded.entries.size(), 2U);
  EXPECT_EQ(loaded.lookup.size(), 2U);
  EXPECT_TRUE(configs_equal(loaded.config, config));

  for (const StaticMapBlock &expected : blocks) {
    StaticMapBlock actual;
    ASSERT_TRUE(SpatialMapIO::load_block(temp.path(), loaded, expected.index,
                                         actual, error))
        << error;
    EXPECT_TRUE(blocks_equal(actual, expected));
  }
  EXPECT_TRUE(SpatialMapIO::validate_all_blocks(temp.path(), loaded, error))
      << error;
}

TEST(SpatialMapIO, PreservesNegativeIndicesAndEmptyDisplayCloud) {
  TemporaryDirectory temp;
  const std::vector<StaticMapBlock> blocks = two_block_fixture();
  SpatialMapCatalog catalog;
  std::string error;
  ASSERT_TRUE(SpatialMapIO::save(temp.path(), blocks, test_config(), catalog,
                                 error))
      << error;

  StaticMapBlock loaded;
  ASSERT_TRUE(SpatialMapIO::load_block(temp.path(), catalog,
                                       BlockIndex(-1, 0, -1), loaded, error))
      << error;
  EXPECT_EQ(loaded.index, BlockIndex(-1, 0, -1));
  EXPECT_TRUE(loaded.display_points.empty());
}

TEST(SpatialMapIO, ReorderedLogicalInputProducesIdenticalBytes) {
  TemporaryDirectory first;
  TemporaryDirectory second;
  std::vector<StaticMapBlock> blocks = two_block_fixture();
  RootVoxelRecord extra_root;
  extra_root.index = RootVoxelIndex(21, -40, 0);
  extra_root.root_node_index = 0U;
  blocks[0].roots.push_back(extra_root);

  SpatialMapCatalog catalog;
  std::string error;
  ASSERT_TRUE(SpatialMapIO::save(first.path(), blocks, test_config(), catalog,
                                 error))
      << error;
  std::reverse(blocks.begin(), blocks.end());
  std::reverse(blocks[1].roots.begin(), blocks[1].roots.end());
  std::reverse(blocks[1].display_points.begin(),
               blocks[1].display_points.end());
  ASSERT_TRUE(SpatialMapIO::save(second.path(), blocks, test_config(), catalog,
                                 error))
      << error;

  EXPECT_EQ(read_bytes(first.index_path()), read_bytes(second.index_path()));
  EXPECT_EQ(read_bytes(first.data_path()), read_bytes(second.data_path()));
}

TEST(SpatialMapIO, CatalogLoadDoesNotValidateEveryPayload) {
  TemporaryDirectory temp;
  SpatialMapCatalog catalog;
  std::string error;
  ASSERT_TRUE(SpatialMapIO::save(temp.path(), two_block_fixture(),
                                 test_config(), catalog, error))
      << error;
  std::vector<std::uint8_t> data = read_bytes(temp.data_path());
  ASSERT_GT(data.size(), kDataHeaderSize);
  data.back() ^= UINT8_C(0x80);
  write_bytes(temp.data_path(), data);

  SpatialMapCatalog loaded;
  EXPECT_TRUE(SpatialMapIO::load_catalog(temp.path(), loaded, error)) << error;
  EXPECT_FALSE(SpatialMapIO::validate_all_blocks(temp.path(), loaded, error));
}

TEST(SpatialMapIO, MissingBlockFailsClearlyAndPreservesOutput) {
  TemporaryDirectory temp;
  SpatialMapCatalog catalog;
  std::string error;
  ASSERT_TRUE(SpatialMapIO::save(temp.path(), two_block_fixture(),
                                 test_config(), catalog, error))
      << error;
  StaticMapBlock output = sentinel_block();
  const StaticMapBlock before = output;
  EXPECT_FALSE(SpatialMapIO::load_block(temp.path(), catalog,
                                        BlockIndex(404, 0, 0), output, error));
  EXPECT_NE(error.find("not present"), std::string::npos);
  EXPECT_TRUE(blocks_equal(output, before));
}

TEST(SpatialMapIO, RejectsTruncatedIndexAndDataAtomically) {
  TemporaryDirectory temp;
  SpatialMapCatalog catalog;
  std::string error;
  ASSERT_TRUE(SpatialMapIO::save(temp.path(), two_block_fixture(),
                                 test_config(), catalog, error))
      << error;

  std::vector<std::uint8_t> index = read_bytes(temp.index_path());
  index.resize(index.size() - 1U);
  write_bytes(temp.index_path(), index);
  SpatialMapCatalog output = sentinel_catalog();
  EXPECT_FALSE(SpatialMapIO::load_catalog(temp.path(), output, error));
  EXPECT_EQ(output.entries[0].index, BlockIndex(9, 8, 7));

  ASSERT_TRUE(SpatialMapIO::save(temp.path(), two_block_fixture(),
                                 test_config(), catalog, error))
      << error;
  std::vector<std::uint8_t> data = read_bytes(temp.data_path());
  data.resize(data.size() - 1U);
  write_bytes(temp.data_path(), data);
  output = sentinel_catalog();
  EXPECT_FALSE(SpatialMapIO::load_catalog(temp.path(), output, error));
  EXPECT_EQ(output.entries[0].index, BlockIndex(9, 8, 7));
}

TEST(SpatialMapIO, RejectsBadMagicVersionAndEndianMarkers) {
  const std::array<std::size_t, 3> fields{{0U, 8U, 12U}};
  for (const bool corrupt_index : {true, false}) {
    for (const std::size_t field : fields) {
      TemporaryDirectory temp;
      SpatialMapCatalog catalog;
      std::string error;
      ASSERT_TRUE(SpatialMapIO::save(temp.path(), two_block_fixture(),
                                     test_config(), catalog, error))
          << error;
      const fs::path path =
          corrupt_index ? temp.index_path() : temp.data_path();
      std::vector<std::uint8_t> bytes = read_bytes(path);
      bytes[field] ^= UINT8_C(0x5a);
      write_bytes(path, bytes);

      SpatialMapCatalog output = sentinel_catalog();
      EXPECT_FALSE(SpatialMapIO::load_catalog(temp.path(), output, error));
      EXPECT_EQ(output.entries[0].index, BlockIndex(9, 8, 7));
    }
  }
}

TEST(SpatialMapIO, RejectsBadCrcAndIdentifiesCorruptBlock) {
  TemporaryDirectory temp;
  SpatialMapCatalog catalog;
  std::string error;
  ASSERT_TRUE(SpatialMapIO::save(temp.path(), two_block_fixture(),
                                 test_config(), catalog, error))
      << error;
  const auto corrupt_entry = catalog.lookup.at(BlockIndex(1, -2, 0));
  const SpatialBlockIndexEntry &entry = catalog.entries[corrupt_entry];
  std::vector<std::uint8_t> data = read_bytes(temp.data_path());
  ASSERT_GT(entry.length, 0U);
  data[static_cast<std::size_t>(entry.offset + entry.length - 1U)] ^=
      UINT8_C(0x01);
  write_bytes(temp.data_path(), data);

  StaticMapBlock output = sentinel_block();
  EXPECT_FALSE(SpatialMapIO::load_block(temp.path(), catalog,
                                        entry.index, output, error));
  EXPECT_NE(error.find("CRC"), std::string::npos);
  EXPECT_TRUE(blocks_equal(output, sentinel_block()));
  EXPECT_FALSE(SpatialMapIO::validate_all_blocks(temp.path(), catalog, error));
  EXPECT_NE(error.find("(1,-2,0)"), std::string::npos);
}

TEST(SpatialMapIO, RejectsOverflowingAndOutOfFileRanges) {
  for (const bool overflowing : {true, false}) {
    TemporaryDirectory temp;
    SpatialMapCatalog catalog;
    std::string error;
    ASSERT_TRUE(SpatialMapIO::save(temp.path(), two_block_fixture(),
                                   test_config(), catalog, error))
        << error;
    std::vector<std::uint8_t> index = read_bytes(temp.index_path());
    const std::size_t record = kIndexHeaderSize;
    write_u64(index, record + kIndexOffsetField,
              overflowing ? std::numeric_limits<std::uint64_t>::max()
                          : static_cast<std::uint64_t>(
                                read_bytes(temp.data_path()).size() + 1U));
    if (overflowing) {
      write_u64(index, record + kIndexLengthField, 2U);
    }
    write_bytes(temp.index_path(), index);
    SpatialMapCatalog output = sentinel_catalog();
    EXPECT_FALSE(SpatialMapIO::load_catalog(temp.path(), output, error));
    EXPECT_EQ(output.entries[0].index, BlockIndex(9, 8, 7));
  }
}

TEST(SpatialMapIO, RejectsOversizedCountsBeforeAllocation) {
  TemporaryDirectory temp;
  SpatialMapCatalog catalog;
  std::string error;
  ASSERT_TRUE(SpatialMapIO::save(temp.path(), two_block_fixture(),
                                 test_config(), catalog, error))
      << error;
  std::vector<std::uint8_t> index = read_bytes(temp.index_path());
  write_u32(index, kIndexHeaderSize + kIndexRootCountField,
            std::numeric_limits<std::uint32_t>::max());
  write_bytes(temp.index_path(), index);

  SpatialMapCatalog output = sentinel_catalog();
  EXPECT_FALSE(SpatialMapIO::load_catalog(temp.path(), output, error));
  EXPECT_NE(error.find("count"), std::string::npos);
  EXPECT_EQ(output.entries[0].index, BlockIndex(9, 8, 7));
}

TEST(SpatialMapIO, RejectsDuplicateAndOverlappingIndexRecords) {
  for (const bool duplicate : {true, false}) {
    TemporaryDirectory temp;
    SpatialMapCatalog catalog;
    std::string error;
    ASSERT_TRUE(SpatialMapIO::save(temp.path(), two_block_fixture(),
                                   test_config(), catalog, error))
        << error;
    std::vector<std::uint8_t> index = read_bytes(temp.index_path());
    const std::size_t first = kIndexHeaderSize;
    const std::size_t second = first + kIndexRecordSize;
    if (duplicate) {
      std::copy(index.begin() + static_cast<std::ptrdiff_t>(first),
                index.begin() + static_cast<std::ptrdiff_t>(first + 72U),
                index.begin() + static_cast<std::ptrdiff_t>(second));
    } else {
      write_u64(index, second + kIndexOffsetField,
                read_u64(index, first + kIndexOffsetField));
    }
    write_bytes(temp.index_path(), index);

    SpatialMapCatalog output = sentinel_catalog();
    EXPECT_FALSE(SpatialMapIO::load_catalog(temp.path(), output, error));
    EXPECT_EQ(output.entries[0].index, BlockIndex(9, 8, 7));
  }
}

TEST(SpatialMapIO, RejectsDuplicateRootsInvalidChildrenAndPlaneReferences) {
  enum class Corruption { kDuplicateRoot, kChild, kPlane };
  for (const Corruption corruption :
       {Corruption::kDuplicateRoot, Corruption::kChild,
        Corruption::kPlane}) {
    TemporaryDirectory temp;
    std::vector<StaticMapBlock> blocks;
    blocks.push_back(make_block(BlockIndex(1, -2, 0),
                                RootVoxelIndex(20, -40, 0), true));
    RootVoxelRecord second_root;
    second_root.index = RootVoxelIndex(21, -40, 0);
    second_root.root_node_index = 0U;
    blocks[0].roots.push_back(second_root);
    SpatialMapCatalog catalog;
    std::string error;
    ASSERT_TRUE(SpatialMapIO::save(temp.path(), blocks, test_config(),
                                   catalog, error))
        << error;

    std::vector<std::uint8_t> index = read_bytes(temp.index_path());
    std::vector<std::uint8_t> data = read_bytes(temp.data_path());
    const std::size_t payload = static_cast<std::size_t>(
        read_u64(index, kIndexHeaderSize + kIndexOffsetField));
    ASSERT_EQ(read_u32(data, payload + kPayloadRootCountField), 2U);
    if (corruption == Corruption::kDuplicateRoot) {
      std::copy(data.begin() + static_cast<std::ptrdiff_t>(
                    payload + kPayloadRoots),
                data.begin() + static_cast<std::ptrdiff_t>(
                    payload + kPayloadRoots + 24U),
                data.begin() + static_cast<std::ptrdiff_t>(
                    payload + kPayloadRoots + kRootRecordSize));
    } else {
      const std::size_t node =
          payload + kPayloadRoots + 2U * kRootRecordSize;
      write_u32(data,
                node + (corruption == Corruption::kChild
                            ? kNodeChildrenField
                            : kNodePlaneField),
                99U);
    }
    refresh_first_checksum(index, data);
    write_bytes(temp.data_path(), data);
    write_bytes(temp.index_path(), index);

    SpatialMapCatalog corrupted_catalog;
    ASSERT_TRUE(SpatialMapIO::load_catalog(temp.path(), corrupted_catalog,
                                           error))
        << error;
    StaticMapBlock output = sentinel_block();
    EXPECT_FALSE(SpatialMapIO::load_block(temp.path(), corrupted_catalog,
                                          blocks[0].index, output, error));
    EXPECT_TRUE(blocks_equal(output, sentinel_block()));
  }
}

TEST(SpatialMapIO, RejectsPayloadCountMismatchAndBlockIndexMismatch) {
  for (const bool count_mismatch : {true, false}) {
    TemporaryDirectory temp;
    SpatialMapCatalog catalog;
    std::string error;
    ASSERT_TRUE(SpatialMapIO::save(temp.path(), two_block_fixture(),
                                   test_config(), catalog, error))
        << error;
    std::vector<std::uint8_t> index = read_bytes(temp.index_path());
    std::vector<std::uint8_t> data = read_bytes(temp.data_path());
    const std::size_t payload = static_cast<std::size_t>(
        read_u64(index, kIndexHeaderSize + kIndexOffsetField));
    if (count_mismatch) {
      write_u32(data, payload + kPayloadRootCountField,
                read_u32(data, payload + kPayloadRootCountField) + 1U);
    } else {
      data[payload] ^= UINT8_C(0x01);
    }
    refresh_first_checksum(index, data);
    write_bytes(temp.data_path(), data);
    write_bytes(temp.index_path(), index);

    SpatialMapCatalog corrupted_catalog;
    ASSERT_TRUE(SpatialMapIO::load_catalog(temp.path(), corrupted_catalog,
                                           error))
        << error;
    StaticMapBlock output = sentinel_block();
    EXPECT_FALSE(SpatialMapIO::load_block(temp.path(), corrupted_catalog,
                                          corrupted_catalog.entries[0].index,
                                          output, error));
    EXPECT_TRUE(blocks_equal(output, sentinel_block()));
  }
}

TEST(SpatialMapIO, FailedSavePreservesExistingPublishedPairAndOutput) {
  TemporaryDirectory temp;
  SpatialMapCatalog catalog;
  std::string error;
  ASSERT_TRUE(SpatialMapIO::save(temp.path(), two_block_fixture(),
                                 test_config(), catalog, error))
      << error;
  const std::vector<std::uint8_t> index_before = read_bytes(temp.index_path());
  const std::vector<std::uint8_t> data_before = read_bytes(temp.data_path());
  SpatialMapCatalog output = sentinel_catalog();

  std::vector<StaticMapBlock> invalid = two_block_fixture();
  invalid.push_back(invalid.front());
  EXPECT_FALSE(SpatialMapIO::save(temp.path(), invalid, test_config(), output,
                                  error));
  EXPECT_EQ(read_bytes(temp.index_path()), index_before);
  EXPECT_EQ(read_bytes(temp.data_path()), data_before);
  EXPECT_EQ(output.entries[0].index, BlockIndex(9, 8, 7));
}

}  // namespace
