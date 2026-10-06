#include "spatial_map_io.hpp"

#include <boost/filesystem.hpp>
#include <boost/system/error_code.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

namespace fs = boost::filesystem;

const char kIndexMagic[8] = {'V', 'X', 'B', 'L', 'K', 'I', 'D', 'X'};
const char kDataMagic[8] = {'V', 'X', 'B', 'L', 'K', 'D', 'A', 'T'};
constexpr std::uint32_t kFormatVersion = 1U;
constexpr std::uint32_t kEndianMarker = UINT32_C(0x01020304);
constexpr std::uint32_t kScalarMarker = 1U;
constexpr std::uint32_t kIndexHeaderSize = 144U;
constexpr std::uint32_t kDataHeaderSize = 24U;
constexpr std::uint64_t kIndexRecordSize = 108U;
constexpr std::uint64_t kRootRecordSize = 28U;
constexpr std::uint64_t kNodeRecordSize = 68U;
constexpr std::uint64_t kPlaneRecordSize = 344U;
constexpr std::uint64_t kDisplayPointRecordSize = 100U;
constexpr std::uint64_t kBlockPayloadPrefixSize = 40U;

constexpr std::uint64_t kMaxBlockCount = UINT64_C(1000000);
constexpr std::uint32_t kMaxRootCount = UINT32_C(1000000);
constexpr std::uint32_t kMaxNodeCount = UINT32_C(8000000);
constexpr std::uint32_t kMaxPlaneCount = UINT32_C(8000000);
constexpr std::uint32_t kMaxDisplayPointCount = UINT32_C(16000000);
constexpr std::uint64_t kMaxPayloadSize = UINT64_C(2147483648);

bool safe_add(std::uint64_t left, std::uint64_t right,
              std::uint64_t &result) {
  if (left > std::numeric_limits<std::uint64_t>::max() - right) {
    return false;
  }
  result = left + right;
  return true;
}

bool safe_multiply(std::uint64_t left, std::uint64_t right,
                   std::uint64_t &result) {
  if (left != 0U &&
      right > std::numeric_limits<std::uint64_t>::max() / left) {
    return false;
  }
  result = left * right;
  return true;
}

std::string block_label(const BlockIndex &index) {
  std::ostringstream stream;
  stream << '(' << index.x << ',' << index.y << ',' << index.z << ')';
  return stream.str();
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

bool counts_within_limits(std::uint32_t roots, std::uint32_t nodes,
                          std::uint32_t planes, std::uint32_t points,
                          std::string &error) {
  if (roots == 0U || nodes == 0U) {
    error = "spatial block counts must include a root and a node";
    return false;
  }
  if (roots > kMaxRootCount || nodes > kMaxNodeCount ||
      planes > kMaxPlaneCount || points > kMaxDisplayPointCount) {
    error = "spatial block count exceeds the format safety limit";
    return false;
  }
  return true;
}

bool expected_payload_size(std::uint32_t roots, std::uint32_t nodes,
                           std::uint32_t planes, std::uint32_t points,
                           std::uint64_t &size, std::string &error) {
  if (!counts_within_limits(roots, nodes, planes, points, error)) {
    return false;
  }
  size = kBlockPayloadPrefixSize;
  const struct {
    std::uint32_t count;
    std::uint64_t record_size;
  } fields[] = {{roots, kRootRecordSize},
                {nodes, kNodeRecordSize},
                {planes, kPlaneRecordSize},
                {points, kDisplayPointRecordSize}};
  for (const auto &field : fields) {
    std::uint64_t bytes = 0U;
    if (!safe_multiply(field.count, field.record_size, bytes) ||
        !safe_add(size, bytes, size)) {
      error = "spatial block count-to-size calculation overflowed";
      return false;
    }
  }
  if (size > kMaxPayloadSize) {
    error = "spatial block payload exceeds the format safety limit";
    return false;
  }
  if (size > static_cast<std::uint64_t>(
                 std::numeric_limits<std::size_t>::max()) ||
      size > static_cast<std::uint64_t>(
                 std::numeric_limits<std::streamsize>::max())) {
    error = "spatial block payload is too large for this platform";
    return false;
  }
  return true;
}

class ByteWriter {
 public:
  void bytes(const char *data, std::size_t size) {
    data_.insert(data_.end(), data, data + size);
  }

  void u8(std::uint8_t value) { data_.push_back(static_cast<char>(value)); }

  void u32(std::uint32_t value) {
    for (unsigned int shift = 0; shift < 32; shift += 8) {
      u8(static_cast<std::uint8_t>((value >> shift) & UINT32_C(0xff)));
    }
  }

  void i32(std::int32_t value) {
    u32(static_cast<std::uint32_t>(value));
  }

  void u64(std::uint64_t value) {
    for (unsigned int shift = 0; shift < 64; shift += 8) {
      u8(static_cast<std::uint8_t>((value >> shift) & UINT64_C(0xff)));
    }
  }

  void i64(std::int64_t value) {
    u64(static_cast<std::uint64_t>(value));
  }

  void f32(float value) {
    std::uint32_t bits = 0U;
    static_assert(sizeof(bits) == sizeof(value),
                  "format requires 32-bit float");
    std::memcpy(&bits, &value, sizeof(bits));
    u32(bits);
  }

  void f64(double value) {
    std::uint64_t bits = 0U;
    static_assert(sizeof(bits) == sizeof(value),
                  "format requires 64-bit double");
    std::memcpy(&bits, &value, sizeof(bits));
    u64(bits);
  }

  void append(const std::vector<char> &other) {
    data_.insert(data_.end(), other.begin(), other.end());
  }

  const std::vector<char> &data() const { return data_; }
  std::vector<char> release() { return std::move(data_); }

 private:
  std::vector<char> data_;
};

class ByteReader {
 public:
  ByteReader(const char *data, std::size_t size)
      : data_(data), size_(size) {}

  bool bytes(char *destination, std::size_t count) {
    if (count > size_ - position_) {
      return false;
    }
    std::memcpy(destination, data_ + position_, count);
    position_ += count;
    return true;
  }

  bool u8(std::uint8_t &value) {
    if (position_ >= size_) {
      return false;
    }
    value = static_cast<std::uint8_t>(data_[position_]);
    ++position_;
    return true;
  }

  bool u32(std::uint32_t &value) {
    value = 0U;
    for (unsigned int shift = 0; shift < 32; shift += 8) {
      std::uint8_t byte = 0U;
      if (!u8(byte)) {
        return false;
      }
      value |= static_cast<std::uint32_t>(byte) << shift;
    }
    return true;
  }

  bool i32(std::int32_t &value) {
    std::uint32_t bits = 0U;
    if (!u32(bits)) {
      return false;
    }
    if (bits <= static_cast<std::uint32_t>(
                    std::numeric_limits<std::int32_t>::max())) {
      value = static_cast<std::int32_t>(bits);
    } else {
      value = -1 - static_cast<std::int32_t>(
                       std::numeric_limits<std::uint32_t>::max() - bits);
    }
    return true;
  }

  bool u64(std::uint64_t &value) {
    value = 0U;
    for (unsigned int shift = 0; shift < 64; shift += 8) {
      std::uint8_t byte = 0U;
      if (!u8(byte)) {
        return false;
      }
      value |= static_cast<std::uint64_t>(byte) << shift;
    }
    return true;
  }

  bool i64(std::int64_t &value) {
    std::uint64_t bits = 0U;
    if (!u64(bits)) {
      return false;
    }
    if (bits <= static_cast<std::uint64_t>(
                    std::numeric_limits<std::int64_t>::max())) {
      value = static_cast<std::int64_t>(bits);
    } else {
      value = -1 - static_cast<std::int64_t>(
                       std::numeric_limits<std::uint64_t>::max() - bits);
    }
    return true;
  }

  bool f32(float &value) {
    std::uint32_t bits = 0U;
    if (!u32(bits)) {
      return false;
    }
    std::memcpy(&value, &bits, sizeof(value));
    return true;
  }

  bool f64(double &value) {
    std::uint64_t bits = 0U;
    if (!u64(bits)) {
      return false;
    }
    std::memcpy(&value, &bits, sizeof(value));
    return true;
  }

  std::size_t position() const { return position_; }
  std::size_t remaining() const { return size_ - position_; }

 private:
  const char *data_;
  std::size_t size_;
  std::size_t position_ = 0U;
};

std::uint32_t crc32_update(std::uint32_t crc, const char *data,
                           std::size_t size) {
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= static_cast<std::uint8_t>(data[i]);
    for (int bit = 0; bit < 8; ++bit) {
      const std::uint32_t mask =
          static_cast<std::uint32_t>(-
              static_cast<std::int32_t>(crc & UINT32_C(1)));
      crc = (crc >> 1U) ^ (UINT32_C(0xedb88320) & mask);
    }
  }
  return crc;
}

std::uint32_t crc32(const char *data, std::size_t size) {
  const std::uint32_t crc =
      crc32_update(UINT32_C(0xffffffff), data, size);
  return ~crc;
}

void write_vector(ByteWriter &writer, const Eigen::Vector3d &value) {
  for (int axis = 0; axis < 3; ++axis) {
    writer.f64(value[axis]);
  }
}

bool read_vector(ByteReader &reader, Eigen::Vector3d &value) {
  for (int axis = 0; axis < 3; ++axis) {
    if (!reader.f64(value[axis])) {
      return false;
    }
  }
  return true;
}

template <int Rows, int Columns>
void write_matrix(ByteWriter &writer,
                  const Eigen::Matrix<double, Rows, Columns> &value) {
  for (int row = 0; row < Rows; ++row) {
    for (int column = 0; column < Columns; ++column) {
      writer.f64(value(row, column));
    }
  }
}

template <int Rows, int Columns>
bool read_matrix(ByteReader &reader,
                 Eigen::Matrix<double, Rows, Columns> &value) {
  for (int row = 0; row < Rows; ++row) {
    for (int column = 0; column < Columns; ++column) {
      if (!reader.f64(value(row, column))) {
        return false;
      }
    }
  }
  return true;
}

void write_block_index(ByteWriter &writer, const BlockIndex &index) {
  writer.i64(index.x);
  writer.i64(index.y);
  writer.i64(index.z);
}

bool read_block_index(ByteReader &reader, BlockIndex &index) {
  return reader.i64(index.x) && reader.i64(index.y) && reader.i64(index.z);
}

void write_root_index(ByteWriter &writer, const RootVoxelIndex &index) {
  writer.i64(index.x);
  writer.i64(index.y);
  writer.i64(index.z);
}

bool read_root_index(ByteReader &reader, RootVoxelIndex &index) {
  return reader.i64(index.x) && reader.i64(index.y) && reader.i64(index.z);
}

void write_config(ByteWriter &writer, const SpatialMapConfig &config) {
  writer.f64(config.size_x);
  writer.f64(config.size_y);
  writer.f64(config.size_z);
  writer.f64(config.fusion_leaf_size);
  writer.f64(config.root_voxel_size);
  writer.i32(static_cast<std::int32_t>(config.max_layer));
  writer.f64(config.min_eigen_value);
  for (const int value : config.min_points) {
    writer.i32(static_cast<std::int32_t>(value));
  }
  for (const double value : config.plane_ratio_threshold) {
    writer.f64(value);
  }
  writer.u8(config.save_display_cloud ? 1U : 0U);
  writer.u8(0U);
  writer.u8(0U);
  writer.u8(0U);
}

bool read_config(ByteReader &reader, SpatialMapConfig &config,
                 std::string &error) {
  std::int32_t max_layer = 0;
  if (!reader.f64(config.size_x) || !reader.f64(config.size_y) ||
      !reader.f64(config.size_z) ||
      !reader.f64(config.fusion_leaf_size) ||
      !reader.f64(config.root_voxel_size) || !reader.i32(max_layer) ||
      !reader.f64(config.min_eigen_value)) {
    error = "truncated spatial map configuration";
    return false;
  }
  config.max_layer = static_cast<int>(max_layer);
  for (int &value : config.min_points) {
    std::int32_t decoded = 0;
    if (!reader.i32(decoded)) {
      error = "truncated spatial map min_points configuration";
      return false;
    }
    value = static_cast<int>(decoded);
  }
  for (double &value : config.plane_ratio_threshold) {
    if (!reader.f64(value)) {
      error = "truncated spatial map plane threshold configuration";
      return false;
    }
  }
  std::uint8_t display = 0U;
  std::uint8_t reserved[3] = {0U, 0U, 0U};
  if (!reader.u8(display) || !reader.u8(reserved[0]) ||
      !reader.u8(reserved[1]) || !reader.u8(reserved[2])) {
    error = "truncated spatial map configuration flags";
    return false;
  }
  if (display > 1U || reserved[0] != 0U || reserved[1] != 0U ||
      reserved[2] != 0U) {
    error = "invalid spatial map configuration flags";
    return false;
  }
  config.save_display_cloud = display != 0U;
  std::string config_error;
  if (!config.validate(config_error)) {
    error = "invalid spatial map configuration: " + config_error;
    return false;
  }
  return true;
}

void write_common_header(ByteWriter &writer, const char magic[8],
                         std::uint32_t header_size) {
  writer.bytes(magic, 8U);
  writer.u32(kFormatVersion);
  writer.u32(kEndianMarker);
  writer.u32(kScalarMarker);
  writer.u32(header_size);
}

bool read_common_header(ByteReader &reader, const char expected_magic[8],
                        std::uint32_t expected_header_size,
                        const char *kind, std::string &error) {
  char magic[8] = {};
  std::uint32_t version = 0U;
  std::uint32_t endian = 0U;
  std::uint32_t scalar = 0U;
  std::uint32_t header_size = 0U;
  if (!reader.bytes(magic, sizeof(magic)) || !reader.u32(version) ||
      !reader.u32(endian) || !reader.u32(scalar) ||
      !reader.u32(header_size)) {
    error = std::string("truncated ") + kind + " header";
    return false;
  }
  if (!std::equal(magic, magic + sizeof(magic), expected_magic)) {
    error = std::string("invalid ") + kind + " magic";
    return false;
  }
  if (version != kFormatVersion) {
    error = std::string("unsupported ") + kind + " format version";
    return false;
  }
  if (endian != kEndianMarker) {
    error = std::string("invalid ") + kind + " endian marker";
    return false;
  }
  if (scalar != kScalarMarker || !std::numeric_limits<float>::is_iec559 ||
      !std::numeric_limits<double>::is_iec559) {
    error = std::string("unsupported ") + kind + " scalar encoding";
    return false;
  }
  if (header_size != expected_header_size) {
    error = std::string("invalid ") + kind + " header size";
    return false;
  }
  return true;
}

bool point_less(const SpatialMapPoint &left,
                const SpatialMapPoint &right) {
  for (int axis = 0; axis < 3; ++axis) {
    if (left.position[axis] != right.position[axis]) {
      return left.position[axis] < right.position[axis];
    }
    if (std::signbit(left.position[axis]) !=
        std::signbit(right.position[axis])) {
      return std::signbit(left.position[axis]);
    }
  }
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      if (left.covariance(row, column) != right.covariance(row, column)) {
        return left.covariance(row, column) < right.covariance(row, column);
      }
      if (std::signbit(left.covariance(row, column)) !=
          std::signbit(right.covariance(row, column))) {
        return std::signbit(left.covariance(row, column));
      }
    }
  }
  if (left.intensity != right.intensity) {
    return left.intensity < right.intensity;
  }
  return std::signbit(left.intensity) && !std::signbit(right.intensity);
}

bool validate_block(const StaticMapBlock &block,
                    const SpatialMapConfig &config,
                    std::string &error) {
  if (block.roots.empty()) {
    error = "block " + block_label(block.index) +
            " does not contain a root voxel";
    return false;
  }
  if (block.roots.size() > std::numeric_limits<std::uint32_t>::max() ||
      block.nodes.size() > std::numeric_limits<std::uint32_t>::max() ||
      block.planes.size() > std::numeric_limits<std::uint32_t>::max() ||
      block.display_points.size() >
          std::numeric_limits<std::uint32_t>::max()) {
    error = "block " + block_label(block.index) +
            " has a vector count outside the format range";
    return false;
  }
  std::string count_error;
  if (!counts_within_limits(
          static_cast<std::uint32_t>(block.roots.size()),
          static_cast<std::uint32_t>(block.nodes.size()),
          static_cast<std::uint32_t>(block.planes.size()),
          static_cast<std::uint32_t>(block.display_points.size()),
          count_error)) {
    error = "block " + block_label(block.index) + ": " + count_error;
    return false;
  }

  std::set<RootVoxelIndex> roots;
  for (const RootVoxelRecord &root : block.roots) {
    if (!roots.insert(root.index).second) {
      error = "block " + block_label(block.index) +
              " contains a duplicate root voxel";
      return false;
    }
    try {
      if (root_voxel_to_block(root.index, config) != block.index) {
        error = "block " + block_label(block.index) +
                " contains a root voxel owned by another block";
        return false;
      }
    } catch (const std::exception &exception) {
      error = "block " + block_label(block.index) +
              " has an invalid root voxel: " + exception.what();
      return false;
    }
    if (root.root_node_index >= block.nodes.size()) {
      error = "block " + block_label(block.index) +
              " has an invalid root node index";
      return false;
    }
  }

  for (std::size_t i = 0; i < block.nodes.size(); ++i) {
    const StaticOctreeNode &node = block.nodes[i];
    if (!vector_is_finite(node.center) ||
        !std::isfinite(node.half_length) || node.half_length <= 0.0) {
      error = "block " + block_label(block.index) +
              " has non-finite or non-positive node geometry";
      return false;
    }
    for (const std::uint32_t child : node.children) {
      if (child != kInvalidSpatialIndex && child >= block.nodes.size()) {
        error = "block " + block_label(block.index) +
                " has an invalid child node index";
        return false;
      }
    }
    if (node.plane_index != kInvalidSpatialIndex &&
        node.plane_index >= block.planes.size()) {
      error = "block " + block_label(block.index) +
              " has an invalid plane index";
      return false;
    }
  }

  for (const StaticPlane &plane : block.planes) {
    if (!vector_is_finite(plane.center) ||
        !vector_is_finite(plane.normal) ||
        !matrix_is_finite(plane.covariance) ||
        !std::isfinite(plane.radius) || plane.radius < 0.0) {
      error = "block " + block_label(block.index) +
              " has invalid plane data";
      return false;
    }
    if (std::abs(plane.normal.norm() - 1.0) > 1e-6 ||
        !plane.covariance.isApprox(plane.covariance.transpose(), 1e-9)) {
      error = "block " + block_label(block.index) +
              " has a non-unit plane normal or asymmetric covariance";
      return false;
    }
  }

  for (const SpatialMapPoint &point : block.display_points) {
    if (!vector_is_finite(point.position) ||
        !matrix_is_finite(point.covariance) ||
        !std::isfinite(static_cast<double>(point.intensity))) {
      error = "block " + block_label(block.index) +
              " has invalid display point data";
      return false;
    }
  }
  if (!config.save_display_cloud && !block.display_points.empty()) {
    error = "block " + block_label(block.index) +
            " contains display points disabled by the configuration";
    return false;
  }
  return true;
}

std::vector<char> serialize_block(const StaticMapBlock &block) {
  ByteWriter writer;
  write_block_index(writer, block.index);
  writer.u32(static_cast<std::uint32_t>(block.roots.size()));
  writer.u32(static_cast<std::uint32_t>(block.nodes.size()));
  writer.u32(static_cast<std::uint32_t>(block.planes.size()));
  writer.u32(static_cast<std::uint32_t>(block.display_points.size()));

  for (const RootVoxelRecord &root : block.roots) {
    write_root_index(writer, root.index);
    writer.u32(root.root_node_index);
  }
  for (const StaticOctreeNode &node : block.nodes) {
    write_vector(writer, node.center);
    writer.f64(node.half_length);
    for (const std::uint32_t child : node.children) {
      writer.u32(child);
    }
    writer.u32(node.plane_index);
  }
  for (const StaticPlane &plane : block.planes) {
    write_vector(writer, plane.center);
    write_vector(writer, plane.normal);
    write_matrix(writer, plane.covariance);
    writer.f64(plane.radius);
  }
  for (const SpatialMapPoint &point : block.display_points) {
    write_vector(writer, point.position);
    write_matrix(writer, point.covariance);
    writer.f32(point.intensity);
  }
  return writer.release();
}

bool deserialize_block(const std::vector<char> &payload,
                       const SpatialBlockIndexEntry &entry,
                       const SpatialMapConfig &config,
                       StaticMapBlock &block, std::string &error) {
  ByteReader reader(payload.data(), payload.size());
  StaticMapBlock candidate;
  std::uint32_t root_count = 0U;
  std::uint32_t node_count = 0U;
  std::uint32_t plane_count = 0U;
  std::uint32_t point_count = 0U;
  if (!read_block_index(reader, candidate.index) ||
      !reader.u32(root_count) || !reader.u32(node_count) ||
      !reader.u32(plane_count) || !reader.u32(point_count)) {
    error = "truncated payload for block " + block_label(entry.index);
    return false;
  }
  if (candidate.index != entry.index) {
    error = "payload block index does not match catalog block " +
            block_label(entry.index);
    return false;
  }
  if (root_count != entry.root_count || node_count != entry.node_count ||
      plane_count != entry.plane_count ||
      point_count != entry.display_point_count) {
    error = "payload counts do not match catalog for block " +
            block_label(entry.index);
    return false;
  }
  std::uint64_t expected_size = 0U;
  std::string size_error;
  if (!expected_payload_size(root_count, node_count, plane_count,
                             point_count, expected_size, size_error)) {
    error = "block " + block_label(entry.index) + ": " + size_error;
    return false;
  }
  if (expected_size != payload.size()) {
    error = "payload size does not match counts for block " +
            block_label(entry.index);
    return false;
  }

  candidate.roots.reserve(root_count);
  for (std::uint32_t i = 0; i < root_count; ++i) {
    RootVoxelRecord root;
    if (!read_root_index(reader, root.index) ||
        !reader.u32(root.root_node_index)) {
      error = "truncated root records for block " +
              block_label(entry.index);
      return false;
    }
    candidate.roots.push_back(root);
  }
  candidate.nodes.reserve(node_count);
  for (std::uint32_t i = 0; i < node_count; ++i) {
    StaticOctreeNode node;
    if (!read_vector(reader, node.center) ||
        !reader.f64(node.half_length)) {
      error = "truncated node records for block " +
              block_label(entry.index);
      return false;
    }
    for (std::uint32_t &child : node.children) {
      if (!reader.u32(child)) {
        error = "truncated node children for block " +
                block_label(entry.index);
        return false;
      }
    }
    if (!reader.u32(node.plane_index)) {
      error = "truncated node plane reference for block " +
              block_label(entry.index);
      return false;
    }
    candidate.nodes.push_back(node);
  }
  candidate.planes.reserve(plane_count);
  for (std::uint32_t i = 0; i < plane_count; ++i) {
    StaticPlane plane;
    if (!read_vector(reader, plane.center) ||
        !read_vector(reader, plane.normal) ||
        !read_matrix(reader, plane.covariance) ||
        !reader.f64(plane.radius)) {
      error = "truncated plane records for block " +
              block_label(entry.index);
      return false;
    }
    candidate.planes.push_back(plane);
  }
  candidate.display_points.reserve(point_count);
  for (std::uint32_t i = 0; i < point_count; ++i) {
    SpatialMapPoint point;
    if (!read_vector(reader, point.position) ||
        !read_matrix(reader, point.covariance) ||
        !reader.f32(point.intensity)) {
      error = "truncated display point records for block " +
              block_label(entry.index);
      return false;
    }
    candidate.display_points.push_back(point);
  }
  if (reader.remaining() != 0U) {
    error = "trailing bytes in payload for block " +
            block_label(entry.index);
    return false;
  }
  if (!validate_block(candidate, config, error)) {
    return false;
  }
  if (!std::is_sorted(candidate.roots.begin(), candidate.roots.end(),
                      [](const RootVoxelRecord &left,
                         const RootVoxelRecord &right) {
                        return left.index < right.index;
                      })) {
    error = "root records are not sorted for block " +
            block_label(entry.index);
    return false;
  }
  block = std::move(candidate);
  return true;
}

bool read_exact(std::ifstream &input, char *destination,
                std::size_t size) {
  if (size > static_cast<std::size_t>(
                 std::numeric_limits<std::streamsize>::max())) {
    return false;
  }
  input.read(destination, static_cast<std::streamsize>(size));
  return input.good() ||
         (input.eof() && input.gcount() == static_cast<std::streamsize>(size));
}

bool file_size(const fs::path &path, std::uint64_t &size,
               std::string &error) {
  boost::system::error_code code;
  const std::uintmax_t value = fs::file_size(path, code);
  if (code) {
    error = "cannot determine file size for " + path.string() + ": " +
            code.message();
    return false;
  }
  if (value > std::numeric_limits<std::uint64_t>::max()) {
    error = "file is too large: " + path.string();
    return false;
  }
  size = static_cast<std::uint64_t>(value);
  return true;
}

bool read_whole_file(const fs::path &path, std::uint64_t maximum_size,
                     std::vector<char> &data, std::string &error) {
  std::uint64_t size = 0U;
  if (!file_size(path, size, error)) {
    return false;
  }
  if (size > maximum_size ||
      size > static_cast<std::uint64_t>(
                 std::numeric_limits<std::size_t>::max())) {
    error = "file exceeds the supported size: " + path.string();
    return false;
  }
  std::ifstream input(path.string(), std::ios::binary);
  if (!input) {
    error = "cannot open file: " + path.string();
    return false;
  }
  std::vector<char> candidate(static_cast<std::size_t>(size));
  if (!candidate.empty() &&
      !read_exact(input, candidate.data(), candidate.size())) {
    error = "failed to read complete file: " + path.string();
    return false;
  }
  data = std::move(candidate);
  return true;
}

bool read_data_header(const fs::path &path, std::uint64_t &size,
                      std::string &error) {
  if (!file_size(path, size, error)) {
    return false;
  }
  if (size < kDataHeaderSize) {
    error = "truncated spatial block data header";
    return false;
  }
  std::ifstream input(path.string(), std::ios::binary);
  if (!input) {
    error = "cannot open spatial block data file: " + path.string();
    return false;
  }
  std::array<char, kDataHeaderSize> header{};
  if (!read_exact(input, header.data(), header.size())) {
    error = "failed to read spatial block data header";
    return false;
  }
  ByteReader reader(header.data(), header.size());
  return read_common_header(reader, kDataMagic, kDataHeaderSize,
                            "spatial block data", error) &&
         reader.remaining() == 0U;
}

bool bounds_match(const BlockAabb &left, const BlockAabb &right) {
  return (left.minimum.array() == right.minimum.array()).all() &&
         (left.maximum.array() == right.maximum.array()).all();
}

bool load_catalog_paths(const fs::path &index_path,
                        const fs::path &data_path,
                        SpatialMapCatalog &catalog,
                        std::string &error) {
  error.clear();
  std::uint64_t maximum_index_size = 0U;
  std::uint64_t records_size = 0U;
  if (!safe_multiply(kMaxBlockCount, kIndexRecordSize, records_size) ||
      !safe_add(kIndexHeaderSize, records_size, maximum_index_size)) {
    error = "internal index size limit overflow";
    return false;
  }
  std::vector<char> bytes;
  if (!read_whole_file(index_path, maximum_index_size, bytes, error)) {
    return false;
  }
  if (bytes.size() < kIndexHeaderSize) {
    error = "truncated spatial block index header";
    return false;
  }
  ByteReader reader(bytes.data(), bytes.size());
  if (!read_common_header(reader, kIndexMagic, kIndexHeaderSize,
                          "spatial block index", error)) {
    return false;
  }

  SpatialMapCatalog candidate;
  if (!read_config(reader, candidate.config, error)) {
    return false;
  }
  std::uint64_t entry_count = 0U;
  std::uint64_t recorded_data_size = 0U;
  if (!reader.u64(entry_count) || !reader.u64(recorded_data_size)) {
    error = "truncated spatial block index counts";
    return false;
  }
  if (reader.position() != kIndexHeaderSize) {
    error = "invalid spatial block index header layout";
    return false;
  }
  if (entry_count > kMaxBlockCount) {
    error = "spatial block count exceeds the format safety limit";
    return false;
  }
  std::uint64_t expected_index_size = 0U;
  if (!safe_multiply(entry_count, kIndexRecordSize, records_size) ||
      !safe_add(kIndexHeaderSize, records_size, expected_index_size) ||
      expected_index_size != bytes.size()) {
    error = "spatial block index size does not match its entry count";
    return false;
  }

  std::uint64_t actual_data_size = 0U;
  if (!read_data_header(data_path, actual_data_size, error)) {
    return false;
  }
  if (recorded_data_size != actual_data_size) {
    error = "spatial block data size does not match the index";
    return false;
  }

  candidate.entries.reserve(static_cast<std::size_t>(entry_count));
  candidate.lookup.reserve(static_cast<std::size_t>(entry_count));
  for (std::uint64_t i = 0U; i < entry_count; ++i) {
    SpatialBlockIndexEntry entry;
    if (!read_block_index(reader, entry.index) ||
        !read_vector(reader, entry.bounds.minimum) ||
        !read_vector(reader, entry.bounds.maximum) ||
        !reader.u64(entry.offset) || !reader.u64(entry.length) ||
        !reader.u32(entry.checksum) || !reader.u32(entry.root_count) ||
        !reader.u32(entry.node_count) || !reader.u32(entry.plane_count) ||
        !reader.u32(entry.display_point_count)) {
      error = "truncated spatial block index record";
      return false;
    }
    std::string count_error;
    std::uint64_t expected_length = 0U;
    if (!expected_payload_size(entry.root_count, entry.node_count,
                               entry.plane_count,
                               entry.display_point_count,
                               expected_length, count_error)) {
      error = "block " + block_label(entry.index) + ": " + count_error;
      return false;
    }
    if (entry.length != expected_length) {
      error = "indexed length does not match counts for block " +
              block_label(entry.index);
      return false;
    }
    std::uint64_t end = 0U;
    if (!safe_add(entry.offset, entry.length, end)) {
      error = "offset plus length overflow for block " +
              block_label(entry.index);
      return false;
    }
    if (entry.offset < kDataHeaderSize || end > actual_data_size) {
      error = "payload range is outside the data file for block " +
              block_label(entry.index);
      return false;
    }
    BlockAabb expected_bounds;
    try {
      expected_bounds = block_aabb(entry.index, candidate.config);
    } catch (const std::exception &exception) {
      error = "cannot calculate bounds for block " +
              block_label(entry.index) + ": " + exception.what();
      return false;
    }
    if (!vector_is_finite(entry.bounds.minimum) ||
        !vector_is_finite(entry.bounds.maximum) ||
        !bounds_match(entry.bounds, expected_bounds)) {
      error = "AABB does not match block index " +
              block_label(entry.index);
      return false;
    }
    if (!candidate.entries.empty() &&
        !(candidate.entries.back().index < entry.index)) {
      error = "spatial block indices are duplicate or not sorted";
      return false;
    }
    if (!candidate.lookup.emplace(entry.index,
                                  candidate.entries.size()).second) {
      error = "duplicate spatial block index " + block_label(entry.index);
      return false;
    }
    candidate.entries.push_back(entry);
  }
  if (reader.remaining() != 0U) {
    error = "trailing bytes in spatial block index";
    return false;
  }

  std::vector<std::pair<std::uint64_t, std::uint64_t>> ranges;
  ranges.reserve(candidate.entries.size());
  for (const SpatialBlockIndexEntry &entry : candidate.entries) {
    ranges.emplace_back(entry.offset, entry.offset + entry.length);
  }
  std::sort(ranges.begin(), ranges.end());
  std::uint64_t expected_offset = kDataHeaderSize;
  for (const auto &range : ranges) {
    if (range.first != expected_offset) {
      error = range.first < expected_offset
                  ? "spatial block payload ranges overlap"
                  : "spatial block data contains an unindexed gap";
      return false;
    }
    expected_offset = range.second;
  }
  if (expected_offset != actual_data_size) {
    error = "spatial block data contains trailing unindexed bytes";
    return false;
  }

  catalog = std::move(candidate);
  return true;
}

bool load_block_path(const fs::path &data_path,
                     const SpatialMapCatalog &catalog,
                     const BlockIndex &index, StaticMapBlock &block,
                     std::string &error) {
  error.clear();
  std::string config_error;
  if (!catalog.config.validate(config_error)) {
    error = "invalid catalog configuration: " + config_error;
    return false;
  }
  const auto found = catalog.lookup.find(index);
  if (found == catalog.lookup.end()) {
    error = "spatial block " + block_label(index) +
            " is not present in the catalog";
    return false;
  }
  if (found->second >= catalog.entries.size() ||
      catalog.entries[found->second].index != index) {
    error = "catalog lookup is inconsistent for block " +
            block_label(index);
    return false;
  }
  const SpatialBlockIndexEntry &entry = catalog.entries[found->second];
  std::uint64_t data_size = 0U;
  if (!read_data_header(data_path, data_size, error)) {
    return false;
  }
  std::uint64_t end = 0U;
  if (!safe_add(entry.offset, entry.length, end)) {
    error = "offset plus length overflow for block " + block_label(index);
    return false;
  }
  if (entry.offset < kDataHeaderSize || end > data_size) {
    error = "payload range is outside the data file for block " +
            block_label(index);
    return false;
  }
  std::uint64_t expected_length = 0U;
  std::string size_error;
  if (!expected_payload_size(entry.root_count, entry.node_count,
                             entry.plane_count,
                             entry.display_point_count,
                             expected_length, size_error) ||
      expected_length != entry.length) {
    error = "invalid indexed payload size for block " + block_label(index) +
            (size_error.empty() ? std::string() : ": " + size_error);
    return false;
  }

  std::ifstream input(data_path.string(), std::ios::binary);
  if (!input) {
    error = "cannot open spatial block data file: " + data_path.string();
    return false;
  }
  if (entry.offset > static_cast<std::uint64_t>(
                         std::numeric_limits<std::streamoff>::max())) {
    error = "payload offset is too large for this platform for block " +
            block_label(index);
    return false;
  }
  input.seekg(static_cast<std::streamoff>(entry.offset), std::ios::beg);
  if (!input) {
    error = "failed to seek to block " + block_label(index);
    return false;
  }
  std::array<char, 65536U> crc_buffer{};
  std::uint64_t remaining = entry.length;
  std::uint32_t streaming_crc = UINT32_C(0xffffffff);
  while (remaining != 0U) {
    const std::size_t chunk = static_cast<std::size_t>(
        std::min<std::uint64_t>(remaining, crc_buffer.size()));
    if (!read_exact(input, crc_buffer.data(), chunk)) {
      error = "truncated payload for block " + block_label(index);
      return false;
    }
    streaming_crc = crc32_update(streaming_crc, crc_buffer.data(), chunk);
    remaining -= chunk;
  }
  if (~streaming_crc != entry.checksum) {
    error = "CRC mismatch for block " + block_label(index);
    return false;
  }

  input.clear();
  input.seekg(static_cast<std::streamoff>(entry.offset), std::ios::beg);
  if (!input) {
    error = "failed to reseek to block " + block_label(index);
    return false;
  }
  std::vector<char> payload(static_cast<std::size_t>(entry.length));
  if (!payload.empty() &&
      !read_exact(input, payload.data(), payload.size())) {
    error = "truncated payload for block " + block_label(index);
    return false;
  }
  if (crc32(payload.data(), payload.size()) != entry.checksum) {
    error = "CRC changed while reading block " + block_label(index);
    return false;
  }

  StaticMapBlock candidate;
  if (!deserialize_block(payload, entry, catalog.config, candidate, error)) {
    return false;
  }
  block = std::move(candidate);
  return true;
}

bool validate_all_paths(const fs::path &data_path,
                        const SpatialMapCatalog &catalog,
                        std::string &error) {
  for (const SpatialBlockIndexEntry &entry : catalog.entries) {
    StaticMapBlock block;
    if (!load_block_path(data_path, catalog, entry.index, block, error)) {
      if (error.find(block_label(entry.index)) == std::string::npos) {
        error = "block " + block_label(entry.index) + ": " + error;
      }
      return false;
    }
  }
  error.clear();
  return true;
}

bool write_file(const fs::path &path, const std::vector<char> &data,
                std::string &error) {
  std::ofstream output(path.string(),
                       std::ios::binary | std::ios::trunc);
  if (!output) {
    error = "cannot create temporary file: " + path.string();
    return false;
  }
  if (!data.empty()) {
    output.write(data.data(), static_cast<std::streamsize>(data.size()));
  }
  output.flush();
  if (!output) {
    error = "failed to write temporary file: " + path.string();
    return false;
  }
  output.close();
  if (!output) {
    error = "failed to close temporary file: " + path.string();
    return false;
  }
  return true;
}

std::string unique_suffix() {
  static std::atomic<std::uint64_t> counter{0U};
  const std::uint64_t ticks = static_cast<std::uint64_t>(
      std::chrono::high_resolution_clock::now().time_since_epoch().count());
  return std::to_string(ticks) + "." +
         std::to_string(counter.fetch_add(1U));
}

class PathCleanup {
 public:
  void add(const fs::path &path) { paths_.push_back(path); }
  void release() { paths_.clear(); }
  ~PathCleanup() {
    for (const fs::path &path : paths_) {
      boost::system::error_code ignored;
      fs::remove(path, ignored);
    }
  }

 private:
  std::vector<fs::path> paths_;
};

bool publish_pair(const fs::path &temporary_index,
                  const fs::path &temporary_data,
                  const fs::path &target_index,
                  const fs::path &target_data,
                  std::string &error) {
  const std::string suffix = unique_suffix();
  const fs::path backup_index(
      target_index.string() + ".backup." + suffix);
  const fs::path backup_data(target_data.string() + ".backup." + suffix);
  bool index_backed_up = false;
  bool data_backed_up = false;
  bool new_index_published = false;
  bool new_data_published = false;
  try {
    if (fs::exists(target_index)) {
      fs::rename(target_index, backup_index);
      index_backed_up = true;
    }
    if (fs::exists(target_data)) {
      fs::rename(target_data, backup_data);
      data_backed_up = true;
    }
    fs::rename(temporary_data, target_data);
    new_data_published = true;
    fs::rename(temporary_index, target_index);
    new_index_published = true;

    boost::system::error_code ignored;
    if (index_backed_up) {
      fs::remove(backup_index, ignored);
    }
    ignored.clear();
    if (data_backed_up) {
      fs::remove(backup_data, ignored);
    }
    return true;
  } catch (const fs::filesystem_error &exception) {
    error = std::string("failed to publish spatial block files: ") +
            exception.what();
  }

  boost::system::error_code ignored;
  if (new_index_published) {
    fs::remove(target_index, ignored);
  }
  ignored.clear();
  if (new_data_published) {
    fs::remove(target_data, ignored);
  }
  try {
    if (data_backed_up && !fs::exists(target_data)) {
      fs::rename(backup_data, target_data);
      data_backed_up = false;
    }
    if (index_backed_up && !fs::exists(target_index)) {
      fs::rename(backup_index, target_index);
      index_backed_up = false;
    }
  } catch (const fs::filesystem_error &exception) {
    error += std::string("; failed to restore previous files: ") +
             exception.what() + "; backups retained at " +
             backup_index.string() + " and " + backup_data.string();
  }
  return false;
}

}  // namespace

bool SpatialMapIO::save(const std::string &directory,
                        const std::vector<StaticMapBlock> &blocks,
                        const SpatialMapConfig &config,
                        SpatialMapCatalog &catalog, std::string &error) {
  error.clear();
  std::string config_error;
  if (!config.validate(config_error)) {
    error = "invalid spatial map configuration: " + config_error;
    return false;
  }
  if (blocks.size() > kMaxBlockCount) {
    error = "spatial block count exceeds the format safety limit";
    return false;
  }

  try {
    std::vector<StaticMapBlock> ordered_blocks = blocks;
    for (StaticMapBlock &block : ordered_blocks) {
      if (!validate_block(block, config, error)) {
        return false;
      }
      std::sort(block.roots.begin(), block.roots.end(),
                [](const RootVoxelRecord &left,
                   const RootVoxelRecord &right) {
                  return left.index < right.index;
                });
      std::sort(block.display_points.begin(), block.display_points.end(),
                point_less);
    }
    std::sort(ordered_blocks.begin(), ordered_blocks.end(),
              [](const StaticMapBlock &left, const StaticMapBlock &right) {
                return left.index < right.index;
              });
    for (std::size_t i = 1; i < ordered_blocks.size(); ++i) {
      if (ordered_blocks[i - 1].index == ordered_blocks[i].index) {
        error = "duplicate spatial block index " +
                block_label(ordered_blocks[i].index);
        return false;
      }
    }

    ByteWriter data_writer;
    write_common_header(data_writer, kDataMagic, kDataHeaderSize);
    SpatialMapCatalog candidate;
    candidate.config = config;
    candidate.entries.reserve(ordered_blocks.size());
    candidate.lookup.reserve(ordered_blocks.size());
    for (const StaticMapBlock &block : ordered_blocks) {
      const std::vector<char> payload = serialize_block(block);
      std::uint64_t expected_size = 0U;
      std::string size_error;
      if (!expected_payload_size(
              static_cast<std::uint32_t>(block.roots.size()),
              static_cast<std::uint32_t>(block.nodes.size()),
              static_cast<std::uint32_t>(block.planes.size()),
              static_cast<std::uint32_t>(block.display_points.size()),
              expected_size, size_error) ||
          expected_size != payload.size()) {
        error = "failed to size block " + block_label(block.index) +
                ": " + size_error;
        return false;
      }
      SpatialBlockIndexEntry entry;
      entry.index = block.index;
      entry.bounds = block_aabb(block.index, config);
      entry.offset = data_writer.data().size();
      entry.length = payload.size();
      entry.checksum = crc32(payload.data(), payload.size());
      entry.root_count = static_cast<std::uint32_t>(block.roots.size());
      entry.node_count = static_cast<std::uint32_t>(block.nodes.size());
      entry.plane_count = static_cast<std::uint32_t>(block.planes.size());
      entry.display_point_count =
          static_cast<std::uint32_t>(block.display_points.size());
      candidate.lookup.emplace(entry.index, candidate.entries.size());
      candidate.entries.push_back(entry);
      data_writer.append(payload);
    }
    std::vector<char> data_bytes = data_writer.release();

    ByteWriter index_writer;
    write_common_header(index_writer, kIndexMagic, kIndexHeaderSize);
    write_config(index_writer, config);
    index_writer.u64(candidate.entries.size());
    index_writer.u64(data_bytes.size());
    for (const SpatialBlockIndexEntry &entry : candidate.entries) {
      write_block_index(index_writer, entry.index);
      write_vector(index_writer, entry.bounds.minimum);
      write_vector(index_writer, entry.bounds.maximum);
      index_writer.u64(entry.offset);
      index_writer.u64(entry.length);
      index_writer.u32(entry.checksum);
      index_writer.u32(entry.root_count);
      index_writer.u32(entry.node_count);
      index_writer.u32(entry.plane_count);
      index_writer.u32(entry.display_point_count);
    }
    std::vector<char> index_bytes = index_writer.release();

    const fs::path output_directory(directory);
    boost::system::error_code code;
    if (!fs::exists(output_directory, code)) {
      if (code || !fs::create_directories(output_directory, code) || code) {
        error = "cannot create spatial map directory: " + directory;
        return false;
      }
    }
    if (!fs::is_directory(output_directory, code) || code) {
      error = "spatial map path is not a directory: " + directory;
      return false;
    }

    const std::string suffix = unique_suffix();
    const fs::path temporary_index =
        output_directory / ("spatial_blocks.idx.tmp." + suffix);
    const fs::path temporary_data =
        output_directory / ("spatial_blocks.bin.tmp." + suffix);
    PathCleanup cleanup;
    cleanup.add(temporary_index);
    cleanup.add(temporary_data);
    if (!write_file(temporary_data, data_bytes, error) ||
        !write_file(temporary_index, index_bytes, error)) {
      return false;
    }

    SpatialMapCatalog verified;
    if (!load_catalog_paths(temporary_index, temporary_data, verified,
                            error) ||
        !validate_all_paths(temporary_data, verified, error)) {
      error = "temporary spatial map validation failed: " + error;
      return false;
    }

    const fs::path target_index =
        output_directory / "spatial_blocks.idx";
    const fs::path target_data =
        output_directory / "spatial_blocks.bin";
    if (!publish_pair(temporary_index, temporary_data,
                      target_index, target_data, error)) {
      return false;
    }
    cleanup.release();
    catalog = std::move(verified);
    return true;
  } catch (const std::exception &exception) {
    error = std::string("failed to save spatial block map: ") +
            exception.what();
    return false;
  }
}

bool SpatialMapIO::load_catalog(const std::string &directory,
                                SpatialMapCatalog &catalog,
                                std::string &error) {
  try {
    const fs::path root(directory);
    return load_catalog_paths(root / "spatial_blocks.idx",
                              root / "spatial_blocks.bin", catalog, error);
  } catch (const std::exception &exception) {
    error = std::string("failed to load spatial map catalog: ") +
            exception.what();
    return false;
  }
}

bool SpatialMapIO::load_block(const std::string &directory,
                              const SpatialMapCatalog &catalog,
                              const BlockIndex &index,
                              StaticMapBlock &block,
                              std::string &error) {
  try {
    return load_block_path(fs::path(directory) / "spatial_blocks.bin",
                           catalog, index, block, error);
  } catch (const std::exception &exception) {
    error = "failed to load block " + block_label(index) + ": " +
            exception.what();
    return false;
  }
}

bool SpatialMapIO::validate_all_blocks(const std::string &directory,
                                       const SpatialMapCatalog &catalog,
                                       std::string &error) {
  try {
    return validate_all_paths(fs::path(directory) / "spatial_blocks.bin",
                              catalog, error);
  } catch (const std::exception &exception) {
    error = std::string("failed to validate spatial block map: ") +
            exception.what();
    return false;
  }
}
