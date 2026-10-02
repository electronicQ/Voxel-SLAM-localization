#include "btc_io.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <new>
#include <string>
#include <utility>
#include <vector>

namespace {

static_assert(sizeof(double) == 8, "BTC codec requires 8-byte double");
static_assert(std::numeric_limits<double>::is_iec559,
              "BTC codec requires IEEE-754 double");

const char kMagic[] = "VXLBTC01";
const std::uint32_t kVersion = 1;
const std::uint32_t kMaximumCount = 10000000U;
const std::size_t kMagicSize = 8;
const std::size_t kFrameHeaderSize = 12;
const std::size_t kBinaryDescriptorMinimumSize = 1 + 3 * 8 + 4;
const std::size_t kStdMinimumSize =
    9 * 8 + 4 + 3 * kBinaryDescriptorMinimumSize;

bool fail(std::string &error, const std::string &message) {
  error = message;
  return false;
}

class BinaryWriter {
 public:
  explicit BinaryWriter(std::ostream &output) : output_(output) {}

  bool writeBytes(const char *bytes, std::size_t count) {
    output_.write(bytes, static_cast<std::streamsize>(count));
    return static_cast<bool>(output_);
  }

  bool writeU8(std::uint8_t value) {
    output_.put(static_cast<char>(value));
    return static_cast<bool>(output_);
  }

  bool writeU32(std::uint32_t value) {
    for (std::size_t index = 0; index < 4; ++index) {
      if (!writeU8(static_cast<std::uint8_t>((value >> (index * 8)) & 0xffU))) {
        return false;
      }
    }
    return true;
  }

  bool writeI32(std::int32_t value) {
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    return writeU32(bits);
  }

  bool writeU64(std::uint64_t value) {
    for (std::size_t index = 0; index < 8; ++index) {
      if (!writeU8(static_cast<std::uint8_t>((value >> (index * 8)) &
                                             UINT64_C(0xff)))) {
        return false;
      }
    }
    return true;
  }

  bool writeDouble(double value) {
    std::uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    return writeU64(bits);
  }

 private:
  std::ostream &output_;
};

class BinaryReader {
 public:
  BinaryReader(std::istream &input, std::size_t size)
      : input_(input), remaining_(size) {}

  std::size_t remaining() const { return remaining_; }

  bool readBytes(char *bytes, std::size_t count) {
    if (count > remaining_ ||
        count > static_cast<std::size_t>(
                    std::numeric_limits<std::streamsize>::max())) {
      return false;
    }
    input_.read(bytes, static_cast<std::streamsize>(count));
    if (!input_) {
      return false;
    }
    remaining_ -= count;
    return true;
  }

  bool readU8(std::uint8_t &value) {
    char byte = 0;
    if (!readBytes(&byte, 1)) {
      return false;
    }
    value = static_cast<std::uint8_t>(static_cast<unsigned char>(byte));
    return true;
  }

  bool readU32(std::uint32_t &value) {
    value = 0;
    for (std::size_t index = 0; index < 4; ++index) {
      std::uint8_t byte = 0;
      if (!readU8(byte)) {
        return false;
      }
      value |= static_cast<std::uint32_t>(byte) << (index * 8);
    }
    return true;
  }

  bool readI32(std::int32_t &value) {
    std::uint32_t bits = 0;
    if (!readU32(bits)) {
      return false;
    }
    std::memcpy(&value, &bits, sizeof(value));
    return true;
  }

  bool readU64(std::uint64_t &value) {
    value = 0;
    for (std::size_t index = 0; index < 8; ++index) {
      std::uint8_t byte = 0;
      if (!readU8(byte)) {
        return false;
      }
      value |= static_cast<std::uint64_t>(byte) << (index * 8);
    }
    return true;
  }

  bool readDouble(double &value) {
    std::uint64_t bits = 0;
    if (!readU64(bits)) {
      return false;
    }
    std::memcpy(&value, &bits, sizeof(value));
    return true;
  }

 private:
  std::istream &input_;
  std::size_t remaining_;
};

bool vectorIsFinite(const Eigen::Vector3d &value) {
  return value.allFinite();
}

bool binaryIsFinite(const BinaryDescriptor &descriptor) {
  return vectorIsFinite(descriptor.location_);
}

bool descriptorIsFinite(const STD &descriptor) {
  return vectorIsFinite(descriptor.triangle_) &&
         vectorIsFinite(descriptor.angle_) &&
         vectorIsFinite(descriptor.center_) &&
         binaryIsFinite(descriptor.binary_A_) &&
         binaryIsFinite(descriptor.binary_B_) &&
         binaryIsFinite(descriptor.binary_C_);
}

bool validateBinaryForSave(const BinaryDescriptor &descriptor,
                           const std::string &context, std::string &error) {
  if (!binaryIsFinite(descriptor)) {
    return fail(error, context + " has a non-finite binary location");
  }
  if (descriptor.occupy_array_.size() > kMaximumCount ||
      descriptor.occupy_array_.size() >
          static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())) {
    return fail(error, context + " bit count exceeds the 10000000 limit");
  }
  return true;
}

bool validateDescriptorForSave(const STD &descriptor,
                               const std::string &context,
                               std::string &error) {
  if (!descriptorIsFinite(descriptor)) {
    return fail(error, context + " contains a non-finite value");
  }
  if (static_cast<std::int64_t>(descriptor.frame_number_) <
          std::numeric_limits<std::int32_t>::min() ||
      static_cast<std::int64_t>(descriptor.frame_number_) >
          std::numeric_limits<std::int32_t>::max()) {
    return fail(error, context + " frame number is outside int32 range");
  }
  return validateBinaryForSave(descriptor.binary_A_, context + " binary A",
                               error) &&
         validateBinaryForSave(descriptor.binary_B_, context + " binary B",
                               error) &&
         validateBinaryForSave(descriptor.binary_C_, context + " binary C",
                               error);
}

bool validateFramesForSave(const std::vector<SavedBtcFrame> &frames,
                           std::string &error) {
  if (frames.size() > kMaximumCount ||
      frames.size() > static_cast<std::size_t>(
                          std::numeric_limits<std::uint32_t>::max())) {
    return fail(error, "BTC frame count exceeds the 10000000 limit");
  }
  for (std::size_t frame_index = 0; frame_index < frames.size();
       ++frame_index) {
    const SavedBtcFrame &frame = frames[frame_index];
    if (frame.frame_id != static_cast<int>(frame_index)) {
      return fail(error,
                  "BTC frame IDs must be contiguous from 0; expected " +
                      std::to_string(frame_index) + ", found " +
                      std::to_string(frame.frame_id));
    }
    if (frame.anchor_keyframe_id < 0) {
      return fail(error, "BTC frame " + std::to_string(frame_index) +
                             " has a negative anchor keyframe ID");
    }
    if (frame.descriptors.size() > kMaximumCount ||
        frame.descriptors.size() >
            static_cast<std::size_t>(
                std::numeric_limits<std::uint32_t>::max())) {
      return fail(error, "BTC frame " + std::to_string(frame_index) +
                             " descriptor count exceeds the 10000000 limit");
    }
    for (std::size_t descriptor_index = 0;
         descriptor_index < frame.descriptors.size(); ++descriptor_index) {
      if (!validateDescriptorForSave(
              frame.descriptors[descriptor_index],
              "BTC frame " + std::to_string(frame_index) + " descriptor " +
                  std::to_string(descriptor_index),
              error)) {
        return false;
      }
    }
  }
  return true;
}

bool writeVector(BinaryWriter &writer, const Eigen::Vector3d &value) {
  return writer.writeDouble(value.x()) && writer.writeDouble(value.y()) &&
         writer.writeDouble(value.z());
}

bool writeBinaryDescriptor(BinaryWriter &writer,
                           const BinaryDescriptor &descriptor) {
  if (!writer.writeU8(descriptor.summary_) ||
      !writeVector(writer, descriptor.location_) ||
      !writer.writeU32(
          static_cast<std::uint32_t>(descriptor.occupy_array_.size()))) {
    return false;
  }
  const std::size_t byte_count = (descriptor.occupy_array_.size() + 7) / 8;
  for (std::size_t byte_index = 0; byte_index < byte_count; ++byte_index) {
    std::uint8_t packed = 0;
    for (std::size_t bit_index = 0; bit_index < 8; ++bit_index) {
      const std::size_t index = byte_index * 8 + bit_index;
      if (index < descriptor.occupy_array_.size() &&
          descriptor.occupy_array_[index]) {
        packed |= static_cast<std::uint8_t>(1U << bit_index);
      }
    }
    if (!writer.writeU8(packed)) {
      return false;
    }
  }
  return true;
}

bool writeDescriptor(BinaryWriter &writer, const STD &descriptor) {
  return writeVector(writer, descriptor.triangle_) &&
         writeVector(writer, descriptor.angle_) &&
         writeVector(writer, descriptor.center_) &&
         writer.writeI32(static_cast<std::int32_t>(descriptor.frame_number_)) &&
         writeBinaryDescriptor(writer, descriptor.binary_A_) &&
         writeBinaryDescriptor(writer, descriptor.binary_B_) &&
         writeBinaryDescriptor(writer, descriptor.binary_C_);
}

bool readVector(BinaryReader &reader, Eigen::Vector3d &value,
                const std::string &context, std::string &error) {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  if (!reader.readDouble(x) || !reader.readDouble(y) ||
      !reader.readDouble(z)) {
    return fail(error, "truncated BTC file while reading " + context);
  }
  value = Eigen::Vector3d(x, y, z);
  if (!vectorIsFinite(value)) {
    return fail(error, "non-finite value in " + context);
  }
  return true;
}

bool readBinaryDescriptor(BinaryReader &reader, BinaryDescriptor &descriptor,
                          const std::string &context, std::string &error) {
  std::uint8_t summary = 0;
  if (!reader.readU8(summary)) {
    return fail(error, "truncated BTC file while reading " + context +
                           " summary");
  }
  descriptor.summary_ = summary;
  if (!readVector(reader, descriptor.location_, context + " location", error)) {
    return false;
  }

  std::uint32_t bit_count = 0;
  if (!reader.readU32(bit_count)) {
    return fail(error, "truncated BTC file while reading " + context +
                           " bit count");
  }
  if (bit_count > kMaximumCount) {
    return fail(error, context + " bit count exceeds the 10000000 limit");
  }
  const std::size_t packed_size =
      (static_cast<std::size_t>(bit_count) + 7) / 8;
  if (packed_size > reader.remaining()) {
    return fail(error, "truncated BTC file while reading " + context +
                           " packed bits");
  }

  descriptor.occupy_array_.assign(bit_count, false);
  for (std::size_t byte_index = 0; byte_index < packed_size; ++byte_index) {
    std::uint8_t packed = 0;
    if (!reader.readU8(packed)) {
      return fail(error, "truncated BTC file while reading " + context +
                             " packed bits");
    }
    for (std::size_t bit_index = 0; bit_index < 8; ++bit_index) {
      const std::size_t index = byte_index * 8 + bit_index;
      if (index < descriptor.occupy_array_.size()) {
        descriptor.occupy_array_[index] =
            (packed & static_cast<std::uint8_t>(1U << bit_index)) != 0;
      }
    }
  }
  return true;
}

bool readDescriptor(BinaryReader &reader, STD &descriptor,
                    const std::string &context, std::string &error) {
  if (!readVector(reader, descriptor.triangle_, context + " triangle", error) ||
      !readVector(reader, descriptor.angle_, context + " angle", error) ||
      !readVector(reader, descriptor.center_, context + " center", error)) {
    return false;
  }
  std::int32_t frame_number = 0;
  if (!reader.readI32(frame_number)) {
    return fail(error, "truncated BTC file while reading " + context +
                           " frame number");
  }
  descriptor.frame_number_ = frame_number;
  return readBinaryDescriptor(reader, descriptor.binary_A_,
                              context + " binary A", error) &&
         readBinaryDescriptor(reader, descriptor.binary_B_,
                              context + " binary B", error) &&
         readBinaryDescriptor(reader, descriptor.binary_C_,
                              context + " binary C", error);
}

bool streamSize(std::ifstream &input, std::size_t &size, std::string &error) {
  input.seekg(0, std::ios::end);
  const std::streamoff end = input.tellg();
  if (end < 0 ||
      static_cast<std::uintmax_t>(end) >
          static_cast<std::uintmax_t>(
              std::numeric_limits<std::size_t>::max())) {
    return fail(error, "cannot determine BTC file size");
  }
  size = static_cast<std::size_t>(end);
  input.seekg(0, std::ios::beg);
  if (!input) {
    return fail(error, "cannot seek to the start of the BTC file");
  }
  return true;
}

bool validateRestoreFrames(const std::vector<SavedBtcFrame> &frames,
                           std::vector<const SavedBtcFrame *> &ordered,
                           std::string &error) {
  if (frames.size() > kMaximumCount) {
    return fail(error, "BTC frame count exceeds the 10000000 limit");
  }
  ordered.reserve(frames.size());
  for (const SavedBtcFrame &frame : frames) {
    ordered.push_back(&frame);
  }
  std::sort(ordered.begin(), ordered.end(),
            [](const SavedBtcFrame *left, const SavedBtcFrame *right) {
              return left->frame_id < right->frame_id;
            });

  for (std::size_t frame_index = 0; frame_index < ordered.size();
       ++frame_index) {
    const SavedBtcFrame &frame = *ordered[frame_index];
    if (frame.frame_id != static_cast<int>(frame_index)) {
      return fail(error,
                  "BTC frame IDs must be contiguous from 0; expected " +
                      std::to_string(frame_index) + ", found " +
                      std::to_string(frame.frame_id));
    }
    if (frame.anchor_keyframe_id < 0) {
      return fail(error, "BTC frame " + std::to_string(frame_index) +
                             " has a negative anchor keyframe ID");
    }
    if (!frame.plane_cloud) {
      return fail(error, "BTC frame " + std::to_string(frame_index) +
                             " has no plane cloud");
    }
    if (frame.descriptors.size() > kMaximumCount) {
      return fail(error, "BTC frame " + std::to_string(frame_index) +
                             " descriptor count exceeds the 10000000 limit");
    }
    for (std::size_t descriptor_index = 0;
         descriptor_index < frame.descriptors.size(); ++descriptor_index) {
      const STD &descriptor = frame.descriptors[descriptor_index];
      if (!descriptorIsFinite(descriptor)) {
        return fail(error, "BTC frame " + std::to_string(frame_index) +
                               " descriptor " +
                               std::to_string(descriptor_index) +
                               " contains a non-finite value");
      }
      if (descriptor.binary_A_.occupy_array_.size() > kMaximumCount ||
          descriptor.binary_B_.occupy_array_.size() > kMaximumCount ||
          descriptor.binary_C_.occupy_array_.size() > kMaximumCount) {
        return fail(error, "BTC frame " + std::to_string(frame_index) +
                               " descriptor bit count exceeds the "
                               "10000000 limit");
      }
    }
  }
  return true;
}

}  // namespace

bool save_btc_frames(const std::string &path,
                     const std::vector<SavedBtcFrame> &frames,
                     std::string &error) {
  error.clear();
  try {
    if (path.empty()) {
      return fail(error, "BTC output path is empty");
    }
    if (!validateFramesForSave(frames, error)) {
      return false;
    }

    std::ofstream output(path.c_str(), std::ios::binary | std::ios::trunc);
    if (!output) {
      return fail(error, "cannot create BTC file: " + path);
    }
    BinaryWriter writer(output);
    if (!writer.writeBytes(kMagic, kMagicSize) ||
        !writer.writeU32(kVersion) ||
        !writer.writeU32(static_cast<std::uint32_t>(frames.size()))) {
      return fail(error, "failed writing BTC header: " + path);
    }

    for (const SavedBtcFrame &frame : frames) {
      if (!writer.writeU32(static_cast<std::uint32_t>(frame.frame_id)) ||
          !writer.writeU32(
              static_cast<std::uint32_t>(frame.anchor_keyframe_id)) ||
          !writer.writeU32(
              static_cast<std::uint32_t>(frame.descriptors.size()))) {
        return fail(error, "failed writing BTC frame header " +
                               std::to_string(frame.frame_id));
      }
      for (std::size_t descriptor_index = 0;
           descriptor_index < frame.descriptors.size(); ++descriptor_index) {
        if (!writeDescriptor(writer, frame.descriptors[descriptor_index])) {
          return fail(error, "failed writing BTC frame " +
                                 std::to_string(frame.frame_id) +
                                 " descriptor " +
                                 std::to_string(descriptor_index));
        }
      }
    }
    output.close();
    if (!output) {
      return fail(error, "failed closing BTC file: " + path);
    }
    return true;
  } catch (const std::exception &exception) {
    return fail(error, std::string("BTC save failed: ") + exception.what());
  }
}

bool load_btc_frames(const std::string &path,
                     std::vector<SavedBtcFrame> &frames,
                     std::string &error) {
  error.clear();
  try {
    std::ifstream input(path.c_str(), std::ios::binary);
    if (!input) {
      return fail(error, "cannot open BTC file: " + path);
    }
    std::size_t file_size = 0;
    if (!streamSize(input, file_size, error)) {
      return false;
    }
    BinaryReader reader(input, file_size);

    char magic[kMagicSize] = {};
    if (!reader.readBytes(magic, kMagicSize)) {
      return fail(error, "truncated BTC file while reading magic");
    }
    if (std::memcmp(magic, kMagic, kMagicSize) != 0) {
      return fail(error, "invalid BTC magic; expected VXLBTC01");
    }

    std::uint32_t version = 0;
    std::uint32_t frame_count = 0;
    if (!reader.readU32(version) || !reader.readU32(frame_count)) {
      return fail(error, "truncated BTC file while reading header");
    }
    if (version != kVersion) {
      return fail(error, "unsupported BTC version: " +
                             std::to_string(version));
    }
    if (frame_count > kMaximumCount) {
      return fail(error, "BTC frame count exceeds the 10000000 limit");
    }
    if (frame_count > reader.remaining() / kFrameHeaderSize) {
      return fail(error, "BTC frame count exceeds remaining file size");
    }

    std::vector<SavedBtcFrame> loaded;
    loaded.reserve(frame_count);
    for (std::uint32_t frame_index = 0; frame_index < frame_count;
         ++frame_index) {
      std::uint32_t frame_id = 0;
      std::uint32_t anchor_keyframe_id = 0;
      std::uint32_t descriptor_count = 0;
      if (!reader.readU32(frame_id) || !reader.readU32(anchor_keyframe_id) ||
          !reader.readU32(descriptor_count)) {
        return fail(error, "truncated BTC file while reading frame " +
                               std::to_string(frame_index) + " header");
      }
      if (frame_id != frame_index) {
        return fail(error,
                    "BTC frame IDs must be contiguous from 0; expected " +
                        std::to_string(frame_index) + ", found " +
                        std::to_string(frame_id));
      }
      if (anchor_keyframe_id >
          static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
        return fail(error, "BTC frame " + std::to_string(frame_index) +
                               " anchor keyframe ID is outside int range");
      }
      if (descriptor_count > kMaximumCount) {
        return fail(error, "BTC frame " + std::to_string(frame_index) +
                               " descriptor count exceeds the 10000000 limit");
      }
      if (descriptor_count > reader.remaining() / kStdMinimumSize) {
        return fail(error, "BTC frame " + std::to_string(frame_index) +
                               " descriptor count exceeds remaining file size");
      }

      SavedBtcFrame frame;
      frame.frame_id = static_cast<int>(frame_id);
      frame.anchor_keyframe_id = static_cast<int>(anchor_keyframe_id);
      frame.descriptors.reserve(descriptor_count);
      for (std::uint32_t descriptor_index = 0;
           descriptor_index < descriptor_count; ++descriptor_index) {
        STD descriptor;
        const std::string context =
            "frame " + std::to_string(frame_index) + " descriptor " +
            std::to_string(descriptor_index);
        if (!readDescriptor(reader, descriptor, context, error)) {
          return false;
        }
        frame.descriptors.push_back(std::move(descriptor));
      }
      loaded.push_back(std::move(frame));
    }
    if (reader.remaining() != 0) {
      return fail(error, "BTC file contains trailing bytes");
    }
    frames = std::move(loaded);
    return true;
  } catch (const std::bad_alloc &) {
    return fail(error, "BTC load failed: allocation limit exceeded");
  } catch (const std::exception &exception) {
    return fail(error, std::string("BTC load failed: ") + exception.what());
  }
}

bool restore_btc_database(const std::vector<SavedBtcFrame> &frames,
                          STDescManager &manager, std::string &error) {
  error.clear();
  try {
    if (!manager.data_base_.empty() || !manager.plane_cloud_vec_.empty()) {
      return fail(error,
                  "cannot restore BTC database into a non-empty manager");
    }

    std::vector<const SavedBtcFrame *> ordered;
    if (!validateRestoreFrames(frames, ordered, error)) {
      return false;
    }

    STDescManager restored(manager.config_setting_);
    for (std::size_t frame_index = 0; frame_index < ordered.size();
         ++frame_index) {
      const SavedBtcFrame &frame = *ordered[frame_index];
      std::vector<STD> descriptors = frame.descriptors;
      for (STD &descriptor : descriptors) {
        descriptor.frame_number_ = static_cast<int>(frame_index);
      }

      pcl::PointCloud<pcl::PointXYZINormal>::Ptr plane_cloud(
          new pcl::PointCloud<pcl::PointXYZINormal>(*frame.plane_cloud));
      plane_cloud->header.seq =
          static_cast<std::uint32_t>(frame.anchor_keyframe_id);
      restored.plane_cloud_vec_.push_back(plane_cloud);
      restored.AddSTDescs(descriptors);
    }

    manager.data_base_.swap(restored.data_base_);
    manager.plane_cloud_vec_.swap(restored.plane_cloud_vec_);
    std::swap(manager.current_frame_id_, restored.current_frame_id_);
    return true;
  } catch (const std::bad_alloc &) {
    return fail(error, "BTC restore failed: allocation limit exceeded");
  } catch (const std::exception &exception) {
    return fail(error, std::string("BTC restore failed: ") + exception.what());
  }
}
