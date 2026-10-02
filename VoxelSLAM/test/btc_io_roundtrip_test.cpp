#include "btc_io.hpp"

#include <gtest/gtest.h>

#include <boost/filesystem.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <initializer_list>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {

namespace fs = boost::filesystem;

class TemporaryBtcPath {
 public:
  TemporaryBtcPath()
      : root_(fs::temp_directory_path() /
              fs::unique_path("voxel-slam-btc-test-%%%%-%%%%-%%%%")),
        file_(root_ / "btc_frames.bin") {
    fs::create_directories(root_);
  }

  ~TemporaryBtcPath() {
    boost::system::error_code ignored;
    fs::remove_all(root_, ignored);
  }

  std::string file() const { return file_.string(); }

 private:
  fs::path root_;
  fs::path file_;
};

BinaryDescriptor makeBinary(unsigned char summary,
                            const Eigen::Vector3d &location,
                            const std::vector<bool> &bits) {
  BinaryDescriptor descriptor;
  descriptor.summary_ = summary;
  descriptor.location_ = location;
  descriptor.occupy_array_ = bits;
  return descriptor;
}

STD makeDescriptor(int seed, int frame_number,
                   const std::vector<bool> &a_bits,
                   const std::vector<bool> &b_bits,
                   const std::vector<bool> &c_bits) {
  STD descriptor;
  descriptor.triangle_ = Eigen::Vector3d(seed + 0.125, seed + 1.25,
                                         seed + 2.5);
  descriptor.angle_ = Eigen::Vector3d(seed + 3.75, seed + 4.875,
                                      seed + 5.0625);
  descriptor.center_ = Eigen::Vector3d(seed - 0.5, seed + 0.25,
                                       seed + 0.75);
  descriptor.frame_number_ = frame_number;
  descriptor.binary_A_ = makeBinary(
      3, Eigen::Vector3d(seed + 0.1, seed + 0.2, seed + 0.3), a_bits);
  descriptor.binary_B_ = makeBinary(
      7, Eigen::Vector3d(seed + 1.1, seed + 1.2, seed + 1.3), b_bits);
  descriptor.binary_C_ = makeBinary(
      11, Eigen::Vector3d(seed + 2.1, seed + 2.2, seed + 2.3), c_bits);
  return descriptor;
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr makePlaneCloud(int seed) {
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZINormal>());
  for (int i = 0; i < 3; ++i) {
    pcl::PointXYZINormal point;
    point.x = static_cast<float>(seed + i);
    point.y = static_cast<float>(seed - i * 0.5);
    point.z = static_cast<float>(seed + i * 0.25);
    point.normal_x = 0.0F;
    point.normal_y = 0.0F;
    point.normal_z = 1.0F;
    point.intensity = static_cast<float>(seed * 10 + i);
    point.curvature = static_cast<float>(i) / 10.0F;
    cloud->push_back(point);
  }
  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

std::vector<SavedBtcFrame> makeFrames() {
  std::vector<SavedBtcFrame> frames(2);
  frames[0].frame_id = 0;
  frames[0].anchor_keyframe_id = 1;
  frames[0].descriptors.push_back(makeDescriptor(
      1, 91, {true, false, true, true, false},
      {false, true, false, true, true, false, false, true, true}, {}));
  frames[0].descriptors.push_back(makeDescriptor(
      20, -7, {false}, {true, false, true},
      {true, true, false, false, true, false, true, false, true, true, false,
       true, false, false, true, true, true}));
  frames[0].plane_cloud = makePlaneCloud(1);

  frames[1].frame_id = 1;
  frames[1].anchor_keyframe_id = 0;
  frames[1].descriptors.push_back(
      makeDescriptor(40, 1234, {}, {true, false}, {false, true, false, true}));
  frames[1].plane_cloud = makePlaneCloud(2);
  return frames;
}

void expectBinaryEqual(const BinaryDescriptor &actual,
                       const BinaryDescriptor &expected) {
  EXPECT_EQ(actual.summary_, expected.summary_);
  EXPECT_TRUE(actual.location_.isApprox(expected.location_, 0.0));
  EXPECT_EQ(actual.occupy_array_, expected.occupy_array_);
}

void expectDescriptorEqual(const STD &actual, const STD &expected) {
  EXPECT_TRUE(actual.triangle_.isApprox(expected.triangle_, 0.0));
  EXPECT_TRUE(actual.angle_.isApprox(expected.angle_, 0.0));
  EXPECT_TRUE(actual.center_.isApprox(expected.center_, 0.0));
  EXPECT_EQ(actual.frame_number_, expected.frame_number_);
  expectBinaryEqual(actual.binary_A_, expected.binary_A_);
  expectBinaryEqual(actual.binary_B_, expected.binary_B_);
  expectBinaryEqual(actual.binary_C_, expected.binary_C_);
}

std::string readFile(const std::string &path) {
  std::ifstream input(path.c_str(), std::ios::binary);
  std::ostringstream contents;
  contents << input.rdbuf();
  return contents.str();
}

void writeFile(const std::string &path, const std::string &contents) {
  std::ofstream output(path.c_str(), std::ios::binary | std::ios::trunc);
  ASSERT_TRUE(output.good());
  output.write(contents.data(), static_cast<std::streamsize>(contents.size()));
  ASSERT_TRUE(output.good());
}

void putU32(std::string &bytes, std::size_t offset, std::uint32_t value) {
  ASSERT_LE(offset + 4, bytes.size());
  for (std::size_t i = 0; i < 4; ++i) {
    bytes[offset + i] = static_cast<char>((value >> (i * 8)) & 0xffU);
  }
}

void putDouble(std::string &bytes, std::size_t offset, double value) {
  static_assert(sizeof(double) == 8, "test requires 8-byte double");
  std::uint64_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  ASSERT_LE(offset + 8, bytes.size());
  for (std::size_t i = 0; i < 8; ++i) {
    bytes[offset + i] = static_cast<char>((bits >> (i * 8)) & UINT64_C(0xff));
  }
}

void expectBytesAt(const std::string &bytes, std::size_t offset,
                   std::initializer_list<std::uint8_t> expected) {
  ASSERT_LE(offset + expected.size(), bytes.size());
  std::size_t index = 0;
  for (std::uint8_t value : expected) {
    EXPECT_EQ(static_cast<unsigned char>(bytes[offset + index]), value)
        << "byte offset " << offset + index;
    ++index;
  }
}

void expectLoadFailurePreservesCaller(const std::string &path,
                                      const std::string &error_fragment) {
  std::vector<SavedBtcFrame> output(1);
  output[0].frame_id = 77;
  std::string error;
  EXPECT_FALSE(load_btc_frames(path, output, error));
  ASSERT_EQ(output.size(), 1U);
  EXPECT_EQ(output[0].frame_id, 77);
  EXPECT_NE(error.find(error_fragment), std::string::npos) << error;
}

STD makeSearchDescriptor(int index, int frame_number) {
  STD descriptor;
  descriptor.triangle_ = Eigen::Vector3d(3.0 + index * 3.0,
                                         4.0 + index * 3.0,
                                         5.0 + index * 3.0);
  descriptor.angle_ = Eigen::Vector3d(0.1, 0.2, 0.3);
  descriptor.center_ = Eigen::Vector3d(index * 5.0, 0.0, 0.0);
  descriptor.frame_number_ = frame_number;
  const std::vector<bool> bits(8, true);
  descriptor.binary_A_ = makeBinary(
      8, descriptor.center_ + Eigen::Vector3d(1.0, 0.0, 0.0), bits);
  descriptor.binary_B_ = makeBinary(
      8, descriptor.center_ + Eigen::Vector3d(0.0, 1.0, 0.0), bits);
  descriptor.binary_C_ = makeBinary(
      8, descriptor.center_ + Eigen::Vector3d(0.0, 0.0, 1.0), bits);
  return descriptor;
}

ConfigSetting makeSearchConfig() {
  ConfigSetting config;
  config.skip_near_num_ = 0;
  config.candidate_num_ = 1;
  config.rough_dis_threshold_ = 0.1F;
  config.similarity_threshold_ = 0.5F;
  config.icp_threshold_ = 0.5F;
  config.normal_threshold_ = 0.1F;
  config.dis_threshold_ = 0.1F;
  return config;
}

std::pair<int, double> search(STDescManager &manager,
                              const std::vector<STD> &descriptors,
                              const pcl::PointCloud<
                                  pcl::PointXYZINormal>::Ptr &plane) {
  std::vector<STD> query = descriptors;
  std::pair<int, double> result(-2, -1.0);
  std::pair<Eigen::Vector3d, Eigen::Matrix3d> transform;
  std::vector<std::pair<STD, STD>> matches;
  manager.SearchLoop(query, result, transform, matches, plane);
  return result;
}

const STD *findStoredDescriptor(const STDescManager &manager,
                                const Eigen::Vector3d &triangle) {
  for (const auto &entry : manager.data_base_) {
    for (const STD &descriptor : entry.second) {
      if (descriptor.triangle_.isApprox(triangle, 0.0)) {
        return &descriptor;
      }
    }
  }
  return nullptr;
}

}  // namespace

TEST(BtcIo, RoundTripsStableFieldsAcrossTwoFrames) {
  TemporaryBtcPath temp;
  const std::vector<SavedBtcFrame> input = makeFrames();
  std::string error;
  ASSERT_TRUE(save_btc_frames(temp.file(), input, error)) << error;

  const std::string bytes = readFile(temp.file());
  ASSERT_GE(bytes.size(), 16U);
  EXPECT_EQ(bytes.substr(0, 8), "VXLBTC01");
  EXPECT_EQ(static_cast<unsigned char>(bytes[8]), 1U);
  EXPECT_EQ(static_cast<unsigned char>(bytes[12]), 2U);

  std::vector<SavedBtcFrame> output;
  ASSERT_TRUE(load_btc_frames(temp.file(), output, error)) << error;
  ASSERT_EQ(output.size(), input.size());
  for (std::size_t frame = 0; frame < input.size(); ++frame) {
    EXPECT_EQ(output[frame].frame_id, input[frame].frame_id);
    EXPECT_EQ(output[frame].anchor_keyframe_id,
              input[frame].anchor_keyframe_id);
    EXPECT_FALSE(output[frame].plane_cloud);
    ASSERT_EQ(output[frame].descriptors.size(),
              input[frame].descriptors.size());
    for (std::size_t descriptor = 0;
         descriptor < input[frame].descriptors.size(); ++descriptor) {
      expectDescriptorEqual(output[frame].descriptors[descriptor],
                            input[frame].descriptors[descriptor]);
    }
  }
}

TEST(BtcIo, WritesGoldenLittleEndianIntegersAndLsbFirstPackedBits) {
  TemporaryBtcPath temp;
  std::vector<SavedBtcFrame> frames(1);
  frames[0].frame_id = 0;
  frames[0].anchor_keyframe_id = 0x12345678;
  frames[0].descriptors.push_back(makeDescriptor(
      5, -2, {true, false, true, false, false, false, false, true, true},
      {false}, {}));

  std::string error;
  ASSERT_TRUE(save_btc_frames(temp.file(), frames, error)) << error;
  const std::string bytes = readFile(temp.file());

  ASSERT_EQ(bytes.size(), 194U);
  EXPECT_EQ(bytes.substr(0, 8), "VXLBTC01");
  // Every uint32/int32 field is checked as four literal golden bytes. The
  // packed bytes are also literal, independent of the codec helpers.
  expectBytesAt(bytes, 8, {0x01, 0x00, 0x00, 0x00});
  expectBytesAt(bytes, 12, {0x01, 0x00, 0x00, 0x00});
  expectBytesAt(bytes, 16, {0x00, 0x00, 0x00, 0x00});
  expectBytesAt(bytes, 20, {0x78, 0x56, 0x34, 0x12});
  expectBytesAt(bytes, 24, {0x01, 0x00, 0x00, 0x00});
  expectBytesAt(bytes, 100, {0xfe, 0xff, 0xff, 0xff});
  expectBytesAt(bytes, 129, {0x09, 0x00, 0x00, 0x00});
  expectBytesAt(bytes, 133, {0x85, 0x01});
  expectBytesAt(bytes, 160, {0x01, 0x00, 0x00, 0x00});
  expectBytesAt(bytes, 164, {0x00});
  expectBytesAt(bytes, 190, {0x00, 0x00, 0x00, 0x00});
}

TEST(BtcIo, RejectsEveryTruncatedPrefixAndPreservesCaller) {
  TemporaryBtcPath temp;
  std::string error;
  ASSERT_TRUE(save_btc_frames(temp.file(), makeFrames(), error)) << error;
  const std::string complete = readFile(temp.file());
  ASSERT_GT(complete.size(), 16U);

  for (std::size_t length = 0; length < complete.size(); ++length) {
    writeFile(temp.file(), complete.substr(0, length));
    std::vector<SavedBtcFrame> output(1);
    output[0].frame_id = 77;
    EXPECT_FALSE(load_btc_frames(temp.file(), output, error))
        << "accepted truncated length " << length;
    ASSERT_EQ(output.size(), 1U);
    EXPECT_EQ(output[0].frame_id, 77);
    EXPECT_FALSE(error.empty());
  }
}

TEST(BtcIo, RejectsWrongMagicVersionIdsAndTrailingBytes) {
  TemporaryBtcPath temp;
  std::string error;
  ASSERT_TRUE(save_btc_frames(temp.file(), makeFrames(), error)) << error;
  const std::string valid = readFile(temp.file());

  std::string bytes = valid;
  bytes[0] = 'X';
  writeFile(temp.file(), bytes);
  expectLoadFailurePreservesCaller(temp.file(), "magic");

  bytes = valid;
  putU32(bytes, 8, 2);
  writeFile(temp.file(), bytes);
  expectLoadFailurePreservesCaller(temp.file(), "version");

  bytes = valid;
  putU32(bytes, 16, 1);
  writeFile(temp.file(), bytes);
  expectLoadFailurePreservesCaller(temp.file(), "contiguous");

  bytes = valid;
  bytes.push_back('\0');
  writeFile(temp.file(), bytes);
  expectLoadFailurePreservesCaller(temp.file(), "trailing");
}

TEST(BtcIo, RejectsExcessiveCountsBitLengthsAndNonFiniteValues) {
  TemporaryBtcPath temp;
  std::string error;
  std::vector<SavedBtcFrame> one_frame = makeFrames();
  one_frame.resize(1);
  one_frame[0].descriptors.resize(1);
  ASSERT_TRUE(save_btc_frames(temp.file(), one_frame, error)) << error;
  const std::string valid = readFile(temp.file());

  std::string bytes = valid;
  putU32(bytes, 12, 10000001U);
  writeFile(temp.file(), bytes);
  expectLoadFailurePreservesCaller(temp.file(), "frame count");

  bytes = valid;
  putU32(bytes, 24, UINT32_MAX);
  writeFile(temp.file(), bytes);
  expectLoadFailurePreservesCaller(temp.file(), "descriptor count");

  // Header (16) + frame header (12) + nine doubles (72) + int32 (4) +
  // binary-A summary (1) + location (24) places A's bit count at byte 129.
  bytes = valid;
  putU32(bytes, 129, 10000001U);
  writeFile(temp.file(), bytes);
  expectLoadFailurePreservesCaller(temp.file(), "bit count");

  bytes = valid;
  putU32(bytes, 129, UINT32_MAX);
  writeFile(temp.file(), bytes);
  expectLoadFailurePreservesCaller(temp.file(), "bit count");

  bytes = valid;
  putDouble(bytes, 28, std::numeric_limits<double>::quiet_NaN());
  writeFile(temp.file(), bytes);
  expectLoadFailurePreservesCaller(temp.file(), "non-finite");

  bytes = valid;
  putDouble(bytes, 105, std::numeric_limits<double>::infinity());
  writeFile(temp.file(), bytes);
  expectLoadFailurePreservesCaller(temp.file(), "non-finite");
}

TEST(BtcIo, RejectsInvalidSaveInputBeforeCreatingAFile) {
  std::string error;

  TemporaryBtcPath ids_temp;
  std::vector<SavedBtcFrame> bad_ids = makeFrames();
  bad_ids[1].frame_id = 3;
  EXPECT_FALSE(save_btc_frames(ids_temp.file(), bad_ids, error));
  EXPECT_NE(error.find("contiguous"), std::string::npos) << error;
  EXPECT_FALSE(fs::exists(ids_temp.file()));

  TemporaryBtcPath finite_temp;
  std::vector<SavedBtcFrame> non_finite = makeFrames();
  non_finite[0].descriptors[0].angle_.x() =
      std::numeric_limits<double>::infinity();
  EXPECT_FALSE(save_btc_frames(finite_temp.file(), non_finite, error));
  EXPECT_NE(error.find("non-finite"), std::string::npos) << error;
  EXPECT_FALSE(fs::exists(finite_temp.file()));
}

TEST(BtcRestore, RestoresOrderedFramesPlanesAnchorsAndRewrittenIds) {
  std::vector<SavedBtcFrame> frames = makeFrames();
  frames[0].anchor_keyframe_id = 101;
  frames[1].anchor_keyframe_id = 202;
  SavedBtcFrame frame_two;
  frame_two.frame_id = 2;
  frame_two.anchor_keyframe_id = 303;
  frame_two.descriptors.push_back(makeDescriptor(
      70, -55, {true, false, true}, {false, true}, {true}));
  frame_two.plane_cloud = makePlaneCloud(3);
  frames.push_back(frame_two);

  const pcl::PointCloud<pcl::PointXYZINormal>::Ptr frame_zero_plane =
      frames[0].plane_cloud;
  const pcl::PointCloud<pcl::PointXYZINormal>::Ptr frame_one_plane =
      frames[1].plane_cloud;
  const pcl::PointCloud<pcl::PointXYZINormal>::Ptr frame_two_plane =
      frames[2].plane_cloud;
  frame_zero_plane->header.seq = 701U;
  frame_one_plane->header.seq = 702U;
  frame_two_plane->header.seq = 703U;
  std::reverse(frames.begin(), frames.end());

  ConfigSetting config = makeSearchConfig();
  STDescManager manager(config);
  std::string error;
  ASSERT_TRUE(restore_btc_database(frames, manager, error)) << error;

  EXPECT_EQ(manager.current_frame_id_, 3U);
  ASSERT_EQ(manager.plane_cloud_vec_.size(), 3U);
  EXPECT_NE(manager.plane_cloud_vec_[0], frame_zero_plane);
  EXPECT_NE(manager.plane_cloud_vec_[1], frame_one_plane);
  EXPECT_NE(manager.plane_cloud_vec_[2], frame_two_plane);
  EXPECT_EQ(manager.plane_cloud_vec_[0]->size(), frame_zero_plane->size());
  EXPECT_EQ(manager.plane_cloud_vec_[1]->size(), frame_one_plane->size());
  EXPECT_EQ(manager.plane_cloud_vec_[2]->size(), frame_two_plane->size());
  ASSERT_FALSE(manager.plane_cloud_vec_[0]->empty());
  ASSERT_FALSE(manager.plane_cloud_vec_[1]->empty());
  ASSERT_FALSE(manager.plane_cloud_vec_[2]->empty());
  EXPECT_FLOAT_EQ(manager.plane_cloud_vec_[0]->front().x,
                  frame_zero_plane->front().x);
  EXPECT_FLOAT_EQ(manager.plane_cloud_vec_[1]->front().x,
                  frame_one_plane->front().x);
  EXPECT_FLOAT_EQ(manager.plane_cloud_vec_[2]->front().x,
                  frame_two_plane->front().x);
  EXPECT_EQ(manager.plane_cloud_vec_[0]->header.seq, 101U);
  EXPECT_EQ(manager.plane_cloud_vec_[1]->header.seq, 202U);
  EXPECT_EQ(manager.plane_cloud_vec_[2]->header.seq, 303U);

  ASSERT_EQ(frames.size(), 3U);
  EXPECT_EQ(frames[0].frame_id, 2);
  EXPECT_EQ(frames[1].frame_id, 1);
  EXPECT_EQ(frames[2].frame_id, 0);
  EXPECT_EQ(frames[0].plane_cloud, frame_two_plane);
  EXPECT_EQ(frames[1].plane_cloud, frame_one_plane);
  EXPECT_EQ(frames[2].plane_cloud, frame_zero_plane);
  EXPECT_EQ(frame_zero_plane->header.seq, 701U);
  EXPECT_EQ(frame_one_plane->header.seq, 702U);
  EXPECT_EQ(frame_two_plane->header.seq, 703U);
  ASSERT_EQ(frames[0].descriptors.size(), 1U);
  ASSERT_EQ(frames[1].descriptors.size(), 1U);
  ASSERT_EQ(frames[2].descriptors.size(), 2U);
  EXPECT_EQ(frames[0].descriptors[0].frame_number_, -55);
  EXPECT_EQ(frames[1].descriptors[0].frame_number_, 1234);
  EXPECT_EQ(frames[2].descriptors[0].frame_number_, 91);
  EXPECT_EQ(frames[2].descriptors[1].frame_number_, -7);

  std::size_t descriptor_count = 0;
  for (const auto &entry : manager.data_base_) {
    descriptor_count += entry.second.size();
  }
  for (const SavedBtcFrame &frame : frames) {
    for (const STD &input_descriptor : frame.descriptors) {
      const STD *stored =
          findStoredDescriptor(manager, input_descriptor.triangle_);
      ASSERT_NE(stored, nullptr);
      EXPECT_EQ(stored->frame_number_, frame.frame_id);
    }
  }
  EXPECT_EQ(descriptor_count, 4U);
}

TEST(BtcRestore, RejectsNonContinuousFramesAndNonEmptyDestination) {
  ConfigSetting config = makeSearchConfig();
  std::string error;

  std::vector<SavedBtcFrame> bad_frames = makeFrames();
  bad_frames[0].plane_cloud->header.seq = 77U;
  bad_frames[1].plane_cloud->header.seq = 88U;
  bad_frames[1].frame_id = 4;
  STDescManager empty_manager(config);
  empty_manager.current_frame_id_ = 17U;
  EXPECT_FALSE(restore_btc_database(bad_frames, empty_manager, error));
  EXPECT_NE(error.find("contiguous"), std::string::npos) << error;
  EXPECT_EQ(empty_manager.current_frame_id_, 17U);
  EXPECT_TRUE(empty_manager.data_base_.empty());
  EXPECT_TRUE(empty_manager.plane_cloud_vec_.empty());
  EXPECT_EQ(bad_frames[0].plane_cloud->header.seq, 77U);
  EXPECT_EQ(bad_frames[1].plane_cloud->header.seq, 88U);
  EXPECT_EQ(bad_frames[0].descriptors[0].frame_number_, 91);
  EXPECT_EQ(bad_frames[1].descriptors[0].frame_number_, 1234);

  STDescManager non_empty_manager(config);
  const pcl::PointCloud<pcl::PointXYZINormal>::Ptr existing_plane =
      makePlaneCloud(9);
  existing_plane->header.seq = 909U;
  non_empty_manager.plane_cloud_vec_.push_back(existing_plane);
  const std::vector<STD> existing_descriptors(
      1, makeDescriptor(90, 12, {true}, {false}, {}));
  non_empty_manager.AddSTDescs(existing_descriptors);
  const unsigned int existing_frame_id = non_empty_manager.current_frame_id_;
  const std::size_t existing_bucket_count = non_empty_manager.data_base_.size();
  EXPECT_FALSE(restore_btc_database(makeFrames(), non_empty_manager, error));
  EXPECT_NE(error.find("non-empty"), std::string::npos) << error;
  EXPECT_EQ(non_empty_manager.plane_cloud_vec_.size(), 1U);
  EXPECT_EQ(non_empty_manager.plane_cloud_vec_[0], existing_plane);
  EXPECT_EQ(non_empty_manager.plane_cloud_vec_[0]->header.seq, 909U);
  EXPECT_EQ(non_empty_manager.current_frame_id_, existing_frame_id);
  EXPECT_EQ(non_empty_manager.data_base_.size(), existing_bucket_count);
  const STD *stored = findStoredDescriptor(
      non_empty_manager, existing_descriptors[0].triangle_);
  ASSERT_NE(stored, nullptr);
  EXPECT_EQ(stored->frame_number_, 12);
}

TEST(BtcRestore, PreservesSearchCandidateAndScore) {
  SavedBtcFrame frame;
  frame.frame_id = 0;
  frame.anchor_keyframe_id = 0;
  frame.plane_cloud = makePlaneCloud(0);
  for (int i = 0; i < 5; ++i) {
    frame.descriptors.push_back(makeSearchDescriptor(i, 99));
  }
  const std::vector<SavedBtcFrame> frames(1, frame);

  ConfigSetting before_config = makeSearchConfig();
  STDescManager before(before_config);
  std::vector<STD> normalized = frame.descriptors;
  for (STD &descriptor : normalized) {
    descriptor.frame_number_ = 0;
  }
  before.plane_cloud_vec_.push_back(frame.plane_cloud);
  before.plane_cloud_vec_.back()->header.seq = frame.anchor_keyframe_id;
  before.AddSTDescs(normalized);

  ConfigSetting after_config = makeSearchConfig();
  STDescManager after(after_config);
  std::string error;
  ASSERT_TRUE(restore_btc_database(frames, after, error)) << error;

  std::vector<STD> query;
  for (int i = 0; i < 5; ++i) {
    query.push_back(makeSearchDescriptor(i, 10));
  }
  const std::pair<int, double> before_result =
      search(before, query, makePlaneCloud(0));
  const std::pair<int, double> after_result =
      search(after, query, makePlaneCloud(0));

  EXPECT_EQ(before_result.first, 0);
  EXPECT_EQ(after_result.first, before_result.first);
  EXPECT_DOUBLE_EQ(after_result.second, before_result.second);
}
