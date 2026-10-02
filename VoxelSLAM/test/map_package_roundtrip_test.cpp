#include "map_package.hpp"

#include <gtest/gtest.h>

#include <boost/filesystem.hpp>
#include <Eigen/Geometry>

#include <cmath>
#include <fstream>
#include <functional>
#include <limits>
#include <locale>
#include <sstream>
#include <string>
#include <vector>

namespace {

namespace fs = boost::filesystem;

class TemporaryMapPath {
 public:
  TemporaryMapPath()
      : root_(fs::temp_directory_path() /
              fs::unique_path("voxel-slam-map-test-%%%%-%%%%-%%%%")),
        map_path_(root_ / "saved-map") {
    fs::create_directories(root_);
  }

  ~TemporaryMapPath() {
    boost::system::error_code ignored;
    fs::remove_all(root_, ignored);
  }

  std::string map_path() const { return map_path_.string(); }
  const fs::path &path() const { return map_path_; }

 private:
  fs::path root_;
  fs::path map_path_;
};

PointType makePoint(float x, float y, float z, float intensity,
                    float normal_x, float normal_y, float normal_z) {
  PointType point;
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = intensity;
  point.normal_x = normal_x;
  point.normal_y = normal_y;
  point.normal_z = normal_z;
  point.curvature = 0.25F;
  return point;
}

SavedKeyframe makeKeyframe(int id, double stamp, const Eigen::Matrix3d &R,
                           const Eigen::Vector3d &t,
                           const std::vector<PointType> &points) {
  SavedKeyframe keyframe;
  keyframe.id = id;
  keyframe.pose.stamp = stamp;
  keyframe.pose.R = R;
  keyframe.pose.t = t;
  keyframe.cloud.reset(new pcl::PointCloud<PointType>());
  keyframe.cloud->points = points;
  keyframe.cloud->width = static_cast<std::uint32_t>(points.size());
  keyframe.cloud->height = 1;
  keyframe.cloud->is_dense = true;
  return keyframe;
}

SavedMap makeTwoKeyframeMap() {
  SavedMap map;
  map.keyframes.push_back(makeKeyframe(
      0, 1000.125,
      Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitZ()).toRotationMatrix(),
      Eigen::Vector3d(1.0, -2.0, 3.5),
      {makePoint(0.1F, 0.2F, 0.3F, 4.0F, 1.0F, 0.0F, 0.0F),
       makePoint(-1.0F, 2.5F, 3.25F, 8.0F, 0.0F, 1.0F, 0.0F)}));
  map.keyframes.push_back(makeKeyframe(
      1, 1001.875,
      Eigen::AngleAxisd(-0.4, Eigen::Vector3d::UnitY()).toRotationMatrix(),
      Eigen::Vector3d(-4.0, 5.25, 0.125),
      {makePoint(9.0F, -8.0F, 7.0F, 6.0F, 0.0F, 0.0F, 1.0F),
       makePoint(0.75F, -0.5F, 0.25F, 2.0F, 0.5F, 0.5F, 0.0F)}));
  return map;
}

MapPackageOptions makeOptions() {
  MapPackageOptions options;
  options.win_size = 12;
  options.keyframe_distance_m = 0.35;
  options.keyframe_angle_deg = 7.5;
  options.keyframe_voxel_size = 0.2;

  ConfigSetting &config = options.btc_config;
  config.useful_corner_num_ = 31;
  config.plane_merge_normal_thre_ = 0.11F;
  config.plane_merge_dis_thre_ = 0.22F;
  config.plane_detection_thre_ = 0.033F;
  config.voxel_size_ = 1.25F;
  config.voxel_init_num_ = 11;
  config.proj_plane_num_ = 4;
  config.proj_image_resolution_ = 0.45F;
  config.proj_image_high_inc_ = 0.55F;
  config.proj_dis_min_ = 0.05F;
  config.proj_dis_max_ = 6.5F;
  config.summary_min_thre_ = 9.0F;
  config.line_filter_enable_ = 1;
  config.touch_filter_enable_ = 1;
  config.descriptor_near_num_ = 12.0F;
  config.descriptor_min_len_ = 1.5F;
  config.descriptor_max_len_ = 12.5F;
  config.non_max_suppression_radius_ = 2.5F;
  config.std_side_resolution_ = 0.25F;
  config.skip_near_num_ = 21;
  config.candidate_num_ = 51;
  config.rough_dis_threshold_ = 0.04F;
  config.similarity_threshold_ = 0.75F;
  config.icp_threshold_ = 0.45F;
  config.normal_threshold_ = 0.15F;
  config.dis_threshold_ = 0.35F;
  return options;
}

std::string readFile(const fs::path &path) {
  std::ifstream input(path.string().c_str(), std::ios::binary);
  std::ostringstream contents;
  contents << input.rdbuf();
  return contents.str();
}

void writeFile(const fs::path &path, const std::string &contents) {
  std::ofstream output(path.string().c_str(),
                       std::ios::binary | std::ios::trunc);
  ASSERT_TRUE(output.good());
  output << contents;
  ASSERT_TRUE(output.good());
}

void replaceOnce(const fs::path &path, const std::string &from,
                 const std::string &to) {
  std::string contents = readFile(path);
  const std::size_t position = contents.find(from);
  ASSERT_NE(position, std::string::npos);
  contents.replace(position, from.size(), to);
  writeFile(path, contents);
}

void expectPointNear(const PointType &actual, const PointType &expected) {
  EXPECT_NEAR(actual.x, expected.x, 1e-6);
  EXPECT_NEAR(actual.y, expected.y, 1e-6);
  EXPECT_NEAR(actual.z, expected.z, 1e-6);
  EXPECT_NEAR(actual.intensity, expected.intensity, 1e-6);
  EXPECT_NEAR(actual.normal_x, expected.normal_x, 1e-6);
  EXPECT_NEAR(actual.normal_y, expected.normal_y, 1e-6);
  EXPECT_NEAR(actual.normal_z, expected.normal_z, 1e-6);
  EXPECT_NEAR(actual.curvature, expected.curvature, 1e-6);
}

class CommaDecimalPoint : public std::numpunct<char> {
 protected:
  char do_decimal_point() const override { return ','; }
};

}  // namespace

TEST(MapPackageOptions, ValueInitializesEveryBtcConfigField) {
  const MapPackageOptions options;
  EXPECT_FLOAT_EQ(options.btc_config.plane_merge_normal_thre_, 0.0F);
  EXPECT_FLOAT_EQ(options.btc_config.plane_merge_dis_thre_, 0.0F);
  EXPECT_EQ(options.btc_config.useful_corner_num_, 30);
  EXPECT_FLOAT_EQ(options.btc_config.voxel_size_, 1.0F);
}

TEST(BtcConfigFingerprint, IncludesEveryFieldAndIgnoresGlobalLocale) {
  const ConfigSetting baseline = makeOptions().btc_config;
  const std::string expected = btc_config_fingerprint(baseline);
  ASSERT_FALSE(expected.empty());

  std::vector<std::function<void(ConfigSetting &)>> mutations = {
      [](ConfigSetting &c) { ++c.useful_corner_num_; },
      [](ConfigSetting &c) { c.plane_merge_normal_thre_ += 1.0F; },
      [](ConfigSetting &c) { c.plane_merge_dis_thre_ += 1.0F; },
      [](ConfigSetting &c) { c.plane_detection_thre_ += 1.0F; },
      [](ConfigSetting &c) { c.voxel_size_ += 1.0F; },
      [](ConfigSetting &c) { ++c.voxel_init_num_; },
      [](ConfigSetting &c) { ++c.proj_plane_num_; },
      [](ConfigSetting &c) { c.proj_image_resolution_ += 1.0F; },
      [](ConfigSetting &c) { c.proj_image_high_inc_ += 1.0F; },
      [](ConfigSetting &c) { c.proj_dis_min_ += 1.0F; },
      [](ConfigSetting &c) { c.proj_dis_max_ += 1.0F; },
      [](ConfigSetting &c) { c.summary_min_thre_ += 1.0F; },
      [](ConfigSetting &c) { ++c.line_filter_enable_; },
      [](ConfigSetting &c) { ++c.touch_filter_enable_; },
      [](ConfigSetting &c) { c.descriptor_near_num_ += 1.0F; },
      [](ConfigSetting &c) { c.descriptor_min_len_ += 1.0F; },
      [](ConfigSetting &c) { c.descriptor_max_len_ += 1.0F; },
      [](ConfigSetting &c) { c.non_max_suppression_radius_ += 1.0F; },
      [](ConfigSetting &c) { c.std_side_resolution_ += 1.0F; },
      [](ConfigSetting &c) { ++c.skip_near_num_; },
      [](ConfigSetting &c) { ++c.candidate_num_; },
      [](ConfigSetting &c) { c.rough_dis_threshold_ += 1.0F; },
      [](ConfigSetting &c) { c.similarity_threshold_ += 1.0F; },
      [](ConfigSetting &c) { c.icp_threshold_ += 1.0F; },
      [](ConfigSetting &c) { c.normal_threshold_ += 1.0F; },
      [](ConfigSetting &c) { c.dis_threshold_ += 1.0F; },
  };

  for (std::size_t i = 0; i < mutations.size(); ++i) {
    ConfigSetting changed = baseline;
    mutations[i](changed);
    EXPECT_NE(btc_config_fingerprint(changed), expected) << "field index " << i;
  }

  const std::locale original = std::locale();
  std::locale::global(std::locale(original, new CommaDecimalPoint));
  const std::string localized = btc_config_fingerprint(baseline);
  std::locale::global(original);
  EXPECT_EQ(localized, expected);
}

TEST(MapPackageIO, RoundTripsLocalFrameKeyframesAndExactPaths) {
  TemporaryMapPath temp;
  const SavedMap input = makeTwoKeyframeMap();
  const MapPackageOptions options = makeOptions();
  std::string error;

  ASSERT_TRUE(MapPackageIO::save(temp.map_path(), input, options, error)) << error;
  EXPECT_TRUE(fs::is_regular_file(temp.path() / "manifest.yaml"));
  EXPECT_TRUE(fs::is_regular_file(temp.path() / "keyframes.csv"));
  EXPECT_TRUE(fs::is_regular_file(temp.path() / "keyframes" / "000000.pcd"));
  EXPECT_TRUE(fs::is_regular_file(temp.path() / "keyframes" / "000001.pcd"));

  const std::string manifest = readFile(temp.path() / "manifest.yaml");
  EXPECT_NE(manifest.find("format: voxel_slam_map"), std::string::npos);
  EXPECT_NE(manifest.find("version: 1"), std::string::npos);
  EXPECT_NE(manifest.find("coordinate_frame: map"), std::string::npos);
  EXPECT_NE(manifest.find("keyframe_cloud_frame: imu"), std::string::npos);
  EXPECT_NE(manifest.find("descriptor_policy: one_per_fused_keyframe"),
            std::string::npos);
  EXPECT_NE(manifest.find("keyframe_count: 2"), std::string::npos);
  EXPECT_NE(manifest.find("btc_frame_count: 0"), std::string::npos);

  const std::string csv = readFile(temp.path() / "keyframes.csv");
  EXPECT_NE(csv.find("kf_id,timestamp,tx,ty,tz,qx,qy,qz,qw,cloud_file"),
            std::string::npos);
  EXPECT_NE(csv.find(",keyframes/000000.pcd"), std::string::npos);
  EXPECT_NE(csv.find(",keyframes/000001.pcd"), std::string::npos);

  SavedMap output;
  ASSERT_TRUE(MapPackageIO::load(temp.map_path(), output, options, error)) << error;
  ASSERT_EQ(output.keyframes.size(), input.keyframes.size());
  EXPECT_TRUE(output.btc_frames.empty());
  for (std::size_t i = 0; i < input.keyframes.size(); ++i) {
    EXPECT_EQ(output.keyframes[i].id, static_cast<int>(i));
    EXPECT_DOUBLE_EQ(output.keyframes[i].pose.stamp,
                     input.keyframes[i].pose.stamp);
    EXPECT_TRUE(output.keyframes[i].pose.R.isApprox(
        input.keyframes[i].pose.R, 1e-12));
    EXPECT_TRUE(output.keyframes[i].pose.t.isApprox(
        input.keyframes[i].pose.t, 1e-12));
    ASSERT_TRUE(output.keyframes[i].cloud);
    ASSERT_EQ(output.keyframes[i].cloud->size(),
              input.keyframes[i].cloud->size());
    for (std::size_t point = 0; point < input.keyframes[i].cloud->size();
         ++point) {
      expectPointNear(output.keyframes[i].cloud->points[point],
                      input.keyframes[i].cloud->points[point]);
    }
  }
}

TEST(MapPackageIO, RejectsWrongManifestVersion) {
  TemporaryMapPath temp;
  const MapPackageOptions options = makeOptions();
  std::string error;
  ASSERT_TRUE(MapPackageIO::save(temp.map_path(), makeTwoKeyframeMap(), options,
                                 error))
      << error;
  replaceOnce(temp.path() / "manifest.yaml", "version: 1", "version: 2");

  SavedMap output;
  EXPECT_FALSE(MapPackageIO::load(temp.map_path(), output, options, error));
  EXPECT_NE(error.find("version"), std::string::npos) << error;
}

TEST(MapPackageIO, RejectsMissingKeyframeCloud) {
  TemporaryMapPath temp;
  const MapPackageOptions options = makeOptions();
  std::string error;
  ASSERT_TRUE(MapPackageIO::save(temp.map_path(), makeTwoKeyframeMap(), options,
                                 error))
      << error;
  ASSERT_TRUE(fs::remove(temp.path() / "keyframes" / "000001.pcd"));

  SavedMap output;
  EXPECT_FALSE(MapPackageIO::load(temp.map_path(), output, options, error));
  EXPECT_NE(error.find("keyframes/000001.pcd"), std::string::npos) << error;
}

TEST(MapPackageIO, RejectsNonContiguousKeyframeIds) {
  TemporaryMapPath temp;
  const MapPackageOptions options = makeOptions();
  std::string error;
  ASSERT_TRUE(MapPackageIO::save(temp.map_path(), makeTwoKeyframeMap(), options,
                                 error))
      << error;
  replaceOnce(temp.path() / "keyframes.csv", "\n1,", "\n2,");

  SavedMap output;
  EXPECT_FALSE(MapPackageIO::load(temp.map_path(), output, options, error));
  EXPECT_NE(error.find("contiguous"), std::string::npos) << error;
}

TEST(MapPackageIO, RejectsMismatchedBtcConfigFingerprint) {
  TemporaryMapPath temp;
  const MapPackageOptions saved_options = makeOptions();
  std::string error;
  ASSERT_TRUE(MapPackageIO::save(temp.map_path(), makeTwoKeyframeMap(),
                                 saved_options, error))
      << error;

  MapPackageOptions load_options = saved_options;
  load_options.btc_config.candidate_num_ += 1;
  SavedMap output;
  EXPECT_FALSE(
      MapPackageIO::load(temp.map_path(), output, load_options, error));
  EXPECT_NE(error.find("btc_config_fingerprint"), std::string::npos) << error;
}

TEST(MapPackageIO, RejectsExistingDestinationWithoutOverwriting) {
  TemporaryMapPath temp;
  const MapPackageOptions options = makeOptions();
  std::string error;
  ASSERT_TRUE(MapPackageIO::save(temp.map_path(), makeTwoKeyframeMap(), options,
                                 error))
      << error;
  const std::string original_manifest = readFile(temp.path() / "manifest.yaml");

  EXPECT_FALSE(MapPackageIO::save(temp.map_path(), makeTwoKeyframeMap(), options,
                                  error));
  EXPECT_NE(error.find("already exists"), std::string::npos) << error;
  EXPECT_EQ(readFile(temp.path() / "manifest.yaml"), original_manifest);
}

TEST(MapPackageIO, RejectsBtcFramesUntilTaskTwo) {
  TemporaryMapPath temp;
  SavedMap map = makeTwoKeyframeMap();
  map.btc_frames.emplace_back();
  std::string error;

  EXPECT_FALSE(MapPackageIO::save(temp.map_path(), map, makeOptions(), error));
  EXPECT_NE(error.find("BTC"), std::string::npos) << error;
  EXPECT_FALSE(fs::exists(temp.path()));
}

TEST(MapPackageIO, RejectsNonFiniteOptionsAndConfig) {
  SavedMap map = makeTwoKeyframeMap();
  std::string error;

  TemporaryMapPath option_temp;
  MapPackageOptions bad_option = makeOptions();
  bad_option.keyframe_voxel_size =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(
      MapPackageIO::save(option_temp.map_path(), map, bad_option, error));
  EXPECT_NE(error.find("keyframe_voxel_size"), std::string::npos) << error;
  EXPECT_FALSE(fs::exists(option_temp.path()));

  TemporaryMapPath config_temp;
  MapPackageOptions bad_config = makeOptions();
  bad_config.btc_config.icp_threshold_ =
      std::numeric_limits<float>::infinity();
  EXPECT_FALSE(
      MapPackageIO::save(config_temp.map_path(), map, bad_config, error));
  EXPECT_NE(error.find("icp_threshold"), std::string::npos) << error;
  EXPECT_FALSE(fs::exists(config_temp.path()));
  EXPECT_TRUE(btc_config_fingerprint(bad_config.btc_config).empty());

  TemporaryMapPath load_temp;
  const MapPackageOptions valid_options = makeOptions();
  ASSERT_TRUE(MapPackageIO::save(load_temp.map_path(), map, valid_options,
                                 error))
      << error;
  SavedMap output;
  EXPECT_FALSE(
      MapPackageIO::load(load_temp.map_path(), output, bad_config, error));
  EXPECT_NE(error.find("icp_threshold"), std::string::npos) << error;
}
