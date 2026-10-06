#include "map_package.hpp"

#include "btc_io.hpp"
#include "spatial_map_builder.hpp"
#include "spatial_map_io.hpp"

#include <boost/filesystem.hpp>
#include <boost/system/error_code.hpp>
#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

namespace fs = boost::filesystem;

const char kManifestFile[] = "manifest.yaml";
const char kKeyframesFile[] = "keyframes.csv";
const char kKeyframesDirectory[] = "keyframes";
const char kBtcFramesFile[] = "btc_frames.bin";
const char kBtcPlanesDirectory[] = "btc_planes";
const char kCsvHeader[] =
    "kf_id,timestamp,tx,ty,tz,qx,qy,qz,qw,cloud_file";

bool fail(std::string &error, const std::string &message) {
  error = message;
  return false;
}

std::string trim(const std::string &value) {
  const std::string whitespace = " \t\r\n";
  const std::size_t begin = value.find_first_not_of(whitespace);
  if (begin == std::string::npos) {
    return std::string();
  }
  const std::size_t end = value.find_last_not_of(whitespace);
  return value.substr(begin, end - begin + 1);
}

bool parseInt(const std::string &text, int &value) {
  std::istringstream input(text);
  input.imbue(std::locale::classic());
  input >> value;
  input >> std::ws;
  return input && input.eof();
}

bool parseDouble(const std::string &text, double &value) {
  std::istringstream input(text);
  input.imbue(std::locale::classic());
  input >> value;
  input >> std::ws;
  return input && input.eof() && std::isfinite(value);
}

bool checkFinite(float value, const char *name, std::string &error) {
  if (!std::isfinite(value)) {
    return fail(error, std::string("non-finite BTC config value: ") + name);
  }
  return true;
}

bool validateConfig(const ConfigSetting &config, std::string &error) {
  return checkFinite(config.plane_merge_normal_thre_,
                     "plane_merge_normal_thre_", error) &&
         checkFinite(config.plane_merge_dis_thre_, "plane_merge_dis_thre_",
                     error) &&
         checkFinite(config.plane_detection_thre_, "plane_detection_thre_",
                     error) &&
         checkFinite(config.voxel_size_, "voxel_size_", error) &&
         checkFinite(config.proj_image_resolution_, "proj_image_resolution_",
                     error) &&
         checkFinite(config.proj_image_high_inc_, "proj_image_high_inc_",
                     error) &&
         checkFinite(config.proj_dis_min_, "proj_dis_min_", error) &&
         checkFinite(config.proj_dis_max_, "proj_dis_max_", error) &&
         checkFinite(config.summary_min_thre_, "summary_min_thre_", error) &&
         checkFinite(config.descriptor_near_num_, "descriptor_near_num_",
                     error) &&
         checkFinite(config.descriptor_min_len_, "descriptor_min_len_",
                     error) &&
         checkFinite(config.descriptor_max_len_, "descriptor_max_len_",
                     error) &&
         checkFinite(config.non_max_suppression_radius_,
                     "non_max_suppression_radius_", error) &&
         checkFinite(config.std_side_resolution_, "std_side_resolution_",
                     error) &&
         checkFinite(config.rough_dis_threshold_, "rough_dis_threshold_",
                     error) &&
         checkFinite(config.similarity_threshold_, "similarity_threshold_",
                     error) &&
         checkFinite(config.icp_threshold_, "icp_threshold_", error) &&
         checkFinite(config.normal_threshold_, "normal_threshold_", error) &&
         checkFinite(config.dis_threshold_, "dis_threshold_", error);
}

bool validateRuntimeOptions(const MapPackageOptions &options,
                            std::string &error) {
  if (!std::isfinite(options.keyframe_distance_m)) {
    return fail(error, "non-finite map package option: keyframe_distance_m");
  }
  if (!std::isfinite(options.keyframe_angle_deg)) {
    return fail(error, "non-finite map package option: keyframe_angle_deg");
  }
  if (!std::isfinite(options.keyframe_voxel_size)) {
    return fail(error, "non-finite map package option: keyframe_voxel_size");
  }
  return validateConfig(options.btc_config, error);
}

bool validateOptions(const MapPackageOptions &options, std::string &error) {
  if (!validateRuntimeOptions(options, error)) {
    return false;
  }
  std::string spatial_error;
  if (!options.spatial_map.validate(spatial_error)) {
    return fail(error, "invalid spatial map configuration: " +
                           spatial_error);
  }
  return true;
}

bool spatialConfigsEqual(const SpatialMapConfig &left,
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

bool buildSpatialMap(const SavedMap &map, const SpatialMapConfig &config,
                     SpatialMapBuildResult &result, std::string &error) {
  SpatialMapPointVector points;
  std::size_t point_count = 0U;
  for (const SavedKeyframe &keyframe : map.keyframes) {
    if (!keyframe.cloud ||
        point_count > std::numeric_limits<std::size_t>::max() -
                          keyframe.cloud->size()) {
      return fail(error, "invalid keyframe cloud while building spatial map");
    }
    point_count += keyframe.cloud->size();
  }
  points.reserve(point_count);
  for (const SavedKeyframe &keyframe : map.keyframes) {
    for (const PointType &point : keyframe.cloud->points) {
      const Eigen::Vector3d local(point.x, point.y, point.z);
      const Eigen::Vector3d diagonal(point.normal_x, point.normal_y,
                                     point.normal_z);
      if (!local.array().isFinite().all() ||
          !diagonal.array().isFinite().all() ||
          (diagonal.array() < 0.0).any() ||
          !std::isfinite(static_cast<double>(point.intensity))) {
        return fail(error,
                    "keyframe contains invalid point covariance for spatial map");
      }
      SpatialMapPoint transformed;
      transformed.position = keyframe.pose.R * local + keyframe.pose.t;
      transformed.covariance =
          keyframe.pose.R * diagonal.asDiagonal() * keyframe.pose.R.transpose();
      transformed.intensity = point.intensity;
      if (!transformed.position.array().isFinite().all() ||
          !transformed.covariance.array().isFinite().all()) {
        return fail(error, "keyframe transform produced invalid spatial point");
      }
      points.push_back(transformed);
    }
  }
  return SpatialMapBuilder::build(points, config, result, error);
}

void appendConfigField(std::ostringstream &serialized, const char *name,
                       int value) {
  serialized << name << '=' << value << '\n';
}

void appendConfigField(std::ostringstream &serialized, const char *name,
                       float value) {
  serialized << name << '=' << value << '\n';
}

std::string serializeConfig(const ConfigSetting &config) {
  std::ostringstream serialized;
  serialized.imbue(std::locale::classic());
  serialized << std::scientific
             << std::setprecision(std::numeric_limits<float>::max_digits10);
  appendConfigField(serialized, "useful_corner_num_",
                    config.useful_corner_num_);
  appendConfigField(serialized, "plane_merge_normal_thre_",
                    config.plane_merge_normal_thre_);
  appendConfigField(serialized, "plane_merge_dis_thre_",
                    config.plane_merge_dis_thre_);
  appendConfigField(serialized, "plane_detection_thre_",
                    config.plane_detection_thre_);
  appendConfigField(serialized, "voxel_size_", config.voxel_size_);
  appendConfigField(serialized, "voxel_init_num_", config.voxel_init_num_);
  appendConfigField(serialized, "proj_plane_num_", config.proj_plane_num_);
  appendConfigField(serialized, "proj_image_resolution_",
                    config.proj_image_resolution_);
  appendConfigField(serialized, "proj_image_high_inc_",
                    config.proj_image_high_inc_);
  appendConfigField(serialized, "proj_dis_min_", config.proj_dis_min_);
  appendConfigField(serialized, "proj_dis_max_", config.proj_dis_max_);
  appendConfigField(serialized, "summary_min_thre_",
                    config.summary_min_thre_);
  appendConfigField(serialized, "line_filter_enable_",
                    config.line_filter_enable_);
  appendConfigField(serialized, "touch_filter_enable_",
                    config.touch_filter_enable_);
  appendConfigField(serialized, "descriptor_near_num_",
                    config.descriptor_near_num_);
  appendConfigField(serialized, "descriptor_min_len_",
                    config.descriptor_min_len_);
  appendConfigField(serialized, "descriptor_max_len_",
                    config.descriptor_max_len_);
  appendConfigField(serialized, "non_max_suppression_radius_",
                    config.non_max_suppression_radius_);
  appendConfigField(serialized, "std_side_resolution_",
                    config.std_side_resolution_);
  appendConfigField(serialized, "skip_near_num_", config.skip_near_num_);
  appendConfigField(serialized, "candidate_num_", config.candidate_num_);
  appendConfigField(serialized, "rough_dis_threshold_",
                    config.rough_dis_threshold_);
  appendConfigField(serialized, "similarity_threshold_",
                    config.similarity_threshold_);
  appendConfigField(serialized, "icp_threshold_", config.icp_threshold_);
  appendConfigField(serialized, "normal_threshold_", config.normal_threshold_);
  appendConfigField(serialized, "dis_threshold_", config.dis_threshold_);
  return serialized.str();
}

std::string fnv1a64(const std::string &value) {
  std::uint64_t hash = UINT64_C(14695981039346656037);
  for (const unsigned char byte : value) {
    hash ^= byte;
    hash *= UINT64_C(1099511628211);
  }
  std::ostringstream output;
  output.imbue(std::locale::classic());
  output << std::hex << std::nouppercase << std::setfill('0') << std::setw(16)
         << hash;
  return output.str();
}

bool poseIsFinite(const SavedPose &pose) {
  return std::isfinite(pose.stamp) && pose.R.allFinite() && pose.t.allFinite();
}

std::string cloudRelativePath(int id) {
  std::ostringstream path;
  path.imbue(std::locale::classic());
  path << kKeyframesDirectory << '/' << std::setfill('0') << std::setw(6) << id
       << ".pcd";
  return path.str();
}

std::string btcPlaneRelativePath(int id) {
  std::ostringstream path;
  path.imbue(std::locale::classic());
  path << kBtcPlanesDirectory << '/' << std::setfill('0') << std::setw(6)
       << id << ".pcd";
  return path.str();
}

bool planePointIsFinite(const pcl::PointXYZINormal &point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z) && std::isfinite(point.intensity) &&
         std::isfinite(point.normal_x) && std::isfinite(point.normal_y) &&
         std::isfinite(point.normal_z) && std::isfinite(point.curvature);
}

bool validateBtcMetadata(const std::vector<SavedBtcFrame> &frames,
                         std::size_t keyframe_count, std::string &error) {
  for (std::size_t index = 0; index < frames.size(); ++index) {
    const SavedBtcFrame &frame = frames[index];
    if (index > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
        frame.frame_id != static_cast<int>(index)) {
      return fail(error, "BTC frame IDs must be contiguous from 0; expected " +
                             std::to_string(index) + ", found " +
                             std::to_string(frame.frame_id));
    }
    if (frame.anchor_keyframe_id < 0 ||
        static_cast<std::size_t>(frame.anchor_keyframe_id) >= keyframe_count) {
      return fail(error, "BTC frame " + std::to_string(frame.frame_id) +
                             " anchor keyframe ID is outside saved keyframes: " +
                             std::to_string(frame.anchor_keyframe_id));
    }
  }
  return true;
}

bool validateBtcPlane(const SavedBtcFrame &frame, std::string &error) {
  if (!frame.plane_cloud) {
    return fail(error, "BTC frame " + std::to_string(frame.frame_id) +
                           " has no plane cloud");
  }
  for (std::size_t index = 0; index < frame.plane_cloud->size(); ++index) {
    if (!planePointIsFinite(frame.plane_cloud->points[index])) {
      return fail(error, "BTC frame " + std::to_string(frame.frame_id) +
                             " plane point " + std::to_string(index) +
                             " has a non-finite field");
    }
  }
  return true;
}

bool validateBtcFramesForSave(const SavedMap &map, std::string &error) {
  if (!validateBtcMetadata(map.btc_frames, map.keyframes.size(), error)) {
    return false;
  }
  for (const SavedBtcFrame &frame : map.btc_frames) {
    if (!validateBtcPlane(frame, error)) {
      return false;
    }
  }
  return true;
}

class TemporaryDirectoryGuard {
 public:
  explicit TemporaryDirectoryGuard(fs::path path)
      : path_(std::move(path)), active_(true) {}

  ~TemporaryDirectoryGuard() {
    if (active_) {
      boost::system::error_code ignored;
      fs::remove_all(path_, ignored);
    }
  }

  void release() { active_ = false; }

 private:
  fs::path path_;
  bool active_;
};

bool writeManifest(const fs::path &path, const SavedMap &map,
                   const MapPackageOptions &options,
                   const SpatialMapConfig &spatial_config,
                   std::size_t spatial_block_count,
                   const std::string &fingerprint, std::string &error) {
  std::ofstream output(path.string().c_str(), std::ios::out | std::ios::trunc);
  if (!output) {
    return fail(error, "cannot create manifest: " + path.string());
  }
  output.imbue(std::locale::classic());
  output << std::setprecision(std::numeric_limits<double>::max_digits10)
         << "format: voxel_slam_map\n"
         << "version: 2\n"
         << "coordinate_frame: map\n"
         << "keyframe_cloud_frame: imu\n"
         << "win_size: " << options.win_size << '\n'
         << "keyframe_distance_m: " << options.keyframe_distance_m << '\n'
         << "keyframe_angle_deg: " << options.keyframe_angle_deg << '\n'
         << "keyframe_voxel_size: " << options.keyframe_voxel_size << '\n'
         << "descriptor_policy: one_per_fused_keyframe\n"
         << "btc_config_fingerprint: " << fingerprint << '\n'
         << "keyframe_count: " << map.keyframes.size() << '\n'
         << "btc_frame_count: " << map.btc_frames.size() << '\n'
         << "spatial_size_x: " << spatial_config.size_x << '\n'
         << "spatial_size_y: " << spatial_config.size_y << '\n'
         << "spatial_size_z: " << spatial_config.size_z << '\n'
         << "spatial_fusion_leaf_size: "
         << spatial_config.fusion_leaf_size << '\n'
         << "spatial_root_voxel_size: "
         << spatial_config.root_voxel_size << '\n'
         << "spatial_max_layer: " << spatial_config.max_layer << '\n'
         << "spatial_min_eigen_value: "
         << spatial_config.min_eigen_value << '\n';
  for (std::size_t i = 0; i < spatial_config.min_points.size(); ++i) {
    output << "spatial_min_points_" << i << ": "
           << spatial_config.min_points[i] << '\n';
  }
  for (std::size_t i = 0;
       i < spatial_config.plane_ratio_threshold.size(); ++i) {
    output << "spatial_plane_ratio_" << i << ": "
           << spatial_config.plane_ratio_threshold[i] << '\n';
  }
  output << "spatial_save_display_cloud: "
         << (spatial_config.save_display_cloud ? 1 : 0) << '\n'
         << "spatial_block_count: " << spatial_block_count << '\n';
  output.close();
  if (!output) {
    return fail(error, "failed writing manifest: " + path.string());
  }
  return true;
}

bool validateKeyframesForSave(const SavedMap &map, std::string &error) {
  for (std::size_t index = 0; index < map.keyframes.size(); ++index) {
    const SavedKeyframe &keyframe = map.keyframes[index];
    if (keyframe.id != static_cast<int>(index)) {
      return fail(error, "keyframe IDs must be contiguous from 0; expected " +
                             std::to_string(index) + ", found " +
                             std::to_string(keyframe.id));
    }
    if (!poseIsFinite(keyframe.pose)) {
      return fail(error, "keyframe " + std::to_string(index) +
                             " has a non-finite pose");
    }
    if (!keyframe.cloud) {
      return fail(error, "keyframe " + std::to_string(index) +
                             " has no local-frame cloud");
    }
  }
  return true;
}

bool writeKeyframes(const fs::path &temporary_directory, const SavedMap &map,
                    std::string &error) {
  const fs::path csv_path = temporary_directory / kKeyframesFile;
  std::ofstream csv(csv_path.string().c_str(), std::ios::out | std::ios::trunc);
  if (!csv) {
    return fail(error, "cannot create keyframe CSV: " + csv_path.string());
  }
  csv.imbue(std::locale::classic());
  csv << std::setprecision(std::numeric_limits<double>::max_digits10);
  csv << kCsvHeader << '\n';

  for (const SavedKeyframe &keyframe : map.keyframes) {
    Eigen::Quaterniond quaternion(keyframe.pose.R);
    if (!quaternion.coeffs().allFinite() || quaternion.norm() == 0.0) {
      return fail(error, "keyframe " + std::to_string(keyframe.id) +
                             " has an invalid rotation");
    }
    quaternion.normalize();
    const std::string relative_path = cloudRelativePath(keyframe.id);
    const fs::path cloud_path = temporary_directory / fs::path(relative_path);
    if (pcl::io::savePCDFileBinary(cloud_path.string(), *keyframe.cloud) < 0) {
      return fail(error, "failed writing keyframe cloud: " + relative_path);
    }

    csv << keyframe.id << ',' << keyframe.pose.stamp << ','
        << keyframe.pose.t.x() << ',' << keyframe.pose.t.y() << ','
        << keyframe.pose.t.z() << ',' << quaternion.x() << ','
        << quaternion.y() << ',' << quaternion.z() << ',' << quaternion.w()
        << ',' << relative_path << '\n';
    if (!csv) {
      return fail(error, "failed writing keyframe CSV: " + csv_path.string());
    }
  }
  csv.close();
  if (!csv) {
    return fail(error, "failed closing keyframe CSV: " + csv_path.string());
  }
  return true;
}

bool writeBtcArtifacts(const fs::path &temporary_directory,
                       const SavedMap &map, std::string &error) {
  const fs::path planes_directory =
      temporary_directory / kBtcPlanesDirectory;
  if (!fs::create_directory(planes_directory)) {
    return fail(error, "cannot create temporary BTC plane directory");
  }
  if (!save_btc_frames((temporary_directory / kBtcFramesFile).string(),
                       map.btc_frames, error)) {
    return false;
  }
  for (const SavedBtcFrame &frame : map.btc_frames) {
    const std::string relative_path = btcPlaneRelativePath(frame.frame_id);
    const fs::path plane_path = temporary_directory / fs::path(relative_path);
    if (pcl::io::savePCDFileBinary(plane_path.string(), *frame.plane_cloud) <
        0) {
      return fail(error, "failed writing BTC plane cloud: " + relative_path);
    }
  }
  return true;
}

bool parseManifest(const fs::path &path,
                   std::map<std::string, std::string> &values,
                   std::string &error) {
  std::ifstream input(path.string().c_str());
  if (!input) {
    return fail(error, "missing manifest: " + path.string());
  }

  const std::vector<std::string> allowed_keys = {
      "format",          "version",          "coordinate_frame",
      "keyframe_cloud_frame",                 "win_size",
      "keyframe_distance_m",                  "keyframe_angle_deg",
      "keyframe_voxel_size",                  "descriptor_policy",
      "btc_config_fingerprint",               "keyframe_count",
      "btc_frame_count",                       "spatial_size_x",
      "spatial_size_y",                        "spatial_size_z",
      "spatial_fusion_leaf_size",              "spatial_root_voxel_size",
      "spatial_max_layer",                     "spatial_min_eigen_value",
      "spatial_min_points_0",                  "spatial_min_points_1",
      "spatial_min_points_2",                  "spatial_min_points_3",
      "spatial_plane_ratio_0",                 "spatial_plane_ratio_1",
      "spatial_plane_ratio_2",                 "spatial_plane_ratio_3",
      "spatial_save_display_cloud",            "spatial_block_count"};

  std::string line;
  int line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    line = trim(line);
    if (line.empty()) {
      continue;
    }
    const std::size_t separator = line.find(':');
    if (separator == std::string::npos) {
      return fail(error, "invalid manifest line " +
                             std::to_string(line_number) + ": expected key: value");
    }
    const std::string key = trim(line.substr(0, separator));
    const std::string value = trim(line.substr(separator + 1));
    if (key.empty() || value.empty()) {
      return fail(error, "invalid manifest line " +
                             std::to_string(line_number) + ": empty key or value");
    }
    if (std::find(allowed_keys.begin(), allowed_keys.end(), key) ==
        allowed_keys.end()) {
      return fail(error, "unknown manifest field: " + key);
    }
    if (!values.emplace(key, value).second) {
      return fail(error, "duplicate manifest field: " + key);
    }
  }
  if (!input.eof()) {
    return fail(error, "failed reading manifest: " + path.string());
  }
  return true;
}

bool requireManifestFields(
    const std::map<std::string, std::string> &values,
    const std::vector<std::string> &required, std::string &error) {
  for (const std::string &key : required) {
    if (values.find(key) == values.end()) {
      return fail(error, "missing manifest field: " + key);
    }
  }
  return true;
}

bool expectManifestValue(const std::map<std::string, std::string> &values,
                         const std::string &key, const std::string &expected,
                         std::string &error) {
  const std::string &actual = values.at(key);
  if (actual != expected) {
    return fail(error, "manifest " + key + " mismatch: expected " + expected +
                           ", found " + actual);
  }
  return true;
}

bool parseManifestInt(const std::map<std::string, std::string> &values,
                      const std::string &key, int &value, std::string &error) {
  if (!parseInt(values.at(key), value)) {
    return fail(error, "invalid integer manifest field " + key + ": " +
                           values.at(key));
  }
  return true;
}

bool parseManifestDouble(const std::map<std::string, std::string> &values,
                         const std::string &key, double &value,
                         std::string &error) {
  if (!parseDouble(values.at(key), value)) {
    return fail(error, "invalid or non-finite manifest field " + key + ": " +
                           values.at(key));
  }
  return true;
}

bool validateManifest(const std::map<std::string, std::string> &values,
                      const MapPackageOptions &options, int &keyframe_count,
                      int &btc_frame_count, SpatialMapConfig &spatial_config,
                      int &spatial_block_count,
                      std::string &error) {
  const std::vector<std::string> base_required = {
      "format", "version", "coordinate_frame", "keyframe_cloud_frame",
      "win_size", "keyframe_distance_m", "keyframe_angle_deg",
      "keyframe_voxel_size", "descriptor_policy",
      "btc_config_fingerprint", "keyframe_count", "btc_frame_count"};
  if (!requireManifestFields(values, base_required, error)) {
    return false;
  }
  if (values.at("version") == "1") {
    return fail(error,
                "legacy map package version 1 is not loadable for localization; "
                "run build_spatial_map to convert it to version 2");
  }
  const std::vector<std::string> spatial_required = {
      "spatial_size_x", "spatial_size_y", "spatial_size_z",
      "spatial_fusion_leaf_size", "spatial_root_voxel_size",
      "spatial_max_layer", "spatial_min_eigen_value",
      "spatial_min_points_0", "spatial_min_points_1",
      "spatial_min_points_2", "spatial_min_points_3",
      "spatial_plane_ratio_0", "spatial_plane_ratio_1",
      "spatial_plane_ratio_2", "spatial_plane_ratio_3",
      "spatial_save_display_cloud", "spatial_block_count"};
  if (!requireManifestFields(values, spatial_required, error)) {
    return false;
  }
  if (!expectManifestValue(values, "format", "voxel_slam_map", error) ||
      !expectManifestValue(values, "version", "2", error) ||
      !expectManifestValue(values, "coordinate_frame", "map", error) ||
      !expectManifestValue(values, "keyframe_cloud_frame", "imu", error) ||
      !expectManifestValue(values, "descriptor_policy",
                           "one_per_fused_keyframe", error)) {
    return false;
  }

  int win_size = 0;
  double keyframe_distance_m = 0.0;
  double keyframe_angle_deg = 0.0;
  double keyframe_voxel_size = 0.0;
  if (!parseManifestInt(values, "win_size", win_size, error) ||
      !parseManifestDouble(values, "keyframe_distance_m", keyframe_distance_m,
                           error) ||
      !parseManifestDouble(values, "keyframe_angle_deg", keyframe_angle_deg,
                           error) ||
      !parseManifestDouble(values, "keyframe_voxel_size", keyframe_voxel_size,
                           error) ||
      !parseManifestInt(values, "keyframe_count", keyframe_count, error) ||
      !parseManifestInt(values, "btc_frame_count", btc_frame_count, error) ||
      !parseManifestDouble(values, "spatial_size_x", spatial_config.size_x,
                           error) ||
      !parseManifestDouble(values, "spatial_size_y", spatial_config.size_y,
                           error) ||
      !parseManifestDouble(values, "spatial_size_z", spatial_config.size_z,
                           error) ||
      !parseManifestDouble(values, "spatial_fusion_leaf_size",
                           spatial_config.fusion_leaf_size, error) ||
      !parseManifestDouble(values, "spatial_root_voxel_size",
                           spatial_config.root_voxel_size, error) ||
      !parseManifestInt(values, "spatial_max_layer",
                        spatial_config.max_layer, error) ||
      !parseManifestDouble(values, "spatial_min_eigen_value",
                           spatial_config.min_eigen_value, error) ||
      !parseManifestInt(values, "spatial_block_count", spatial_block_count,
                        error)) {
    return false;
  }
  for (std::size_t i = 0; i < spatial_config.min_points.size(); ++i) {
    if (!parseManifestInt(values, "spatial_min_points_" + std::to_string(i),
                          spatial_config.min_points[i], error) ||
        !parseManifestDouble(
            values, "spatial_plane_ratio_" + std::to_string(i),
            spatial_config.plane_ratio_threshold[i], error)) {
      return false;
    }
  }
  int save_display_cloud = 0;
  if (!parseManifestInt(values, "spatial_save_display_cloud",
                        save_display_cloud, error) ||
      (save_display_cloud != 0 && save_display_cloud != 1)) {
    return fail(error, "invalid spatial_save_display_cloud manifest field");
  }
  spatial_config.save_display_cloud = save_display_cloud != 0;
  if (keyframe_count < 0 || btc_frame_count < 0 ||
      spatial_block_count <= 0) {
    return fail(error,
                "manifest keyframe/BTC counts must be non-negative and "
                "spatial_block_count must be positive");
  }
  std::string spatial_error;
  if (!spatial_config.validate(spatial_error)) {
    return fail(error, "invalid manifest spatial map configuration: " +
                           spatial_error);
  }
  if (win_size != options.win_size) {
    return fail(error, "manifest win_size does not match load options");
  }
  if (keyframe_distance_m != options.keyframe_distance_m ||
      keyframe_angle_deg != options.keyframe_angle_deg ||
      keyframe_voxel_size != options.keyframe_voxel_size) {
    return fail(error, "manifest keyframe options do not match load options");
  }
  const std::string expected_fingerprint =
      btc_config_fingerprint(options.btc_config);
  if (values.at("btc_config_fingerprint") != expected_fingerprint) {
    return fail(error,
                "manifest btc_config_fingerprint does not match load options");
  }
  return true;
}

bool validateLegacyManifest(
    const std::map<std::string, std::string> &values,
    MapPackageOptions &options, std::string &fingerprint,
    int &keyframe_count, int &btc_frame_count, std::string &error) {
  const std::vector<std::string> required = {
      "format", "version", "coordinate_frame", "keyframe_cloud_frame",
      "win_size", "keyframe_distance_m", "keyframe_angle_deg",
      "keyframe_voxel_size", "descriptor_policy",
      "btc_config_fingerprint", "keyframe_count", "btc_frame_count"};
  if (!requireManifestFields(values, required, error) ||
      !expectManifestValue(values, "format", "voxel_slam_map", error) ||
      !expectManifestValue(values, "version", "1", error) ||
      !expectManifestValue(values, "coordinate_frame", "map", error) ||
      !expectManifestValue(values, "keyframe_cloud_frame", "imu", error) ||
      !expectManifestValue(values, "descriptor_policy",
                           "one_per_fused_keyframe", error) ||
      !parseManifestInt(values, "win_size", options.win_size, error) ||
      !parseManifestDouble(values, "keyframe_distance_m",
                           options.keyframe_distance_m, error) ||
      !parseManifestDouble(values, "keyframe_angle_deg",
                           options.keyframe_angle_deg, error) ||
      !parseManifestDouble(values, "keyframe_voxel_size",
                           options.keyframe_voxel_size, error) ||
      !parseManifestInt(values, "keyframe_count", keyframe_count, error) ||
      !parseManifestInt(values, "btc_frame_count", btc_frame_count, error)) {
    return false;
  }
  if (keyframe_count < 0 || btc_frame_count < 0) {
    return fail(error, "legacy manifest counts must be non-negative");
  }
  fingerprint = values.at("btc_config_fingerprint");
  if (fingerprint.empty()) {
    return fail(error, "legacy manifest has an empty BTC fingerprint");
  }
  return true;
}

std::vector<std::string> splitCsv(const std::string &line) {
  std::vector<std::string> fields;
  std::size_t begin = 0;
  while (begin <= line.size()) {
    const std::size_t comma = line.find(',', begin);
    if (comma == std::string::npos) {
      fields.push_back(trim(line.substr(begin)));
      break;
    }
    fields.push_back(trim(line.substr(begin, comma - begin)));
    begin = comma + 1;
  }
  return fields;
}

bool loadKeyframes(const fs::path &directory, int expected_count,
                   SavedMap &loaded, std::string &error) {
  const fs::path csv_path = directory / kKeyframesFile;
  std::ifstream csv(csv_path.string().c_str());
  if (!csv) {
    return fail(error, "missing keyframe CSV: " + csv_path.string());
  }
  csv.imbue(std::locale::classic());
  std::string line;
  if (!std::getline(csv, line) || trim(line) != kCsvHeader) {
    return fail(error, "invalid keyframe CSV header: " + csv_path.string());
  }

  int line_number = 1;
  while (std::getline(csv, line)) {
    ++line_number;
    if (trim(line).empty()) {
      continue;
    }
    const std::vector<std::string> fields = splitCsv(line);
    if (fields.size() != 10) {
      return fail(error, "invalid keyframe CSV line " +
                             std::to_string(line_number) +
                             ": expected 10 fields");
    }

    SavedKeyframe keyframe;
    double tx = 0.0;
    double ty = 0.0;
    double tz = 0.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 0.0;
    if (!parseInt(fields[0], keyframe.id) ||
        !parseDouble(fields[1], keyframe.pose.stamp) ||
        !parseDouble(fields[2], tx) || !parseDouble(fields[3], ty) ||
        !parseDouble(fields[4], tz) || !parseDouble(fields[5], qx) ||
        !parseDouble(fields[6], qy) || !parseDouble(fields[7], qz) ||
        !parseDouble(fields[8], qw)) {
      return fail(error, "invalid or non-finite numeric value in keyframe CSV line " +
                             std::to_string(line_number));
    }

    const int expected_id = static_cast<int>(loaded.keyframes.size());
    if (keyframe.id != expected_id) {
      return fail(error, "keyframe IDs must be contiguous from 0; expected " +
                             std::to_string(expected_id) + ", found " +
                             std::to_string(keyframe.id));
    }
    const std::string expected_path = cloudRelativePath(keyframe.id);
    if (fields[9] != expected_path) {
      return fail(error, "keyframe " + std::to_string(keyframe.id) +
                             " cloud path mismatch: expected " + expected_path +
                             ", found " + fields[9]);
    }

    Eigen::Quaterniond quaternion(qw, qx, qy, qz);
    if (!quaternion.coeffs().allFinite() || quaternion.norm() == 0.0) {
      return fail(error, "keyframe " + std::to_string(keyframe.id) +
                             " has an invalid rotation");
    }
    quaternion.normalize();
    keyframe.pose.R = quaternion.toRotationMatrix();
    keyframe.pose.t = Eigen::Vector3d(tx, ty, tz);

    const fs::path cloud_path = directory / fs::path(expected_path);
    if (!fs::is_regular_file(cloud_path)) {
      return fail(error, "missing keyframe cloud: " + expected_path);
    }
    keyframe.cloud.reset(new pcl::PointCloud<PointType>());
    if (pcl::io::loadPCDFile<PointType>(cloud_path.string(), *keyframe.cloud) <
        0) {
      return fail(error, "failed reading keyframe cloud: " + expected_path);
    }
    loaded.keyframes.push_back(std::move(keyframe));
  }
  if (!csv.eof()) {
    return fail(error, "failed reading keyframe CSV: " + csv_path.string());
  }
  if (static_cast<int>(loaded.keyframes.size()) != expected_count) {
    return fail(error, "keyframe count mismatch: manifest declares " +
                           std::to_string(expected_count) + ", CSV contains " +
                           std::to_string(loaded.keyframes.size()));
  }
  return true;
}

bool loadBtcArtifacts(const fs::path &directory, int expected_count,
                      std::size_t keyframe_count, SavedMap &loaded,
                      std::string &error) {
  const fs::path frames_path = directory / kBtcFramesFile;
  const fs::path planes_directory = directory / kBtcPlanesDirectory;
  const bool frames_exist = fs::exists(frames_path);
  const bool planes_exist = fs::exists(planes_directory);
  if (expected_count == 0 && !frames_exist && !planes_exist) {
    return true;
  }
  if (!fs::is_regular_file(frames_path)) {
    return fail(error, "missing BTC binary: " + std::string(kBtcFramesFile));
  }
  if (!fs::is_directory(planes_directory)) {
    return fail(error, "missing BTC plane directory: " +
                           std::string(kBtcPlanesDirectory));
  }

  std::vector<SavedBtcFrame> frames;
  if (!load_btc_frames(frames_path.string(), frames, error)) {
    return false;
  }
  if (frames.size() != static_cast<std::size_t>(expected_count)) {
    return fail(error, "BTC frame count mismatch: manifest declares " +
                           std::to_string(expected_count) +
                           ", binary contains " +
                           std::to_string(frames.size()));
  }
  if (!validateBtcMetadata(frames, keyframe_count, error)) {
    return false;
  }

  std::set<std::string> expected_files;
  for (const SavedBtcFrame &frame : frames) {
    expected_files.insert(fs::path(btcPlaneRelativePath(frame.frame_id))
                              .filename()
                              .string());
  }
  for (fs::directory_iterator entry(planes_directory), end; entry != end;
       ++entry) {
    const std::string filename = entry->path().filename().string();
    if (expected_files.erase(filename) == 0) {
      return fail(error, "unexpected BTC plane path: " +
                             std::string(kBtcPlanesDirectory) + '/' +
                             filename);
    }
    if (!fs::is_regular_file(entry->path())) {
      return fail(error, "BTC plane path is not a regular file: " +
                             std::string(kBtcPlanesDirectory) + '/' +
                             filename);
    }
  }
  if (!expected_files.empty()) {
    return fail(error, "missing BTC plane cloud: " +
                           std::string(kBtcPlanesDirectory) + '/' +
                           *expected_files.begin());
  }

  for (SavedBtcFrame &frame : frames) {
    const std::string relative_path = btcPlaneRelativePath(frame.frame_id);
    const fs::path plane_path = directory / fs::path(relative_path);
    frame.plane_cloud.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
    if (pcl::io::loadPCDFile<pcl::PointXYZINormal>(plane_path.string(),
                                                   *frame.plane_cloud) < 0) {
      return fail(error, "failed reading BTC plane cloud: " + relative_path);
    }
    if (!validateBtcPlane(frame, error)) {
      return false;
    }
    frame.plane_cloud->header.seq =
        static_cast<std::uint32_t>(frame.anchor_keyframe_id);
  }
  loaded.btc_frames = std::move(frames);
  return true;
}

}  // namespace

std::string btc_config_fingerprint(const ConfigSetting &config) {
  std::string error;
  if (!validateConfig(config, error)) {
    return std::string();
  }
  return fnv1a64(serializeConfig(config));
}

namespace {

bool savePackageInternal(const std::string &directory, const SavedMap &map,
                         const MapPackageOptions &options,
                         const std::string &fingerprint_override,
                         std::string &error) {
  error.clear();
  try {
    if (directory.empty()) {
      return fail(error, "map package destination is empty");
    }
    if (!validateOptions(options, error)) {
      return false;
    }
    if (!validateKeyframesForSave(map, error)) {
      return false;
    }
    if (!validateBtcFramesForSave(map, error)) {
      return false;
    }

    const fs::path final_directory(directory);
    if (fs::exists(final_directory)) {
      return fail(error, "map package destination already exists: " +
                             final_directory.string());
    }
    fs::path parent = final_directory.parent_path();
    if (parent.empty()) {
      parent = fs::current_path();
    }
    if (!fs::is_directory(parent)) {
      return fail(error, "map package parent directory does not exist: " +
                             parent.string());
    }

    const fs::path temporary_directory =
        parent / (final_directory.filename().string() + ".tmp-" +
                  fs::unique_path("%%%%-%%%%-%%%%").string());
    if (!fs::create_directory(temporary_directory)) {
      return fail(error, "cannot create temporary map package: " +
                             temporary_directory.string());
    }
    TemporaryDirectoryGuard cleanup(temporary_directory);
    if (!fs::create_directory(temporary_directory / kKeyframesDirectory)) {
      return fail(error, "cannot create temporary keyframe directory");
    }

    const std::string fingerprint = fingerprint_override.empty()
                                        ? btc_config_fingerprint(
                                              options.btc_config)
                                        : fingerprint_override;
    if (fingerprint.empty()) {
      return fail(error, "cannot fingerprint non-finite BTC config");
    }
    SpatialMapBuildResult spatial_result;
    if (!buildSpatialMap(map, options.spatial_map, spatial_result, error)) {
      return false;
    }
    if (spatial_result.blocks.empty()) {
      return fail(error,
                  "spatial map contains no usable planar blocks; adjust "
                  "MapBlock thresholds or provide denser mapping data");
    }
    if (!writeKeyframes(temporary_directory, map, error) ||
        !writeBtcArtifacts(temporary_directory, map, error)) {
      return false;
    }
    SpatialMapCatalog spatial_catalog;
    if (!SpatialMapIO::save(temporary_directory.string(),
                            spatial_result.blocks, options.spatial_map,
                            spatial_catalog, error) ||
        !writeManifest(temporary_directory / kManifestFile, map, options,
                       options.spatial_map, spatial_catalog.entries.size(),
                       fingerprint, error)) {
      return false;
    }

    if (fs::exists(final_directory)) {
      return fail(error, "map package destination already exists: " +
                             final_directory.string());
    }
    fs::rename(temporary_directory, final_directory);
    cleanup.release();
    return true;
  } catch (const std::exception &exception) {
    return fail(error, std::string("map package save failed: ") +
                           exception.what());
  }
}

}  // namespace

bool MapPackageIO::save(const std::string &directory, const SavedMap &map,
                        const MapPackageOptions &options,
                        std::string &error) {
  return savePackageInternal(directory, map, options, std::string(), error);
}

bool MapPackageIO::load(const std::string &directory,
                        LoadedMapPackage &package,
                        const MapPackageOptions &options,
                        std::string &error) {
  error.clear();
  try {
    if (!validateRuntimeOptions(options, error)) {
      return false;
    }
    const fs::path package_directory(directory);
    if (!fs::is_directory(package_directory)) {
      return fail(error, "map package directory does not exist: " + directory);
    }

    std::map<std::string, std::string> manifest;
    if (!parseManifest(package_directory / kManifestFile, manifest, error)) {
      return false;
    }
    int keyframe_count = 0;
    int btc_frame_count = 0;
    int spatial_block_count = 0;
    SpatialMapConfig manifest_spatial_config;
    if (!validateManifest(manifest, options, keyframe_count, btc_frame_count,
                          manifest_spatial_config, spatial_block_count,
                          error)) {
      return false;
    }

    SavedMap loaded;
    if (!loadKeyframes(package_directory, keyframe_count, loaded, error)) {
      return false;
    }
    if (!loadBtcArtifacts(package_directory, btc_frame_count,
                          loaded.keyframes.size(), loaded, error)) {
      return false;
    }
    SpatialMapCatalog spatial_catalog;
    if (!SpatialMapIO::load_catalog(directory, spatial_catalog, error)) {
      return false;
    }
    if (!spatialConfigsEqual(manifest_spatial_config,
                             spatial_catalog.config)) {
      return fail(error,
                  "manifest spatial configuration does not match block index");
    }
    if (spatial_catalog.entries.size() !=
        static_cast<std::size_t>(spatial_block_count)) {
      return fail(error, "manifest spatial block count does not match index");
    }
    LoadedMapPackage candidate;
    candidate.historical_map = std::move(loaded);
    candidate.spatial_catalog = std::move(spatial_catalog);
    candidate.directory = directory;
    package = std::move(candidate);
    return true;
  } catch (const std::exception &exception) {
    return fail(error, std::string("map package load failed: ") +
                           exception.what());
  }
}

bool MapPackageIO::load(const std::string &directory, SavedMap &map,
                        const MapPackageOptions &options,
                        std::string &error) {
  LoadedMapPackage loaded;
  if (!load(directory, loaded, options, error)) {
    return false;
  }
  map = std::move(loaded.historical_map);
  return true;
}

bool MapPackageIO::load_legacy_v1(const std::string &directory,
                                  LoadedLegacyMapPackage &package,
                                  std::string &error) {
  error.clear();
  try {
    const fs::path package_directory(directory);
    if (!fs::is_directory(package_directory)) {
      return fail(error, "legacy map package directory does not exist: " +
                             directory);
    }
    std::map<std::string, std::string> manifest;
    if (!parseManifest(package_directory / kManifestFile, manifest, error)) {
      return false;
    }
    LoadedLegacyMapPackage candidate;
    int keyframe_count = 0;
    int btc_frame_count = 0;
    if (!validateLegacyManifest(manifest, candidate.options,
                                candidate.btc_config_fingerprint,
                                keyframe_count, btc_frame_count, error)) {
      return false;
    }
    if (!loadKeyframes(package_directory, keyframe_count,
                       candidate.historical_map, error) ||
        !loadBtcArtifacts(package_directory, btc_frame_count,
                          candidate.historical_map.keyframes.size(),
                          candidate.historical_map, error)) {
      return false;
    }
    package = std::move(candidate);
    return true;
  } catch (const std::exception &exception) {
    return fail(error, std::string("legacy map package load failed: ") +
                           exception.what());
  }
}

bool MapPackageIO::save_converted_legacy(
    const std::string &directory, const LoadedLegacyMapPackage &legacy,
    const SpatialMapConfig &spatial_config, std::string &error) {
  MapPackageOptions options = legacy.options;
  options.spatial_map = spatial_config;
  return savePackageInternal(directory, legacy.historical_map, options,
                             legacy.btc_config_fingerprint, error);
}
