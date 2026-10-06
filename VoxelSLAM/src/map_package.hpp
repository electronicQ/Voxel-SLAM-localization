#pragma once

#include "BTC.h"
#include "feature_point.hpp"
#include "spatial_map_io.hpp"

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <vector>

struct SavedPose {
  double stamp = 0.0;
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
};

struct SavedKeyframe {
  int id = -1;
  SavedPose pose;
  pcl::PointCloud<PointType>::Ptr cloud;
};

struct SavedBtcFrame {
  int frame_id = -1;
  int anchor_keyframe_id = -1;
  std::vector<STD> descriptors;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr plane_cloud;
};

struct SavedMap {
  std::vector<SavedKeyframe> keyframes;
  std::vector<SavedBtcFrame> btc_frames;
};

struct MapPackageOptions {
  int win_size = 10;
  double keyframe_distance_m = 0.1;
  double keyframe_angle_deg = 5.0;
  double keyframe_voxel_size = 0.1;
  ConfigSetting btc_config{};
  SpatialMapConfig spatial_map{};
};

struct LoadedMapPackage {
  SavedMap historical_map;
  SpatialMapCatalog spatial_catalog;
  std::string directory;
};

struct LoadedLegacyMapPackage {
  SavedMap historical_map;
  MapPackageOptions options;
  std::string btc_config_fingerprint;
};

class MapPackageIO {
 public:
  static bool save(const std::string &directory, const SavedMap &map,
                   const MapPackageOptions &options, std::string &error);
  static bool load(const std::string &directory, SavedMap &map,
                   const MapPackageOptions &options, std::string &error);

  static bool load(const std::string &directory, LoadedMapPackage &package,
                   const MapPackageOptions &options, std::string &error);

  // Converter-only compatibility path. Localization must use the v2 loader.
  static bool load_legacy_v1(const std::string &directory,
                             LoadedLegacyMapPackage &package,
                             std::string &error);

  static bool save_converted_legacy(
      const std::string &directory, const LoadedLegacyMapPackage &legacy,
      const SpatialMapConfig &spatial_config, std::string &error);
};

std::string btc_config_fingerprint(const ConfigSetting &config);
