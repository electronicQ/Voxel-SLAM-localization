#pragma once

#include "map_package.hpp"

#include <Eigen/Core>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

struct PriorMapCacheUpdate {
  std::vector<int> loaded_ids;
  std::vector<int> unloaded_ids;
  std::vector<int> active_ids;
  pcl::PointCloud<PointType>::ConstPtr world_cloud;
};

struct TrackingQuality {
  int matched_points = 0;
  double mean_abs_point_plane = 0.0;
  double min_normal_eigenvalue = 0.0;
  bool finite = false;
  bool solve_succeeded = false;
};

class PriorMapTracker {
 public:
  PriorMapTracker();

  // The SavedMap remains caller-owned and must outlive this tracker state.
  bool initialize(const SavedMap &map, std::string &error);

  std::vector<int> radius_search_xy(const Eigen::Vector2d &center,
                                    double radius) const;
  bool nearest_height_xy(const Eigen::Vector2d &center, double &height,
                         int &keyframe_id, std::string &error) const;

  bool rebuild_local_map(const Eigen::Vector3d &center, double radius,
                         int preload_limit, PriorMapCacheUpdate &update,
                         std::string &error);
  bool update_loaded_keyframes(const Eigen::Vector3d &center,
                               double load_radius, double unload_radius,
                               int max_load_per_scan,
                               PriorMapCacheUpdate &update,
                               std::string &error);
  std::vector<int> loaded_keyframe_ids() const;
  pcl::PointCloud<PointType>::ConstPtr loaded_world_cloud() const;

  void reset_runtime_cache();
  const SavedMap *source_map() const;
  std::uint64_t generation() const;

 private:
  struct IndexState {
    IndexState();

    const SavedMap *source_map = nullptr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr xy_points;
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_points;
    pcl::KdTreeFLANN<pcl::PointXYZ> xy_index;
    pcl::KdTreeFLANN<pcl::PointXYZ> xyz_index;
    std::vector<int> xy_index_to_id;
    std::vector<int> xyz_index_to_id;
    std::vector<double> xy_index_to_height;
    std::unordered_map<int, std::size_t> id_to_keyframe_index;
  };

  std::shared_ptr<IndexState> index_state_;
  std::unordered_set<int> loaded_keyframe_ids_;
  pcl::PointCloud<PointType>::Ptr loaded_world_cloud_;
  std::uint64_t generation_ = 0;
};
