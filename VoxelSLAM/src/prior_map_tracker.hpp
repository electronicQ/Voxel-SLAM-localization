#pragma once

#include "map_package.hpp"

#include <Eigen/Core>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

class PriorMapTracker {
 public:
  // The SavedMap remains caller-owned and must outlive this tracker state.
  bool initialize(const SavedMap &map, std::string &error);

  std::vector<int> radius_search_xy(const Eigen::Vector2d &center,
                                    double radius) const;
  bool nearest_height_xy(const Eigen::Vector2d &center, double &height,
                         int &keyframe_id, std::string &error) const;

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
  };

  std::shared_ptr<IndexState> index_state_;
  std::unordered_set<int> loaded_keyframe_ids_;
  std::uint64_t generation_ = 0;
};
