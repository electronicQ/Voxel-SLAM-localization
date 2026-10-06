#pragma once

#include "map_package.hpp"

#include <Eigen/Core>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

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

  const SavedMap *source_map() const;
  std::uint64_t generation() const;

 private:
  struct IndexState {
    IndexState();

    const SavedMap *source_map = nullptr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr xy_points;
    pcl::KdTreeFLANN<pcl::PointXYZ> xy_index;
    std::vector<int> xy_index_to_id;
    std::vector<double> xy_index_to_height;
  };

  std::shared_ptr<IndexState> index_state_;
  std::uint64_t generation_ = 0;
};
