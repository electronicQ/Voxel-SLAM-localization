#pragma once

#include "map_package.hpp"
#include "voxel_map.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <vector>

bool captureAcceptedKeyframe(
    bool accepted, const Keyframe &keyframe,
    const std::vector<STD> &descriptors,
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &plane_cloud,
    SavedMap &saved_map, std::vector<const Keyframe *> &final_pose_sources,
    std::string &error);

bool refreshCapturedKeyframePoses(
    const std::vector<const Keyframe *> &final_pose_sources,
    SavedMap &saved_map, std::string &error);
