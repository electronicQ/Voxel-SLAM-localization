#pragma once

#include "map_package.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <vector>

struct CapturedPoseUpdate {
  int capture_id = -1;
  SavedPose pose;
};

bool captureAcceptedKeyframe(
    bool accepted, const SavedPose &pose,
    const pcl::PointCloud<PointType>::Ptr &keyframe_cloud,
    const std::vector<STD> &descriptors,
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &plane_cloud,
    SavedMap &saved_map, int &capture_id, std::string &error);

bool refreshCapturedKeyframePoses(
    const std::vector<CapturedPoseUpdate> &pose_updates,
    SavedMap &saved_map, std::string &error);
