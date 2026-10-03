#pragma once

#include "map_package.hpp"

#include <Eigen/Core>

#include <string>
#include <vector>

struct SessionAlignmentOptions {
  double max_translation_correction_m = 1000000.0;
  double max_rotation_correction_deg = 180.0;
};

struct SessionAlignmentResult {
  SavedPose fixed_history;
  std::vector<SavedPose> optimized_current;
  SavedPose T_W_O;
};

bool align_current_session_to_map(
    const SavedPose &history_anchor, int current_anchor_index,
    const Eigen::Matrix3d &R_HC, const Eigen::Vector3d &t_HC,
    const std::vector<SavedPose> &current_odometry,
    const SessionAlignmentOptions &options, SessionAlignmentResult &result,
    std::string &error);
