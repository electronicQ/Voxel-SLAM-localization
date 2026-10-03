#pragma once

#include "map_package.hpp"
#include "prior_map_tracker.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <functional>
#include <mutex>
#include <string>

struct PendingInitialPose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool valid = false;
  std::uint64_t generation = 0;
  double stamp_sec = 0.0;
  Eigen::Vector2d xy = Eigen::Vector2d::Zero();
  double yaw = 0.0;
};

class InitialPoseRequestBuffer {
 public:
  bool submit(const std::string &frame_id, double stamp_sec,
              const Eigen::Vector2d &xy, double yaw,
              std::uint64_t &generation, std::string &error);
  PendingInitialPose snapshot() const;
  bool is_current(std::uint64_t generation) const;
  bool run_if_current(std::uint64_t generation,
                      const std::function<void()> &action) const;

 private:
  mutable std::mutex mutex_;
  PendingInitialPose pending_;
};

struct DirectPoseSeed {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::uint64_t request_generation = 0;
  int height_keyframe_id = -1;
  SavedPose T_W_C;
  SavedPose T_W_O;
};

bool make_direct_pose_seed(
    const PendingInitialPose &request, bool imu_ready,
    const Eigen::Matrix3d &R_O_C, const Eigen::Vector3d &p_O_C,
    const PriorMapTracker &tracker, DirectPoseSeed &seed,
    std::string &error);
