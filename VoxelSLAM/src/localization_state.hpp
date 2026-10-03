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

enum class LocalizationState {
  WAITING_FOR_MAP,
  LOCAL_ODOMETRY,
  RELOCALIZING,
  ALIGNING,
  IN_MAP_TRACKING,
  LOST
};

enum class AlignmentSource { BTC, RVIZ_INITIAL_POSE };

enum class LocalizationEvent {
  MAP_LOADED,
  BTC_STARTED,
  BTC_MATCHED,
  BTC_FAILED,
  ALIGNMENT_APPLIED,
  DIRECT_POSE_STARTED,
  DIRECT_POSE_VALIDATED,
  DIRECT_POSE_FAILED,
  TRACKING_GOOD,
  TRACKING_BAD,
  TRACKING_NONFINITE,
  RECOVERY_STARTED
};

struct LocalizationTransition {
  bool accepted = false;
  bool cleanup_requested = false;
};

class LocalizationStateMachine {
 public:
  explicit LocalizationStateMachine(int bad_tracking_limit = 5);

  LocalizationTransition handle(LocalizationEvent event,
                                std::string &warning);
  LocalizationState state() const;
  AlignmentSource alignment_source() const;
  bool map_loaded() const;
  bool localization_valid() const;
  int bad_tracking_streak() const;
  std::uint64_t cleanup_count() const;

 private:
  LocalizationTransition reject(std::string &warning) const;
  LocalizationTransition enter_lost();

  LocalizationState state_ = LocalizationState::WAITING_FOR_MAP;
  AlignmentSource alignment_source_ = AlignmentSource::BTC;
  bool map_loaded_ = false;
  bool localization_valid_ = false;
  int bad_tracking_streak_ = 0;
  int bad_tracking_limit_ = 5;
  std::uint64_t cleanup_count_ = 0;
};

const char *localization_state_name(LocalizationState state);
