#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <cstdint>
#include <string>
#include <vector>

struct GlobalCorrectionTransform {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
};

struct CorrectionState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  Eigen::Vector3d bg = Eigen::Vector3d::Zero();
  Eigen::Vector3d ba = Eigen::Vector3d::Zero();
  Eigen::Vector3d g = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 15, 15> cov =
      Eigen::Matrix<double, 15, 15>::Identity();
};

using CorrectionStateVector =
    std::vector<CorrectionState, Eigen::aligned_allocator<CorrectionState>>;
using CorrectionPointVector =
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;

struct CorrectionTargets {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CorrectionState current_state;
  CorrectionStateVector window_states;
  CorrectionStateVector cached_states;
  CorrectionPointVector world_points;
  CorrectionPointVector local_points;
  bool rotate_gravity = true;
  bool has_applied_generation = false;
  std::uint64_t last_applied_generation = 0;
};

bool apply_global_correction_once(
    const GlobalCorrectionTransform &T_W_O, std::uint64_t generation,
    CorrectionTargets &targets, std::string &error);

template <typename PoseState>
bool apply_global_correction_once(
    const PoseState &T_W_O, std::uint64_t generation,
    CorrectionTargets &targets, std::string &error) {
  GlobalCorrectionTransform transform;
  transform.R = T_W_O.R;
  transform.t = T_W_O.p;
  return apply_global_correction_once(transform, generation, targets, error);
}
