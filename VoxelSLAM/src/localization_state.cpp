#include "localization_state.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

bool fail(std::string &error, const std::string &message) {
  error = message;
  return false;
}

bool validRotation(const Eigen::Matrix3d &rotation) {
  return rotation.array().isFinite().all() &&
         rotation.transpose().isApprox(Eigen::Matrix3d::Identity(), 1e-6) &&
         std::abs(rotation.determinant() - 1.0) <= 1e-6;
}

Eigen::Matrix3d rotationFromRpy(double roll, double pitch, double yaw) {
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

}  // namespace

bool InitialPoseRequestBuffer::submit(
    const std::string &frame_id, double stamp_sec, const Eigen::Vector2d &xy,
    double yaw, std::uint64_t &generation, std::string &error) {
  error.clear();
  generation = 0;
  if (frame_id != "map" && frame_id != "/map") {
    return fail(error, "RViz initial pose must use the map frame");
  }
  if (!std::isfinite(stamp_sec) || !xy.array().isFinite().all() ||
      !std::isfinite(yaw)) {
    return fail(error, "RViz initial pose contains a non-finite value");
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (pending_.generation == std::numeric_limits<std::uint64_t>::max()) {
    return fail(error, "RViz initial-pose generation is exhausted");
  }
  PendingInitialPose candidate;
  candidate.valid = true;
  candidate.generation = pending_.generation + 1;
  candidate.stamp_sec = stamp_sec;
  candidate.xy = xy;
  candidate.yaw = yaw;
  pending_ = candidate;
  generation = candidate.generation;
  return true;
}

PendingInitialPose InitialPoseRequestBuffer::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return pending_;
}

bool InitialPoseRequestBuffer::is_current(std::uint64_t generation) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return pending_.valid && pending_.generation == generation;
}

bool make_direct_pose_seed(
    const PendingInitialPose &request, bool imu_ready,
    const Eigen::Matrix3d &R_O_C, const Eigen::Vector3d &p_O_C,
    const PriorMapTracker &tracker, DirectPoseSeed &seed,
    std::string &error) {
  error.clear();
  if (!request.valid || request.generation == 0) {
    return fail(error, "RViz initial-pose request is invalid");
  }
  if (!imu_ready) {
    return fail(error, "IMU orientation is not ready");
  }
  if (!request.xy.array().isFinite().all() ||
      !std::isfinite(request.yaw) || !validRotation(R_O_C) ||
      !p_O_C.array().isFinite().all()) {
    return fail(error, "direct-pose seed input is invalid or non-finite");
  }

  double height = 0.0;
  int height_keyframe_id = -1;
  if (!tracker.nearest_height_xy(request.xy, height, height_keyframe_id,
                                 error)) {
    return false;
  }

  const double sin_pitch =
      std::max(-1.0, std::min(1.0, -R_O_C(2, 0)));
  const double pitch = std::asin(sin_pitch);
  double roll = 0.0;
  if (std::abs(std::cos(pitch)) > 1e-8) {
    roll = std::atan2(R_O_C(2, 1), R_O_C(2, 2));
  } else {
    roll = std::atan2(-R_O_C(1, 2), R_O_C(1, 1));
  }

  DirectPoseSeed candidate;
  candidate.request_generation = request.generation;
  candidate.height_keyframe_id = height_keyframe_id;
  candidate.T_W_C.stamp = request.stamp_sec;
  candidate.T_W_C.R = rotationFromRpy(roll, pitch, request.yaw);
  candidate.T_W_C.t =
      Eigen::Vector3d(request.xy.x(), request.xy.y(), height);
  candidate.T_W_O.stamp = request.stamp_sec;
  candidate.T_W_O.R = candidate.T_W_C.R * R_O_C.transpose();
  candidate.T_W_O.t = candidate.T_W_C.t -
                      candidate.T_W_O.R * p_O_C;
  if (!validRotation(candidate.T_W_O.R) ||
      !candidate.T_W_O.t.array().isFinite().all()) {
    return fail(error, "direct-pose seed produced an invalid transform");
  }

  seed = candidate;
  return true;
}
