#include "session_aligner.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <Eigen/Geometry>

#include <cmath>
#include <cstddef>
#include <exception>
#include <new>
#include <utility>

namespace {

bool fail(std::string &error, const std::string &message) {
  error = message;
  return false;
}

bool validRotation(const Eigen::Matrix3d &rotation) {
  if (!rotation.array().isFinite().all()) {
    return false;
  }
  return rotation.transpose()
             .isApprox(Eigen::Matrix3d::Identity(), 1e-6) &&
         std::abs(rotation.determinant() - 1.0) <= 1e-6;
}

bool finitePose(const SavedPose &pose) {
  return std::isfinite(pose.stamp) && validRotation(pose.R) &&
         pose.t.array().isFinite().all();
}

gtsam::Pose3 toGtsam(const SavedPose &pose) {
  return gtsam::Pose3(gtsam::Rot3(pose.R),
                      gtsam::Point3(pose.t.x(), pose.t.y(), pose.t.z()));
}

SavedPose fromGtsam(const gtsam::Pose3 &pose, double stamp) {
  SavedPose result;
  result.stamp = stamp;
  result.R = pose.rotation().matrix();
  result.t = pose.translation();
  return result;
}

double rotationAngleDegrees(const gtsam::Pose3 &pose) {
  const Eigen::AngleAxisd angle_axis(pose.rotation().matrix());
  return std::abs(angle_axis.angle()) * 57.2957795130823208768;
}

}  // namespace

bool align_current_session_to_map(
    const SavedPose &history_anchor, int current_anchor_index,
    const Eigen::Matrix3d &R_HC, const Eigen::Vector3d &t_HC,
    const std::vector<SavedPose> &current_odometry,
    const SessionAlignmentOptions &options, SessionAlignmentResult &result,
    std::string &error) {
  error.clear();
  if (current_odometry.empty()) {
    return fail(error, "current odometry session is empty");
  }
  if (current_anchor_index < 0 ||
      static_cast<std::size_t>(current_anchor_index) >=
          current_odometry.size()) {
    return fail(error, "current anchor index is out of range");
  }
  if (!finitePose(history_anchor)) {
    return fail(error, "history anchor pose is invalid or non-finite");
  }
  if (!validRotation(R_HC) || !t_HC.array().isFinite().all()) {
    return fail(error, "T_H_C is invalid or non-finite");
  }
  for (const SavedPose &pose : current_odometry) {
    if (!finitePose(pose)) {
      return fail(error, "current odometry contains an invalid pose");
    }
  }
  if (!std::isfinite(options.max_translation_correction_m) ||
      options.max_translation_correction_m <= 0.0 ||
      !std::isfinite(options.max_rotation_correction_deg) ||
      options.max_rotation_correction_deg <= 0.0 ||
      options.max_rotation_correction_deg > 180.0) {
    return fail(error, "session-alignment safety limits are invalid");
  }

  try {
    const gtsam::Pose3 T_W_H = toGtsam(history_anchor);
    const gtsam::Pose3 T_H_C(gtsam::Rot3(R_HC),
                             gtsam::Point3(t_HC.x(), t_HC.y(), t_HC.z()));
    const gtsam::Pose3 T_O_C_anchor =
        toGtsam(current_odometry[static_cast<std::size_t>(
            current_anchor_index)]);
    const gtsam::Pose3 T_W_C_anchor = T_W_H.compose(T_H_C);
    const gtsam::Pose3 seed_T_W_O =
        T_W_C_anchor.compose(T_O_C_anchor.inverse());
    if (seed_T_W_O.translation().norm() >
            options.max_translation_correction_m ||
        rotationAngleDegrees(seed_T_W_O) >
            options.max_rotation_correction_deg) {
      return fail(error, "session alignment exceeds configured safety limits");
    }

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    const auto history_prior =
        gtsam::noiseModel::Isotropic::Sigma(6, 1e-6);
    const auto cross_session_noise =
        gtsam::noiseModel::Isotropic::Sigma(6, 1e-4);
    const auto odometry_noise =
        gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);

    initial.insert(0, T_W_H);
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, T_W_H, history_prior));
    for (std::size_t i = 0; i < current_odometry.size(); ++i) {
      const gtsam::Pose3 T_O_C = toGtsam(current_odometry[i]);
      initial.insert(static_cast<gtsam::Key>(i + 1),
                     seed_T_W_O.compose(T_O_C));
      if (i > 0) {
        const gtsam::Pose3 previous = toGtsam(current_odometry[i - 1]);
        graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
            static_cast<gtsam::Key>(i),
            static_cast<gtsam::Key>(i + 1), previous.between(T_O_C),
            odometry_noise));
      }
    }
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
        0, static_cast<gtsam::Key>(current_anchor_index + 1), T_H_C,
        cross_session_noise));

    const gtsam::Values optimized =
        gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
    if (optimized.size() != current_odometry.size() + 1U) {
      return fail(error, "session alignment returned an unexpected pose count");
    }

    SessionAlignmentResult candidate;
    const gtsam::Pose3 optimized_history =
        optimized.at<gtsam::Pose3>(0);
    candidate.fixed_history = fromGtsam(optimized_history,
                                        history_anchor.stamp);
    if (!finitePose(candidate.fixed_history) ||
        !candidate.fixed_history.R.isApprox(history_anchor.R, 1e-6) ||
        !candidate.fixed_history.t.isApprox(history_anchor.t, 1e-6)) {
      return fail(error, "fixed history anchor moved during session alignment");
    }

    candidate.optimized_current.reserve(current_odometry.size());
    for (std::size_t i = 0; i < current_odometry.size(); ++i) {
      candidate.optimized_current.push_back(fromGtsam(
          optimized.at<gtsam::Pose3>(static_cast<gtsam::Key>(i + 1)),
          current_odometry[i].stamp));
      if (!finitePose(candidate.optimized_current.back())) {
        return fail(error, "session alignment returned a non-finite pose");
      }
    }
    const gtsam::Pose3 optimized_T_W_O =
        toGtsam(candidate.optimized_current[static_cast<std::size_t>(
            current_anchor_index)])
            .compose(T_O_C_anchor.inverse());
    candidate.T_W_O = fromGtsam(
        optimized_T_W_O,
        current_odometry[static_cast<std::size_t>(current_anchor_index)].stamp);
    if (!finitePose(candidate.T_W_O) ||
        candidate.T_W_O.t.norm() > options.max_translation_correction_m ||
        rotationAngleDegrees(optimized_T_W_O) >
            options.max_rotation_correction_deg) {
      return fail(error,
                  "optimized session alignment exceeds configured safety limits");
    }

    result = std::move(candidate);
    return true;
  } catch (const std::bad_alloc &) {
    return fail(error, "out of memory during session alignment");
  } catch (const std::exception &exception) {
    return fail(error,
                std::string("GTSAM session alignment failed: ") +
                    exception.what());
  } catch (...) {
    return fail(error, "unknown failure during session alignment");
  }
}
