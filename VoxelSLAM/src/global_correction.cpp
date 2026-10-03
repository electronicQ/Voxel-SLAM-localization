#include "global_correction.hpp"

#include <cmath>
#include <exception>
#include <new>
#include <utility>

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

bool validState(const CorrectionState &state) {
  return validRotation(state.R) && state.p.array().isFinite().all() &&
         state.v.array().isFinite().all() &&
         state.bg.array().isFinite().all() &&
         state.ba.array().isFinite().all() &&
         state.g.array().isFinite().all() &&
         state.cov.array().isFinite().all();
}

bool validPoints(const CorrectionPointVector &points) {
  for (const Eigen::Vector3d &point : points) {
    if (!point.array().isFinite().all()) {
      return false;
    }
  }
  return true;
}

bool validTargets(const CorrectionTargets &targets) {
  if (!validState(targets.current_state)) {
    return false;
  }
  for (const CorrectionState &state : targets.window_states) {
    if (!validState(state)) {
      return false;
    }
  }
  for (const CorrectionState &state : targets.cached_states) {
    if (!validState(state)) {
      return false;
    }
  }
  return validPoints(targets.world_points) &&
         validPoints(targets.local_points);
}

void correctState(const GlobalCorrectionTransform &correction,
                  bool rotate_gravity,
                  const Eigen::Matrix<double, 15, 15> &covariance_adjoint,
                  CorrectionState &state) {
  state.R = correction.R * state.R;
  state.p = correction.R * state.p + correction.t;
  state.v = correction.R * state.v;
  if (rotate_gravity) {
    state.g = correction.R * state.g;
  }
  state.cov = covariance_adjoint * state.cov * covariance_adjoint.transpose();
}

}  // namespace

bool apply_global_correction_once(
    const GlobalCorrectionTransform &T_W_O, std::uint64_t generation,
    CorrectionTargets &targets, std::string &error) {
  error.clear();
  if (generation == 0) {
    return fail(error, "global-correction generation must be non-zero");
  }
  if (targets.has_applied_generation &&
      generation <= targets.last_applied_generation) {
    return fail(error, "global-correction generation is duplicate or stale");
  }
  if (!validRotation(T_W_O.R) || !T_W_O.t.array().isFinite().all()) {
    return fail(error, "global correction is invalid or non-finite");
  }
  if (!validTargets(targets)) {
    return fail(error, "global-correction targets are invalid or non-finite");
  }

  try {
    CorrectionTargets candidate = targets;
    Eigen::Matrix<double, 15, 15> covariance_adjoint =
        Eigen::Matrix<double, 15, 15>::Identity();
    covariance_adjoint.block<3, 3>(3, 3) = T_W_O.R;
    covariance_adjoint.block<3, 3>(6, 6) = T_W_O.R;

    correctState(T_W_O, candidate.rotate_gravity, covariance_adjoint,
                 candidate.current_state);
    for (CorrectionState &state : candidate.window_states) {
      correctState(T_W_O, candidate.rotate_gravity, covariance_adjoint, state);
    }
    for (CorrectionState &state : candidate.cached_states) {
      correctState(T_W_O, candidate.rotate_gravity, covariance_adjoint, state);
    }
    for (Eigen::Vector3d &point : candidate.world_points) {
      point = T_W_O.R * point + T_W_O.t;
    }
    candidate.has_applied_generation = true;
    candidate.last_applied_generation = generation;

    if (!validTargets(candidate)) {
      return fail(error, "global correction produced an invalid target state");
    }
    targets = std::move(candidate);
    return true;
  } catch (const std::bad_alloc &) {
    return fail(error, "out of memory while applying global correction");
  } catch (const std::exception &exception) {
    return fail(error, std::string("failed to apply global correction: ") +
                           exception.what());
  } catch (...) {
    return fail(error, "unknown failure while applying global correction");
  }
}
