#include "imu_rate_predictor.hpp"

#include <cmath>

namespace {

bool isFinite(const ImuRateSample &sample) {
  return std::isfinite(sample.t) && sample.gyro.allFinite() &&
         sample.accel.allFinite();
}

bool isFinite(const IMUST &state) {
  return std::isfinite(state.t) && state.R.allFinite() &&
         state.p.allFinite() && state.v.allFinite() && state.bg.allFinite() &&
         state.ba.allFinite() && state.g.allFinite() && state.cov.allFinite();
}

bool propagate(const ImuRateSample &sample, double acceleration_scale,
               IMUST &state) {
  const double dt = sample.t - state.t;
  if (!std::isfinite(dt) || dt <= 0.0) {
    return false;
  }

  const Eigen::Vector3d omega = sample.gyro - state.bg;
  const Eigen::Vector3d acc_body =
      acceleration_scale * sample.accel - state.ba;
  const Eigen::Vector3d acc_world = state.R * acc_body + state.g;

  state.p += state.v * dt + 0.5 * acc_world * dt * dt;
  state.v += acc_world * dt;
  state.R = state.R * Exp(omega, dt);
  state.t = sample.t;
  return isFinite(state);
}

}  // namespace

ImuRatePredictor::ImuRatePredictor(double max_prediction_sec)
    : max_prediction_sec_(
          std::isfinite(max_prediction_sec) && max_prediction_sec > 0.0
              ? max_prediction_sec
              : 0.5) {
  snapshot_.g.setZero();
  predicted_.g.setZero();
}

void ImuRatePredictor::setEnabled(bool enabled) {
  std::lock_guard<std::mutex> lock(mutex_);
  enabled_ = enabled;
}

void ImuRatePredictor::setMaxPredictionSec(double seconds) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!std::isfinite(seconds) || seconds <= 0.0) {
    return;
  }
  max_prediction_sec_ = seconds;
}

void ImuRatePredictor::updateSnapshot(const IMUST &posterior,
                                      double acceleration_scale) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!isFinite(posterior) || !std::isfinite(acceleration_scale)) {
    return;
  }
  if (has_replay_coverage_boundary_ &&
      posterior.t < replay_coverage_boundary_) {
    replay_coverage_valid_ = false;
    return;
  }

  snapshot_ = posterior;
  predicted_ = posterior;
  acceleration_scale_ = acceleration_scale;
  has_snapshot_ = true;
  replay_coverage_valid_ = true;

  while (!tail_.empty() && tail_.front().t <= posterior.t) {
    tail_.pop_front();
  }
  if (!has_replay_coverage_boundary_ ||
      posterior.t > replay_coverage_boundary_) {
    replay_coverage_boundary_ = posterior.t;
    has_replay_coverage_boundary_ = true;
  }

  for (const ImuRateSample &sample : tail_) {
    IMUST candidate = predicted_;
    if (!propagate(sample, acceleration_scale_, candidate)) {
      clearStateLocked();
      return;
    }
    predicted_ = candidate;
  }
}

ImuRatePrediction ImuRatePredictor::pushImu(const ImuRateSample &sample) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) {
    return makePredictionLocked(ImuRateRejectReason::DISABLED);
  }
  if (!has_snapshot_) {
    return makePredictionLocked(ImuRateRejectReason::NO_SNAPSHOT);
  }
  if (!isFinite(sample)) {
    return makePredictionLocked(ImuRateRejectReason::NONFINITE);
  }
  if (sample.t <= predicted_.t) {
    return makePredictionLocked(ImuRateRejectReason::NON_MONOTONIC_TIME);
  }

  IMUST candidate = predicted_;
  if (!propagate(sample, acceleration_scale_, candidate)) {
    return makePredictionLocked(ImuRateRejectReason::NONFINITE);
  }

  predicted_ = candidate;
  tail_.push_back(sample);
  if (tail_.size() > kMaxRetainedSamples) {
    if (!has_replay_coverage_boundary_ ||
        tail_.front().t > replay_coverage_boundary_) {
      replay_coverage_boundary_ = tail_.front().t;
      has_replay_coverage_boundary_ = true;
    }
    tail_.pop_front();
  }

  ImuRatePrediction prediction;
  prediction.state = predicted_;
  prediction.unbiased_angular_velocity = sample.gyro - predicted_.bg;
  if (!replay_coverage_valid_) {
    prediction.reason = ImuRateRejectReason::NO_SNAPSHOT;
    return prediction;
  }
  if (sample.t - snapshot_.t > max_prediction_sec_) {
    prediction.reason = ImuRateRejectReason::PREDICTION_HORIZON_EXCEEDED;
    return prediction;
  }

  prediction.publishable = true;
  prediction.reason = ImuRateRejectReason::NONE;
  return prediction;
}

void ImuRatePredictor::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  clearStateLocked();
}

ImuRatePrediction ImuRatePredictor::makePredictionLocked(
    ImuRateRejectReason reason) const {
  ImuRatePrediction prediction;
  prediction.reason = reason;
  if (has_snapshot_) {
    prediction.state = predicted_;
  } else {
    prediction.state.g.setZero();
  }
  return prediction;
}

void ImuRatePredictor::clearStateLocked() {
  has_snapshot_ = false;
  replay_coverage_valid_ = true;
  has_replay_coverage_boundary_ = false;
  acceleration_scale_ = 1.0;
  replay_coverage_boundary_ = 0.0;
  snapshot_ = IMUST();
  snapshot_.g.setZero();
  predicted_ = IMUST();
  predicted_.g.setZero();
  tail_.clear();
}
