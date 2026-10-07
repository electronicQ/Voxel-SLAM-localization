#ifndef IMU_RATE_PREDICTOR_HPP
#define IMU_RATE_PREDICTOR_HPP

#include "tools.hpp"

#include <Eigen/StdDeque>

#include <cstddef>
#include <deque>
#include <mutex>

struct ImuRateSample {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double t = 0.0;
  Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel = Eigen::Vector3d::Zero();
};

enum class ImuRateRejectReason {
  NONE,
  DISABLED,
  NO_SNAPSHOT,
  NONFINITE,
  NON_MONOTONIC_TIME,
  PREDICTION_HORIZON_EXCEEDED
};

struct ImuRatePrediction {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool publishable = false;
  IMUST state;
  Eigen::Vector3d unbiased_angular_velocity = Eigen::Vector3d::Zero();
  ImuRateRejectReason reason = ImuRateRejectReason::NO_SNAPSHOT;
};

class ImuRatePredictor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit ImuRatePredictor(double max_prediction_sec = 0.5);
  void setEnabled(bool enabled);
  void setMaxPredictionSec(double seconds);
  bool updateSnapshot(const IMUST &posterior, double acceleration_scale);
  ImuRatePrediction pushImu(const ImuRateSample &sample);
  void reset();

 private:
  using SampleDeque =
      std::deque<ImuRateSample, Eigen::aligned_allocator<ImuRateSample>>;

  static constexpr std::size_t kMaxRetainedSamples = 4096;

  ImuRatePrediction makePredictionLocked(ImuRateRejectReason reason) const;
  void clearStateLocked();

  mutable std::mutex mutex_;
  bool enabled_ = true;
  bool has_snapshot_ = false;
  bool replay_coverage_valid_ = true;
  bool has_replay_coverage_boundary_ = false;
  double max_prediction_sec_ = 0.5;
  double acceleration_scale_ = 1.0;
  double replay_coverage_boundary_ = 0.0;
  IMUST snapshot_;
  IMUST predicted_;
  SampleDeque tail_;
};

#endif
