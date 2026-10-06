#include "imu_rate_predictor.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <limits>

namespace {

constexpr double kTolerance = 1e-9;

IMUST makePosterior(double t = 0.0) {
  IMUST posterior;
  posterior.t = t;
  posterior.R.setIdentity();
  posterior.p.setZero();
  posterior.v.setZero();
  posterior.bg.setZero();
  posterior.ba.setZero();
  posterior.g = Eigen::Vector3d(0.0, 0.0, -9.8);
  posterior.cov.setIdentity();
  posterior.cov *= 0.25;
  posterior.cov(0, 3) = posterior.cov(3, 0) = 0.125;
  return posterior;
}

ImuRateSample gravityBalancedSample(double t) {
  ImuRateSample sample;
  sample.t = t;
  sample.accel = Eigen::Vector3d(0.0, 0.0, 4.9);
  return sample;
}

void expectStateNear(const IMUST &actual, const IMUST &expected,
                     double tolerance = kTolerance) {
  EXPECT_NEAR(actual.t, expected.t, tolerance);
  EXPECT_TRUE(actual.R.isApprox(expected.R, tolerance));
  EXPECT_TRUE(actual.p.isApprox(expected.p, tolerance));
  EXPECT_TRUE(actual.v.isApprox(expected.v, tolerance));
  EXPECT_TRUE(actual.bg.isApprox(expected.bg, tolerance));
  EXPECT_TRUE(actual.ba.isApprox(expected.ba, tolerance));
  EXPECT_TRUE(actual.g.isApprox(expected.g, tolerance));
  EXPECT_TRUE(actual.cov.isApprox(expected.cov, tolerance));
}

TEST(ImuRatePredictor, KeepsGravityBalancedStationaryStateAndCovariance) {
  ImuRatePredictor predictor(0.5);
  const IMUST posterior = makePosterior(1.0);
  predictor.updateSnapshot(posterior, 2.0);

  const ImuRatePrediction prediction =
      predictor.pushImu(gravityBalancedSample(1.1));

  EXPECT_TRUE(prediction.publishable);
  EXPECT_EQ(prediction.reason, ImuRateRejectReason::NONE);
  IMUST expected = posterior;
  expected.t = 1.1;
  expectStateNear(prediction.state, expected);
  EXPECT_TRUE(prediction.unbiased_angular_velocity.isZero(kTolerance));
}

TEST(ImuRatePredictor, PropagatesConstantWorldVelocity) {
  ImuRatePredictor predictor;
  IMUST posterior = makePosterior(2.0);
  posterior.p = Eigen::Vector3d(1.0, -2.0, 3.0);
  posterior.v = Eigen::Vector3d(4.0, 0.5, -1.0);
  predictor.updateSnapshot(posterior, 2.0);

  const ImuRatePrediction prediction =
      predictor.pushImu(gravityBalancedSample(2.25));

  ASSERT_TRUE(prediction.publishable);
  EXPECT_TRUE(prediction.state.p.isApprox(
      posterior.p + posterior.v * 0.25, kTolerance));
  EXPECT_TRUE(prediction.state.v.isApprox(posterior.v, kTolerance));
}

TEST(ImuRatePredictor, IntegratesConstantYawRate) {
  ImuRatePredictor predictor(2.0);
  IMUST posterior = makePosterior();
  posterior.bg = Eigen::Vector3d(0.0, 0.0, 0.2);
  predictor.updateSnapshot(posterior, 2.0);
  ImuRateSample sample = gravityBalancedSample(0.5);
  sample.gyro = Eigen::Vector3d(0.0, 0.0, 1.2);

  const ImuRatePrediction prediction = predictor.pushImu(sample);

  ASSERT_TRUE(prediction.publishable);
  const Eigen::Matrix3d expected =
      Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  EXPECT_TRUE(prediction.state.R.isApprox(expected, kTolerance));
  EXPECT_TRUE(prediction.unbiased_angular_velocity.isApprox(
      Eigen::Vector3d(0.0, 0.0, 1.0), kTolerance));
}

TEST(ImuRatePredictor, ReplacesSnapshotAndReplaysOnlyItsNewerTailOnce) {
  ImuRatePredictor predictor;
  IMUST initial = makePosterior();
  initial.v = Eigen::Vector3d::UnitX();
  predictor.updateSnapshot(initial, 2.0);
  ASSERT_TRUE(predictor.pushImu(gravityBalancedSample(0.1)).publishable);
  ASSERT_TRUE(predictor.pushImu(gravityBalancedSample(0.2)).publishable);

  IMUST replacement = makePosterior(0.1);
  replacement.p = Eigen::Vector3d(10.0, 0.0, 0.0);
  replacement.v = Eigen::Vector3d(2.0, 0.0, 0.0);
  replacement.cov *= 3.0;
  predictor.updateSnapshot(replacement, 2.0);

  const ImuRatePrediction prediction =
      predictor.pushImu(gravityBalancedSample(0.3));

  ASSERT_TRUE(prediction.publishable);
  EXPECT_NEAR(prediction.state.p.x(), 10.4, kTolerance);
  EXPECT_NEAR(prediction.state.v.x(), 2.0, kTolerance);
  EXPECT_TRUE(prediction.state.cov.isApprox(replacement.cov, kTolerance));
}

TEST(ImuRatePredictor, ResetRequiresANewSnapshot) {
  ImuRatePredictor predictor;
  predictor.updateSnapshot(makePosterior(), 2.0);
  ASSERT_TRUE(predictor.pushImu(gravityBalancedSample(0.1)).publishable);

  predictor.reset();
  const ImuRatePrediction prediction =
      predictor.pushImu(gravityBalancedSample(0.2));

  EXPECT_FALSE(prediction.publishable);
  EXPECT_EQ(prediction.reason, ImuRateRejectReason::NO_SNAPSHOT);
}

TEST(ImuRatePredictor, RejectsNonMonotonicTimeWithoutCorruptingState) {
  ImuRatePredictor predictor;
  predictor.updateSnapshot(makePosterior(), 2.0);
  const ImuRatePrediction first =
      predictor.pushImu(gravityBalancedSample(0.1));
  ASSERT_TRUE(first.publishable);

  ImuRateSample duplicate = gravityBalancedSample(0.1);
  duplicate.accel.x() = 1000.0;
  const ImuRatePrediction rejected = predictor.pushImu(duplicate);
  EXPECT_FALSE(rejected.publishable);
  EXPECT_EQ(rejected.reason, ImuRateRejectReason::NON_MONOTONIC_TIME);
  expectStateNear(rejected.state, first.state);

  const ImuRatePrediction next =
      predictor.pushImu(gravityBalancedSample(0.2));
  ASSERT_TRUE(next.publishable);
  EXPECT_TRUE(next.state.p.isZero(kTolerance));
  EXPECT_TRUE(next.state.v.isZero(kTolerance));
}

TEST(ImuRatePredictor, RejectsNonFiniteInputWithoutCorruptingState) {
  ImuRatePredictor predictor;
  predictor.updateSnapshot(makePosterior(), 2.0);
  const ImuRatePrediction first =
      predictor.pushImu(gravityBalancedSample(0.1));
  ASSERT_TRUE(first.publishable);

  ImuRateSample invalid = gravityBalancedSample(0.2);
  invalid.gyro.x() = std::numeric_limits<double>::quiet_NaN();
  const ImuRatePrediction rejected = predictor.pushImu(invalid);
  EXPECT_FALSE(rejected.publishable);
  EXPECT_EQ(rejected.reason, ImuRateRejectReason::NONFINITE);
  expectStateNear(rejected.state, first.state);

  const ImuRatePrediction next =
      predictor.pushImu(gravityBalancedSample(0.2));
  ASSERT_TRUE(next.publishable);
  EXPECT_TRUE(next.state.p.isZero(kTolerance));
  EXPECT_TRUE(next.state.v.isZero(kTolerance));
}

TEST(ImuRatePredictor, RetainsHorizonExceededSampleForPosteriorReplay) {
  ImuRatePredictor predictor(0.15);
  IMUST initial = makePosterior();
  initial.v = Eigen::Vector3d::UnitX();
  predictor.updateSnapshot(initial, 2.0);
  ASSERT_TRUE(predictor.pushImu(gravityBalancedSample(0.1)).publishable);

  const ImuRatePrediction stale =
      predictor.pushImu(gravityBalancedSample(0.2));
  EXPECT_FALSE(stale.publishable);
  EXPECT_EQ(stale.reason,
            ImuRateRejectReason::PREDICTION_HORIZON_EXCEEDED);

  IMUST replacement = makePosterior(0.15);
  replacement.p = Eigen::Vector3d(5.0, 0.0, 0.0);
  replacement.v = Eigen::Vector3d::UnitX();
  predictor.updateSnapshot(replacement, 2.0);
  const ImuRatePrediction recovered =
      predictor.pushImu(gravityBalancedSample(0.25));

  ASSERT_TRUE(recovered.publishable);
  EXPECT_NEAR(recovered.state.p.x(), 5.1, kTolerance);
}

TEST(ImuRatePredictor, DisabledPredictorDoesNotConsumeSamples) {
  ImuRatePredictor predictor;
  predictor.updateSnapshot(makePosterior(), 2.0);
  predictor.setEnabled(false);

  const ImuRatePrediction disabled =
      predictor.pushImu(gravityBalancedSample(0.1));
  EXPECT_FALSE(disabled.publishable);
  EXPECT_EQ(disabled.reason, ImuRateRejectReason::DISABLED);

  predictor.setEnabled(true);
  const ImuRatePrediction enabled =
      predictor.pushImu(gravityBalancedSample(0.1));
  EXPECT_TRUE(enabled.publishable);
}

TEST(ImuRatePredictor, UpdatedMaximumPredictionHorizonTakesEffect) {
  ImuRatePredictor predictor;
  predictor.updateSnapshot(makePosterior(), 2.0);
  predictor.setMaxPredictionSec(0.05);

  const ImuRatePrediction prediction =
      predictor.pushImu(gravityBalancedSample(0.1));

  EXPECT_FALSE(prediction.publishable);
  EXPECT_EQ(prediction.reason,
            ImuRateRejectReason::PREDICTION_HORIZON_EXCEEDED);
}

TEST(ImuRatePredictor, IgnoresInvalidMaximumPredictionUpdates) {
  ImuRatePredictor predictor(0.5);
  IMUST posterior = makePosterior();
  posterior.v = Eigen::Vector3d::UnitX();
  predictor.updateSnapshot(posterior, 2.0);
  ASSERT_TRUE(predictor.pushImu(gravityBalancedSample(0.1)).publishable);

  predictor.setMaxPredictionSec(
      std::numeric_limits<double>::quiet_NaN());
  const ImuRatePrediction after_nan =
      predictor.pushImu(gravityBalancedSample(0.2));
  ASSERT_TRUE(after_nan.publishable);
  EXPECT_NEAR(after_nan.state.p.x(), 0.2, kTolerance);

  predictor.setMaxPredictionSec(0.0);
  const ImuRatePrediction after_zero =
      predictor.pushImu(gravityBalancedSample(0.3));
  ASSERT_TRUE(after_zero.publishable);
  EXPECT_NEAR(after_zero.state.p.x(), 0.3, kTolerance);

  predictor.setMaxPredictionSec(-1.0);
  const ImuRatePrediction after_negative =
      predictor.pushImu(gravityBalancedSample(0.4));
  ASSERT_TRUE(after_negative.publishable);
  EXPECT_NEAR(after_negative.state.p.x(), 0.4, kTolerance);
}

TEST(ImuRatePredictor, InvalidConstructorHorizonFallsBackToDefault) {
  const double invalid_values[] = {
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(), 0.0, -1.0};

  for (double invalid_value : invalid_values) {
    SCOPED_TRACE(invalid_value);
    ImuRatePredictor predictor(invalid_value);
    predictor.updateSnapshot(makePosterior(), 2.0);

    const ImuRatePrediction at_default_boundary =
        predictor.pushImu(gravityBalancedSample(0.5));
    const ImuRatePrediction beyond_default =
        predictor.pushImu(gravityBalancedSample(0.6));

    EXPECT_TRUE(at_default_boundary.publishable);
    EXPECT_EQ(at_default_boundary.reason, ImuRateRejectReason::NONE);
    EXPECT_FALSE(beyond_default.publishable);
    EXPECT_EQ(beyond_default.reason,
              ImuRateRejectReason::PREDICTION_HORIZON_EXCEEDED);
  }
}

TEST(ImuRatePredictor, IgnoresInvalidSnapshotUpdates) {
  ImuRatePredictor predictor(0.5);
  IMUST posterior = makePosterior();
  predictor.updateSnapshot(posterior, 2.0);
  ImuRateSample accelerated = gravityBalancedSample(0.1);
  accelerated.accel.x() = 1.0;
  ASSERT_TRUE(predictor.pushImu(accelerated).publishable);

  IMUST nonfinite_posterior = makePosterior(0.1);
  nonfinite_posterior.p.x() =
      std::numeric_limits<double>::infinity();
  predictor.updateSnapshot(nonfinite_posterior, 2.0);
  const ImuRatePrediction after_bad_posterior =
      predictor.pushImu(gravityBalancedSample(0.2));
  ASSERT_TRUE(after_bad_posterior.publishable);
  EXPECT_NEAR(after_bad_posterior.state.p.x(), 0.03, kTolerance);
  EXPECT_NEAR(after_bad_posterior.state.v.x(), 0.2, kTolerance);

  IMUST replacement = makePosterior(0.2);
  replacement.p.x() = 100.0;
  predictor.updateSnapshot(
      replacement, std::numeric_limits<double>::quiet_NaN());
  const ImuRatePrediction after_bad_scale =
      predictor.pushImu(gravityBalancedSample(0.3));
  ASSERT_TRUE(after_bad_scale.publishable);
  EXPECT_NEAR(after_bad_scale.state.p.x(), 0.05, kTolerance);
  EXPECT_NEAR(after_bad_scale.state.v.x(), 0.2, kTolerance);

  predictor.updateSnapshot(makePosterior(), 2.0);
  const ImuRatePrediction after_replay =
      predictor.pushImu(gravityBalancedSample(0.4));
  ASSERT_TRUE(after_replay.publishable);
  EXPECT_NEAR(after_replay.state.p.x(), 0.07, kTolerance);
  EXPECT_NEAR(after_replay.state.v.x(), 0.2, kTolerance);
}

TEST(ImuRatePredictor, BlocksReplayAcrossEvictedTailUntilCoverageRecovers) {
  constexpr int kRetainedSampleLimit = 4096;
  constexpr double kDt = 0.001;
  ImuRatePredictor predictor(10.0);
  IMUST initial = makePosterior();
  initial.v = Eigen::Vector3d::UnitX();
  predictor.updateSnapshot(initial, 2.0);

  for (int i = 1; i <= kRetainedSampleLimit + 1; ++i) {
    const ImuRatePrediction prediction =
        predictor.pushImu(gravityBalancedSample(i * kDt));
    ASSERT_TRUE(prediction.publishable) << "sample " << i;
  }

  IMUST too_old = makePosterior(0.5 * kDt);
  too_old.v = Eigen::Vector3d::UnitX();
  predictor.updateSnapshot(too_old, 2.0);
  const ImuRatePrediction blocked = predictor.pushImu(
      gravityBalancedSample((kRetainedSampleLimit + 2) * kDt));
  EXPECT_FALSE(blocked.publishable);
  EXPECT_EQ(blocked.reason, ImuRateRejectReason::NO_SNAPSHOT);

  IMUST covered = makePosterior(2.0 * kDt);
  covered.p.x() = 2.0 * kDt;
  covered.v = Eigen::Vector3d::UnitX();
  predictor.updateSnapshot(covered, 2.0);
  const ImuRatePrediction recovered = predictor.pushImu(
      gravityBalancedSample((kRetainedSampleLimit + 3) * kDt));

  ASSERT_TRUE(recovered.publishable);
  EXPECT_EQ(recovered.reason, ImuRateRejectReason::NONE);
  EXPECT_NEAR(recovered.state.p.x(),
              (kRetainedSampleLimit + 3) * kDt, kTolerance);
}

TEST(ImuRatePredictor, KeepsReplayBoundaryAfterSuccessfulRecovery) {
  constexpr int kRetainedSampleLimit = 4096;
  constexpr double kDt = 0.001;
  ImuRatePredictor predictor(10.0);
  IMUST initial = makePosterior();
  initial.v = Eigen::Vector3d::UnitX();
  predictor.updateSnapshot(initial, 2.0);

  for (int i = 1; i <= kRetainedSampleLimit + 1; ++i) {
    const ImuRatePrediction prediction =
        predictor.pushImu(gravityBalancedSample(i * kDt));
    ASSERT_TRUE(prediction.publishable) << "sample " << i;
  }

  IMUST first_boundary = makePosterior(kDt);
  first_boundary.p.x() = kDt;
  first_boundary.v = Eigen::Vector3d::UnitX();
  predictor.updateSnapshot(first_boundary, 2.0);

  IMUST too_old = makePosterior(0.5 * kDt);
  too_old.p.x() = 0.5 * kDt;
  too_old.v = Eigen::Vector3d::UnitX();
  predictor.updateSnapshot(too_old, 2.0);
  const ImuRatePrediction blocked = predictor.pushImu(
      gravityBalancedSample((kRetainedSampleLimit + 2) * kDt));
  EXPECT_FALSE(blocked.publishable);
  EXPECT_EQ(blocked.reason, ImuRateRejectReason::NO_SNAPSHOT);

  IMUST current_boundary = makePosterior(2.0 * kDt);
  current_boundary.p.x() = 2.0 * kDt;
  current_boundary.v = Eigen::Vector3d::UnitX();
  predictor.updateSnapshot(current_boundary, 2.0);
  const ImuRatePrediction recovered = predictor.pushImu(
      gravityBalancedSample((kRetainedSampleLimit + 3) * kDt));

  ASSERT_TRUE(recovered.publishable);
  EXPECT_EQ(recovered.reason, ImuRateRejectReason::NONE);
  EXPECT_NEAR(recovered.state.p.x(),
              (kRetainedSampleLimit + 3) * kDt, kTolerance);
}

}  // namespace
