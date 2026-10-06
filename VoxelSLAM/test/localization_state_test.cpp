#include "localization_state.hpp"

#include <gtest/gtest.h>

#include <string>

TEST(LocalizationStateMachine, RunsBtcAndDirectPoseSuccessPaths) {
  LocalizationStateMachine machine(5);
  std::string warning;

  EXPECT_TRUE(machine.handle(LocalizationEvent::MAP_LOADED, warning).accepted);
  EXPECT_EQ(machine.state(), LocalizationState::LOCAL_ODOMETRY);
  EXPECT_TRUE(machine.map_loaded());

  EXPECT_TRUE(machine.handle(LocalizationEvent::BTC_STARTED, warning).accepted);
  EXPECT_EQ(machine.state(), LocalizationState::RELOCALIZING);
  EXPECT_TRUE(machine.handle(LocalizationEvent::BTC_MATCHED, warning).accepted);
  EXPECT_EQ(machine.state(), LocalizationState::ALIGNING);
  EXPECT_EQ(machine.alignment_source(), AlignmentSource::BTC);
  EXPECT_TRUE(
      machine.handle(LocalizationEvent::ALIGNMENT_APPLIED, warning).accepted);
  EXPECT_EQ(machine.state(), LocalizationState::PREPARING_LOCAL_MAP);
  EXPECT_TRUE(
      machine.handle(LocalizationEvent::LOCAL_MAP_READY, warning).accepted);
  EXPECT_EQ(machine.state(), LocalizationState::IN_MAP_TRACKING);
  EXPECT_TRUE(machine.localization_valid());

  EXPECT_TRUE(
      machine.handle(LocalizationEvent::DIRECT_POSE_STARTED, warning).accepted);
  EXPECT_EQ(machine.state(), LocalizationState::ALIGNING);
  EXPECT_EQ(machine.alignment_source(), AlignmentSource::RVIZ_INITIAL_POSE);
  EXPECT_FALSE(machine.localization_valid());
  EXPECT_TRUE(
      machine.handle(LocalizationEvent::ALIGNMENT_APPLIED, warning).accepted);
  EXPECT_EQ(machine.state(), LocalizationState::PREPARING_LOCAL_MAP);
  EXPECT_TRUE(
      machine.handle(LocalizationEvent::LOCAL_MAP_READY, warning).accepted);
  EXPECT_EQ(machine.state(), LocalizationState::PREPARING_LOCAL_MAP);
  EXPECT_TRUE(
      machine.handle(LocalizationEvent::DIRECT_POSE_VALIDATED, warning).accepted);
  EXPECT_EQ(machine.state(), LocalizationState::IN_MAP_TRACKING);
  EXPECT_TRUE(machine.localization_valid());
}

TEST(LocalizationStateMachine, ReturnsToLocalOdometryAfterBtcOrDirectFailure) {
  LocalizationStateMachine machine(5);
  std::string warning;
  ASSERT_TRUE(machine.handle(LocalizationEvent::MAP_LOADED, warning).accepted);

  ASSERT_TRUE(machine.handle(LocalizationEvent::BTC_STARTED, warning).accepted);
  LocalizationTransition btc_failure =
      machine.handle(LocalizationEvent::BTC_FAILED, warning);
  EXPECT_TRUE(btc_failure.accepted);
  EXPECT_FALSE(btc_failure.cleanup_requested);
  EXPECT_EQ(machine.state(), LocalizationState::LOCAL_ODOMETRY);

  ASSERT_TRUE(
      machine.handle(LocalizationEvent::DIRECT_POSE_STARTED, warning).accepted);
  LocalizationTransition direct_failure =
      machine.handle(LocalizationEvent::DIRECT_POSE_FAILED, warning);
  EXPECT_TRUE(direct_failure.accepted);
  EXPECT_TRUE(direct_failure.cleanup_requested);
  EXPECT_EQ(machine.state(), LocalizationState::LOCAL_ODOMETRY);
  EXPECT_FALSE(machine.localization_valid());
  EXPECT_EQ(machine.cleanup_count(), 1U);
}

TEST(LocalizationStateMachine, RequiresFiveConsecutiveBadFramesAndNaNLosesNow) {
  LocalizationStateMachine machine(5);
  std::string warning;
  ASSERT_TRUE(machine.handle(LocalizationEvent::MAP_LOADED, warning).accepted);
  ASSERT_TRUE(machine.handle(LocalizationEvent::BTC_STARTED, warning).accepted);
  ASSERT_TRUE(machine.handle(LocalizationEvent::BTC_MATCHED, warning).accepted);
  ASSERT_TRUE(
      machine.handle(LocalizationEvent::ALIGNMENT_APPLIED, warning).accepted);
  ASSERT_TRUE(
      machine.handle(LocalizationEvent::LOCAL_MAP_READY, warning).accepted);

  for (int i = 0; i < 4; ++i) {
    LocalizationTransition transition =
        machine.handle(LocalizationEvent::TRACKING_BAD, warning);
    EXPECT_TRUE(transition.accepted);
    EXPECT_FALSE(transition.cleanup_requested);
    EXPECT_EQ(machine.state(), LocalizationState::IN_MAP_TRACKING);
  }
  EXPECT_EQ(machine.bad_tracking_streak(), 4);
  EXPECT_TRUE(
      machine.handle(LocalizationEvent::TRACKING_GOOD, warning).accepted);
  EXPECT_EQ(machine.bad_tracking_streak(), 0);

  for (int i = 0; i < 5; ++i) {
    machine.handle(LocalizationEvent::TRACKING_BAD, warning);
  }
  EXPECT_EQ(machine.state(), LocalizationState::LOST);
  EXPECT_FALSE(machine.localization_valid());
  EXPECT_EQ(machine.cleanup_count(), 1U);

  LocalizationTransition recovery =
      machine.handle(LocalizationEvent::RECOVERY_STARTED, warning);
  EXPECT_TRUE(recovery.accepted);
  EXPECT_EQ(machine.state(), LocalizationState::LOCAL_ODOMETRY);
  EXPECT_TRUE(machine.map_loaded());

  ASSERT_TRUE(machine.handle(LocalizationEvent::BTC_STARTED, warning).accepted);
  ASSERT_TRUE(machine.handle(LocalizationEvent::BTC_MATCHED, warning).accepted);
  ASSERT_TRUE(
      machine.handle(LocalizationEvent::ALIGNMENT_APPLIED, warning).accepted);
  ASSERT_TRUE(
      machine.handle(LocalizationEvent::LOCAL_MAP_READY, warning).accepted);
  LocalizationTransition non_finite =
      machine.handle(LocalizationEvent::TRACKING_NONFINITE, warning);
  EXPECT_TRUE(non_finite.accepted);
  EXPECT_TRUE(non_finite.cleanup_requested);
  EXPECT_EQ(machine.state(), LocalizationState::LOST);
  EXPECT_EQ(machine.cleanup_count(), 2U);
}

TEST(LocalizationStateMachine, WaitsForSpatialMapBeforeBecomingValid) {
  LocalizationStateMachine machine(5);
  std::string warning;
  ASSERT_TRUE(machine.handle(LocalizationEvent::MAP_LOADED, warning).accepted);
  ASSERT_TRUE(machine.handle(LocalizationEvent::BTC_STARTED, warning).accepted);
  ASSERT_TRUE(machine.handle(LocalizationEvent::BTC_MATCHED, warning).accepted);
  ASSERT_TRUE(
      machine.handle(LocalizationEvent::ALIGNMENT_APPLIED, warning).accepted);
  EXPECT_EQ(machine.state(), LocalizationState::PREPARING_LOCAL_MAP);
  EXPECT_FALSE(machine.localization_valid());
  ASSERT_TRUE(
      machine.handle(LocalizationEvent::LOCAL_MAP_READY, warning).accepted);
  EXPECT_EQ(machine.state(), LocalizationState::IN_MAP_TRACKING);
  EXPECT_TRUE(machine.localization_valid());
}

TEST(LocalizationStateMachine, LocalMapFailureCleansUpAndReturnsToOdometry) {
  LocalizationStateMachine machine(5);
  std::string warning;
  ASSERT_TRUE(machine.handle(LocalizationEvent::MAP_LOADED, warning).accepted);
  ASSERT_TRUE(machine.handle(LocalizationEvent::BTC_STARTED, warning).accepted);
  ASSERT_TRUE(machine.handle(LocalizationEvent::BTC_MATCHED, warning).accepted);
  ASSERT_TRUE(
      machine.handle(LocalizationEvent::ALIGNMENT_APPLIED, warning).accepted);
  const LocalizationTransition failed =
      machine.handle(LocalizationEvent::LOCAL_MAP_FAILED, warning);
  EXPECT_TRUE(failed.accepted);
  EXPECT_TRUE(failed.cleanup_requested);
  EXPECT_EQ(machine.state(), LocalizationState::LOCAL_ODOMETRY);
}

TEST(LocalizationStateMachine, MapGapTimeoutEntersLostImmediately) {
  LocalizationStateMachine machine(5);
  std::string warning;
  ASSERT_TRUE(machine.handle(LocalizationEvent::MAP_LOADED, warning).accepted);
  ASSERT_TRUE(machine.handle(LocalizationEvent::BTC_STARTED, warning).accepted);
  ASSERT_TRUE(machine.handle(LocalizationEvent::BTC_MATCHED, warning).accepted);
  ASSERT_TRUE(
      machine.handle(LocalizationEvent::ALIGNMENT_APPLIED, warning).accepted);
  ASSERT_TRUE(
      machine.handle(LocalizationEvent::LOCAL_MAP_READY, warning).accepted);
  const LocalizationTransition transition =
      machine.handle(LocalizationEvent::MAP_GAP_TIMEOUT, warning);
  EXPECT_EQ(machine.state(), LocalizationState::LOST);
  EXPECT_TRUE(transition.cleanup_requested);
}

TEST(LocalizationStateMachine, RejectsIllegalEventsWithoutChangingState) {
  LocalizationStateMachine machine(5);
  std::string warning;
  LocalizationTransition transition =
      machine.handle(LocalizationEvent::TRACKING_GOOD, warning);
  EXPECT_FALSE(transition.accepted);
  EXPECT_NE(warning.find("illegal"), std::string::npos) << warning;
  EXPECT_EQ(machine.state(), LocalizationState::WAITING_FOR_MAP);
  EXPECT_EQ(machine.cleanup_count(), 0U);
}

TEST(FrontendSessionEpoch, RejectsResultsCapturedBeforeReset) {
  FrontendSessionEpoch epoch;
  std::string error;

  const std::uint64_t first_session = epoch.capture();
  EXPECT_TRUE(epoch.is_current(first_session));
  ASSERT_TRUE(epoch.advance(error)) << error;
  EXPECT_FALSE(epoch.is_current(first_session));

  const std::uint64_t second_session = epoch.capture();
  EXPECT_NE(second_session, first_session);
  EXPECT_TRUE(epoch.is_current(second_session));
}
