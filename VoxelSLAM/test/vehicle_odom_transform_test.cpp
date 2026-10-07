#include "vehicle_odom_transform.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

namespace {

constexpr double kTolerance = 1e-12;

VehicleOdomState makeImuState() {
  VehicleOdomState state;
  state.t = 12.5;
  state.R = Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ())
                .toRotationMatrix();
  state.p = Eigen::Vector3d(1.0, -2.0, 0.5);
  state.v_world = Eigen::Vector3d(0.5, -0.25, 0.1);
  state.omega_body = Eigen::Vector3d(0.1, 0.2, -0.3);
  return state;
}

TEST(VehicleOdomTransform, IdentityExtrinsicPreservesState) {
  const VehicleOdomState imu = makeImuState();

  const VehicleOdomState base =
      composeVehicleOdom(imu, Eigen::Isometry3d::Identity());

  EXPECT_DOUBLE_EQ(base.t, imu.t);
  EXPECT_TRUE(base.R.isApprox(imu.R, kTolerance));
  EXPECT_TRUE(base.p.isApprox(imu.p, kTolerance));
  EXPECT_TRUE(base.v_world.isApprox(imu.v_world, kTolerance));
  EXPECT_TRUE(base.omega_body.isApprox(imu.omega_body, kTolerance));
}

TEST(VehicleOdomTransform, TranslatedBaseUsesWorldRotatedLeverArm) {
  VehicleOdomState imu = makeImuState();
  imu.omega_body.setZero();
  Eigen::Isometry3d T_imu_base = Eigen::Isometry3d::Identity();
  T_imu_base.translation() = Eigen::Vector3d(0.8, -0.2, 0.4);

  const VehicleOdomState base = composeVehicleOdom(imu, T_imu_base);

  EXPECT_TRUE(base.R.isApprox(imu.R, kTolerance));
  EXPECT_TRUE(base.p.isApprox(
      imu.p + imu.R * T_imu_base.translation(), kTolerance));
  EXPECT_TRUE(base.v_world.isApprox(imu.v_world, kTolerance));
}

TEST(VehicleOdomTransform, RotatedExtrinsicComposesPoseAndBodyAngularRate) {
  VehicleOdomState imu = makeImuState();
  Eigen::Isometry3d T_imu_base = Eigen::Isometry3d::Identity();
  T_imu_base.linear() =
      Eigen::AngleAxisd(-0.6, Eigen::Vector3d::UnitY())
          .toRotationMatrix();

  const VehicleOdomState base = composeVehicleOdom(imu, T_imu_base);

  EXPECT_TRUE(base.R.isApprox(
      imu.R * T_imu_base.linear(), kTolerance));
  EXPECT_TRUE(base.omega_body.isApprox(
      T_imu_base.linear().transpose() * imu.omega_body, kTolerance));
}

TEST(VehicleOdomTransform, AddsAngularLeverArmVelocityInWorldFrame) {
  VehicleOdomState imu;
  imu.R = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ())
              .toRotationMatrix();
  imu.v_world = Eigen::Vector3d(2.0, 3.0, 0.0);
  imu.omega_body = Eigen::Vector3d(0.0, 0.0, 1.5);
  Eigen::Isometry3d T_imu_base = Eigen::Isometry3d::Identity();
  T_imu_base.translation() = Eigen::Vector3d(0.6, -0.1, 0.0);

  const VehicleOdomState base = composeVehicleOdom(imu, T_imu_base);

  const Eigen::Vector3d omega_world = imu.R * imu.omega_body;
  const Eigen::Vector3d lever_world =
      imu.R * T_imu_base.translation();
  EXPECT_TRUE(base.v_world.isApprox(
      imu.v_world + omega_world.cross(lever_world), kTolerance));
}

}  // namespace
