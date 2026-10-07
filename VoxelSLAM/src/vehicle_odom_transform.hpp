#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

struct VehicleOdomState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double t = 0.0;
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d omega_body = Eigen::Vector3d::Zero();
};

// T_imu_base maps coordinates expressed in base into the IMU frame.
VehicleOdomState composeVehicleOdom(
    const VehicleOdomState &imu,
    const Eigen::Isometry3d &T_imu_base);
