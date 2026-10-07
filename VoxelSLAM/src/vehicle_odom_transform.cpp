#include "vehicle_odom_transform.hpp"

VehicleOdomState composeVehicleOdom(
    const VehicleOdomState &imu,
    const Eigen::Isometry3d &T_imu_base) {
  VehicleOdomState base;
  base.t = imu.t;
  base.R = imu.R * T_imu_base.linear();

  const Eigen::Vector3d lever_world =
      imu.R * T_imu_base.translation();
  base.p = imu.p + lever_world;

  const Eigen::Vector3d omega_world = imu.R * imu.omega_body;
  base.v_world = imu.v_world + omega_world.cross(lever_world);
  base.omega_body = base.R.transpose() * omega_world;
  return base;
}
