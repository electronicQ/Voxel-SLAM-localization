#include "vehicle_odom_transform.hpp"

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace {

bool finiteVector(const Eigen::Vector3d &value) {
  return value.array().isFinite().all();
}

bool finiteMatrix(const Eigen::Matrix3d &value) {
  return value.array().isFinite().all();
}

bool sameFrame(std::string left, std::string right) {
  while (!left.empty() && left.front() == '/') left.erase(left.begin());
  while (!right.empty() && right.front() == '/') right.erase(right.begin());
  return !left.empty() && left == right;
}

void markCovarianceUnavailable(std::array<double, 36> &covariance) {
  std::fill(covariance.begin(), covariance.end(), 0.0);
  covariance[0] = -1.0;
}

}  // namespace

class ScanPlannerOdomAdapter {
 public:
  ScanPlannerOdomAdapter(ros::NodeHandle &node,
                         ros::NodeHandle &private_node)
      : node_(node), private_node_(private_node) {
    private_node_.param<std::string>("imu_odom_topic", imu_odom_topic_,
                                     "/LIO/odom_imu");
    private_node_.param<std::string>("vehicle_odom_topic",
                                     vehicle_odom_topic_,
                                     "/LIO/odom_vehicle");
    private_node_.param<std::string>("imu_frame", imu_frame_,
                                     "aft_mapped");
    private_node_.param<std::string>("base_frame", base_frame_,
                                     "base_link");
    private_node_.param<std::string>("localization_valid_topic",
                                     localization_valid_topic_,
                                     "/localization/valid");
    private_node_.param<bool>("require_localization_valid",
                              require_localization_valid_, true);
    private_node_.param<double>("tf_timeout_sec", tf_timeout_sec_, 0.2);
    private_node_.param<double>("input_timeout_sec", input_timeout_sec_,
                                0.2);

    if (imu_odom_topic_.empty() || vehicle_odom_topic_.empty() ||
        imu_frame_.empty() || base_frame_.empty() ||
        localization_valid_topic_.empty() ||
        !std::isfinite(tf_timeout_sec_) || tf_timeout_sec_ < 0.0 ||
        !std::isfinite(input_timeout_sec_) || input_timeout_sec_ < 0.0) {
      throw std::invalid_argument(
          "invalid scan_planner_odom_adapter parameters");
    }

    vehicle_odom_publisher_ =
        node_.advertise<nav_msgs::Odometry>(vehicle_odom_topic_, 1000);
    if (require_localization_valid_) {
      localization_valid_subscriber_ = node_.subscribe<std_msgs::msg::Bool>(
          localization_valid_topic_, 10,
          &ScanPlannerOdomAdapter::localizationValidCallback, this);
    } else {
      localization_valid_ = true;
    }
    imu_odom_subscriber_ = node_.subscribe<nav_msgs::msg::Odometry>(
        imu_odom_topic_, 1000,
        &ScanPlannerOdomAdapter::imuOdomCallback, this);
  }

 private:
  void localizationValidCallback(const std_msgs::msg::Bool::ConstSharedPtr &message) {
    if (message) localization_valid_ = message->data;
  }

  bool loadStaticExtrinsic() {
    if (extrinsic_ready_) return true;
    try {
      std::string error;
      if (!tf_listener_.waitForTransform(
              imu_frame_, base_frame_, ros::Time(0),
              ros::Duration(tf_timeout_sec_), ros::Duration(0.01), &error)) {
        ROS_WARN_STREAM_THROTTLE(
            2.0, "Waiting for static TF " << imu_frame_ << " <- "
                                           << base_frame_ << ": " << error);
        return false;
      }

      tf::StampedTransform transform;
      tf_listener_.lookupTransform(imu_frame_, base_frame_, ros::Time(0),
                                   transform);
      const tf::Vector3 &translation = transform.getOrigin();
      const tf::Quaternion &rotation = transform.getRotation();
      Eigen::Quaterniond quaternion(rotation.w(), rotation.x(), rotation.y(),
                                    rotation.z());
      const Eigen::Vector3d offset(translation.x(), translation.y(),
                                   translation.z());
      if (!finiteVector(offset) || !quaternion.coeffs().allFinite() ||
          quaternion.norm() < 1e-12) {
        ROS_WARN_THROTTLE(2.0, "Static IMU-to-base TF is non-finite.");
        return false;
      }
      quaternion.normalize();
      T_imu_base_.setIdentity();
      T_imu_base_.linear() = quaternion.toRotationMatrix();
      T_imu_base_.translation() = offset;
      extrinsic_ready_ = finiteMatrix(T_imu_base_.linear());
      return extrinsic_ready_;
    } catch (const tf::TransformException &exception) {
      ROS_WARN_STREAM_THROTTLE(
          2.0, "Cannot read static TF " << imu_frame_ << " <- "
                                         << base_frame_ << ": "
                                         << exception.what());
      return false;
    }
  }

  bool convertInput(const nav_msgs::Odometry &message,
                    VehicleOdomState &state) const {
    const geometry_msgs::msg::Point &position = message.pose.pose.position;
    const geometry_msgs::msg::Quaternion &orientation =
        message.pose.pose.orientation;
    Eigen::Quaterniond quaternion(orientation.w, orientation.x,
                                  orientation.y, orientation.z);
    state.t = rclcpp::Time(message.header.stamp).seconds();
    state.p = Eigen::Vector3d(position.x, position.y, position.z);
    state.v_world = Eigen::Vector3d(message.twist.twist.linear.x,
                                    message.twist.twist.linear.y,
                                    message.twist.twist.linear.z);
    state.omega_body = Eigen::Vector3d(message.twist.twist.angular.x,
                                       message.twist.twist.angular.y,
                                       message.twist.twist.angular.z);
    if (!std::isfinite(state.t) || message.header.frame_id.empty() ||
        !finiteVector(state.p) || !finiteVector(state.v_world) ||
        !finiteVector(state.omega_body) ||
        !quaternion.coeffs().allFinite() || quaternion.norm() < 1e-12) {
      return false;
    }
    quaternion.normalize();
    state.R = quaternion.toRotationMatrix();
    return finiteMatrix(state.R);
  }

  void imuOdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &message) {
    if (!message) return;
    if (!sameFrame(message->child_frame_id, imu_frame_)) {
      ROS_WARN_STREAM_THROTTLE(
          2.0, "Dropping odometry with child frame '"
                   << message->child_frame_id << "'; expected '"
                   << imu_frame_ << "'.");
      return;
    }
    if (require_localization_valid_ && !localization_valid_) {
      ROS_WARN_THROTTLE(
          2.0, "Vehicle odometry is gated until localization is valid.");
      return;
    }

    const ros::Time now = ros::Time::now();
    if (!now.isZero() && !ros::Time(rclcpp::Time(message->header.stamp)).isZero()) {
      const double age = (now - message->header.stamp).toSec();
      if (std::isfinite(age) && age > input_timeout_sec_) {
        ROS_WARN_STREAM_THROTTLE(
            2.0, "Dropping stale IMU odometry, age=" << age << " s");
        return;
      }
    }

    VehicleOdomState imu;
    if (!convertInput(*message, imu)) {
      ROS_WARN_THROTTLE(2.0, "Dropping non-finite IMU odometry.");
      return;
    }
    if (!loadStaticExtrinsic()) return;

    const VehicleOdomState base = composeVehicleOdom(imu, T_imu_base_);
    if (!std::isfinite(base.t) || !finiteMatrix(base.R) ||
        !finiteVector(base.p) || !finiteVector(base.v_world) ||
        !finiteVector(base.omega_body)) {
      ROS_WARN_THROTTLE(2.0, "Composed vehicle odometry is non-finite.");
      return;
    }

    nav_msgs::Odometry output;
    output.header = message->header;
    output.child_frame_id = base_frame_;
    output.pose.pose.position.x = base.p.x();
    output.pose.pose.position.y = base.p.y();
    output.pose.pose.position.z = base.p.z();
    Eigen::Quaterniond orientation(base.R);
    orientation.normalize();
    output.pose.pose.orientation.x = orientation.x();
    output.pose.pose.orientation.y = orientation.y();
    output.pose.pose.orientation.z = orientation.z();
    output.pose.pose.orientation.w = orientation.w();
    output.twist.twist.linear.x = base.v_world.x();
    output.twist.twist.linear.y = base.v_world.y();
    output.twist.twist.linear.z = base.v_world.z();
    output.twist.twist.angular.x = base.omega_body.x();
    output.twist.twist.angular.y = base.omega_body.y();
    output.twist.twist.angular.z = base.omega_body.z();

    // The input pose covariance mixes world translation with local rotation,
    // and the lever arm introduces additional cross terms.  Mark both output
    // covariances unavailable instead of silently copying incorrect frames.
    markCovarianceUnavailable(output.pose.covariance);
    markCovarianceUnavailable(output.twist.covariance);
    vehicle_odom_publisher_.publish(output);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Subscriber imu_odom_subscriber_;
  ros::Subscriber localization_valid_subscriber_;
  ros::Publisher vehicle_odom_publisher_;
  tf::TransformListener tf_listener_;

  std::string imu_odom_topic_ = "/LIO/odom_imu";
  std::string vehicle_odom_topic_ = "/LIO/odom_vehicle";
  std::string imu_frame_ = "aft_mapped";
  std::string base_frame_ = "base_link";
  std::string localization_valid_topic_ = "/localization/valid";
  bool require_localization_valid_ = true;
  bool localization_valid_ = false;
  bool extrinsic_ready_ = false;
  double tf_timeout_sec_ = 0.2;
  double input_timeout_sec_ = 0.2;
  Eigen::Isometry3d T_imu_base_ = Eigen::Isometry3d::Identity();
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "scan_planner_odom_adapter");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");
  try {
    ScanPlannerOdomAdapter adapter(node, private_node);
    ros::spin();
  } catch (const std::exception &exception) {
    ROS_FATAL_STREAM("scan_planner_odom_adapter failed: "
                     << exception.what());
    return 1;
  }
  return 0;
}
