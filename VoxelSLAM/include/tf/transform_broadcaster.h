#pragma once

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_broadcaster.h>

#include <ros/ros.h>

#include <string>

namespace tf {
using Vector3 = tf2::Vector3;
using Quaternion = tf2::Quaternion;
using Transform = tf2::Transform;

class StampedTransform : public Transform {
public:
  StampedTransform() = default;
  StampedTransform(const Transform &transform, const ros::Time &stamp,
                  const std::string &frame_id, const std::string &child_frame_id)
      : Transform(transform), stamp_(stamp.native()), frame_id_(frame_id),
        child_frame_id_(child_frame_id) {}

  const rclcpp::Time &stamp() const { return stamp_; }
  const std::string &frame_id() const { return frame_id_; }
  const std::string &child_frame_id() const { return child_frame_id_; }

private:
  rclcpp::Time stamp_{0, 0, RCL_ROS_TIME};
  std::string frame_id_;
  std::string child_frame_id_;
};

class TransformBroadcaster {
public:
  TransformBroadcaster() : broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(ros::node())) {}

  void sendTransform(const StampedTransform &transform) {
    geometry_msgs::msg::TransformStamped message;
    message.header.stamp = ros::Time(transform.stamp()).toMsg();
    message.header.frame_id = transform.frame_id();
    message.child_frame_id = transform.child_frame_id();
    const auto origin = transform.getOrigin();
    const auto rotation = transform.getRotation();
    message.transform.translation.x = origin.x();
    message.transform.translation.y = origin.y();
    message.transform.translation.z = origin.z();
    message.transform.rotation.x = rotation.x();
    message.transform.rotation.y = rotation.y();
    message.transform.rotation.z = rotation.z();
    message.transform.rotation.w = rotation.w();
    broadcaster_->sendTransform(message);
  }

private:
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};
}  // namespace tf
