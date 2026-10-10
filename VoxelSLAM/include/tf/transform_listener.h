#pragma once

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/time.hpp>

#include <tf/transform_broadcaster.h>

#include <stdexcept>

namespace tf {
class TransformException : public std::runtime_error {
public:
  explicit TransformException(const std::string &message) : std::runtime_error(message) {}
};

class TransformListener {
public:
  TransformListener()
      : buffer_(std::make_shared<tf2_ros::Buffer>(ros::node()->get_clock())),
        listener_(std::make_shared<tf2_ros::TransformListener>(*buffer_)) {}

  bool waitForTransform(const std::string &target, const std::string &source,
                        const ros::Time &, const ros::Duration &timeout,
                        const ros::Duration &, std::string *error) {
    try {
      return buffer_->canTransform(target, source, tf2::TimePointZero,
                                   tf2::durationFromSec(timeout.toSec()));
    } catch (const std::exception &exception) {
      if (error) *error = exception.what();
      return false;
    }
  }

  void lookupTransform(const std::string &target, const std::string &source,
                       const ros::Time &, StampedTransform &output) {
    try {
      const auto transform = buffer_->lookupTransform(target, source, tf2::TimePointZero);
      tf2::Quaternion rotation(transform.transform.rotation.x,
                               transform.transform.rotation.y,
                               transform.transform.rotation.z,
                               transform.transform.rotation.w);
      tf2::Vector3 translation(transform.transform.translation.x,
                               transform.transform.translation.y,
                               transform.transform.translation.z);
      tf2::Transform value(rotation, translation);
      output = StampedTransform(value, ros::Time(rclcpp::Time(transform.header.stamp)),
                                transform.header.frame_id, transform.child_frame_id);
    } catch (const std::exception &exception) {
      throw TransformException(exception.what());
    }
  }

private:
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
};
}  // namespace tf
