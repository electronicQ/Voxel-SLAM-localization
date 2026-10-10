#pragma once

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <array>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

namespace ros {

class Time {
public:
  Time() : value_(0, 0, RCL_ROS_TIME) {}
  explicit Time(uint32_t seconds) : value_(static_cast<int64_t>(seconds), 0, RCL_ROS_TIME) {}
  explicit Time(const rclcpp::Time &value) : value_(value) {}

  static Time now() { return Time(rclcpp::Clock(RCL_ROS_TIME).now()); }
  static Time fromSec(double seconds) {
    return Time(rclcpp::Time(static_cast<int64_t>(seconds * 1e9), RCL_ROS_TIME));
  }
  double toSec() const { return value_.seconds(); }
  bool isZero() const { return value_.nanoseconds() == 0; }
  const rclcpp::Time &native() const { return value_; }

  builtin_interfaces::msg::Time toMsg() const {
    builtin_interfaces::msg::Time message;
    const int64_t nanoseconds = value_.nanoseconds();
    message.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
    message.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
    return message;
  }
  operator builtin_interfaces::msg::Time() const { return toMsg(); }

private:
  rclcpp::Time value_;
};

class Duration {
public:
  explicit Duration(double seconds = 0.0) : seconds_(seconds) {}
  double toSec() const { return seconds_; }
  void sleep() const {
    if (seconds_ > 0.0) {
      std::this_thread::sleep_for(std::chrono::duration<double>(seconds_));
    }
  }

private:
  double seconds_;
};

inline Duration operator-(const Time &left, const builtin_interfaces::msg::Time &right) {
  return Duration(left.toSec() - rclcpp::Time(right).seconds());
}

class CallbackQueue {
public:
  rclcpp::CallbackGroup::SharedPtr group(const rclcpp::Node::SharedPtr &node) {
    if (!group_) {
      // The group is owned by the dedicated IMU executor.  Prevent the
      // node's default executor from auto-adding it as well, which would make
      // add_callback_group() fail at runtime in rclcpp.
      group_ = node->create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive, false);
    }
    return group_;
  }

private:
  rclcpp::CallbackGroup::SharedPtr group_;
};

class Publisher {
public:
  Publisher() = default;

  template <typename MessageT>
  void publish(const MessageT &message) const {
    if (publish_fn_) publish_fn_(&message);
  }

  void shutdown() { publish_fn_ = nullptr; }
  explicit operator bool() const { return static_cast<bool>(publish_fn_); }

  template <typename MessageT>
  static Publisher make(const typename rclcpp::Publisher<MessageT>::SharedPtr &publisher) {
    Publisher result;
    result.publish_fn_ = [publisher](const void *message) {
      publisher->publish(*static_cast<const MessageT *>(message));
    };
    return result;
  }

private:
  std::function<void(const void *)> publish_fn_;
};

class Subscriber {
public:
  Subscriber() = default;
  explicit Subscriber(const rclcpp::SubscriptionBase::SharedPtr &subscription)
      : subscription_(subscription) {}
  void shutdown() { subscription_.reset(); }
  explicit operator bool() const { return static_cast<bool>(subscription_); }

private:
  rclcpp::SubscriptionBase::SharedPtr subscription_;
};

class NodeHandle {
public:
  NodeHandle() : node_(global_node()) {}
  explicit NodeHandle(const std::string &) : node_(global_node()) {}

  template <typename T>
  void param(const std::string &name, T &value, const T &default_value) {
    const auto key = normalize(name);
    if (!node_->has_parameter(key)) node_->declare_parameter<T>(key, default_value);
    node_->get_parameter_or<T>(key, value, default_value);
  }

  template <typename MessageT>
  Publisher advertise(const std::string &topic, std::size_t depth,
                      bool latched = false) {
    auto qos = rclcpp::QoS(depth);
    if (latched) qos.transient_local().reliable();
    auto publisher = node_->create_publisher<MessageT>(topic, qos);
    return Publisher::make<MessageT>(publisher);
  }

  template <typename MessageT, typename CallbackT>
  Subscriber subscribe(const std::string &topic, std::size_t depth, CallbackT callback) {
    return subscribe<MessageT>(topic, depth, std::move(callback), nullptr);
  }

  template <typename MessageT, typename CallbackT>
  Subscriber subscribe(const std::string &topic, std::size_t depth,
                       CallbackT callback, CallbackQueue *queue) {
    rclcpp::SubscriptionOptions options;
    if (queue) options.callback_group = queue->group(node_);
    auto subscription = node_->create_subscription<MessageT>(
        topic, rclcpp::QoS(depth).best_effort(), std::move(callback), options);
    return Subscriber(subscription);
  }

  template <typename MessageT, typename ObjectT>
  Subscriber subscribe(const std::string &topic, std::size_t depth,
                       void (ObjectT::*callback)(typename MessageT::ConstSharedPtr),
                       ObjectT *object) {
    auto subscription = node_->create_subscription<MessageT>(
        topic, rclcpp::QoS(depth).best_effort(),
        [object, callback](typename MessageT::ConstSharedPtr message) {
          (object->*callback)(std::move(message));
        });
    return Subscriber(subscription);
  }

  template <typename MessageT, typename ObjectT>
  Subscriber subscribe(const std::string &topic, std::size_t depth,
                       void (ObjectT::*callback)(const typename MessageT::ConstSharedPtr &),
                       ObjectT *object) {
    auto subscription = node_->create_subscription<MessageT>(
        topic, rclcpp::QoS(depth).best_effort(),
        [object, callback](typename MessageT::ConstSharedPtr message) {
          (object->*callback)(message);
        });
    return Subscriber(subscription);
  }

  rclcpp::Node::SharedPtr node() const { return node_; }
  bool ok() const { return rclcpp::ok(); }

private:
  static std::string normalize(std::string name) {
    for (char &character : name) {
      if (character == '/') character = '.';
    }
    while (!name.empty() && name.front() == '.') name.erase(name.begin());
    return name;
  }

  static rclcpp::Node::SharedPtr &global_node() {
    static rclcpp::Node::SharedPtr node;
    if (!node) node = std::make_shared<rclcpp::Node>("cmn_voxel");
    return node;
  }

  rclcpp::Node::SharedPtr node_;
};

inline rclcpp::Node::SharedPtr node() { return NodeHandle().node(); }

class AsyncSpinner {
public:
  AsyncSpinner(int, CallbackQueue *queue) : queue_(queue) {}
  void start() {
    if (!queue_ || thread_.joinable()) return;
    node_ = ros::node();
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_callback_group(queue_->group(node_), node_->get_node_base_interface());
    thread_ = std::thread([this]() { executor_->spin(); });
  }
  void stop() {
    if (!executor_) return;
    executor_->cancel();
    if (thread_.joinable()) thread_.join();
    executor_.reset();
    node_.reset();
  }

private:
  CallbackQueue *queue_ = nullptr;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread thread_;
};

inline void init(int argc, char **argv, const std::string &) {
  rclcpp::init(argc, argv);
}

inline void spin() {
  NodeHandle node;
  rclcpp::spin(node.node());
}

inline void spinOnce() {
  NodeHandle node;
  rclcpp::spin_some(node.node());
}

inline void shutdown() { rclcpp::shutdown(); }

namespace detail {
inline void log_text(const char *level, const std::string &text) {
  auto logger = rclcpp::get_logger("voxel_slam");
  if (std::string(level) == "INFO") RCLCPP_INFO(logger, "%s", text.c_str());
  else if (std::string(level) == "WARN") RCLCPP_WARN(logger, "%s", text.c_str());
  else if (std::string(level) == "ERROR") RCLCPP_ERROR(logger, "%s", text.c_str());
  else RCLCPP_FATAL(logger, "%s", text.c_str());
}
}  // namespace detail

}  // namespace ros

#define ROS_INFO(...) RCLCPP_INFO(rclcpp::get_logger("voxel_slam"), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(rclcpp::get_logger("voxel_slam"), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("voxel_slam"), __VA_ARGS__)
#define ROS_FATAL(...) RCLCPP_FATAL(rclcpp::get_logger("voxel_slam"), __VA_ARGS__)
#define ROS_INFO_STREAM(value) do { std::ostringstream stream; stream << value; ::ros::detail::log_text("INFO", stream.str()); } while (0)
#define ROS_WARN_STREAM(value) do { std::ostringstream stream; stream << value; ::ros::detail::log_text("WARN", stream.str()); } while (0)
#define ROS_ERROR_STREAM(value) do { std::ostringstream stream; stream << value; ::ros::detail::log_text("ERROR", stream.str()); } while (0)
#define ROS_FATAL_STREAM(value) do { std::ostringstream stream; stream << value; ::ros::detail::log_text("FATAL", stream.str()); } while (0)
#define ROS_WARN_THROTTLE(period, ...) ROS_WARN(__VA_ARGS__)
#define ROS_WARN_STREAM_THROTTLE(period, value) ROS_WARN_STREAM(value)
