#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <net/if.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

namespace
{
constexpr char kLowStateTopic[] = "rt/lowstate";
constexpr char kDefaultImuTopic[] = "/imu/data";
constexpr char kDefaultFrameId[] = "go2_imu";
constexpr char kDefaultNetworkInterface[] = "enP8p1s0";
constexpr double kDefaultPublishRate = 200.0;
constexpr std::size_t kPublisherDepth = 200;
}  // namespace

class Go2ImuBridge final : public rclcpp::Node
{
public:
  Go2ImuBridge()
  : Node("go2_imu_bridge"),
    frame_id_(declare_parameter<std::string>("frame_id", kDefaultFrameId)),
    imu_topic_(declare_parameter<std::string>("imu_topic", kDefaultImuTopic)),
    publish_rate_(declare_parameter<double>("publish_rate", kDefaultPublishRate))
  {
    const auto network_interface =
      declare_parameter<std::string>("net", kDefaultNetworkInterface);

    if (!std::isfinite(publish_rate_) || publish_rate_ <= 0.0) {
      throw std::invalid_argument("publish_rate must be finite and greater than zero");
    }

    if (if_nametoindex(network_interface.c_str()) == 0U) {
      throw std::invalid_argument(
        "network interface does not exist: " + network_interface);
    }

    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(kPublisherDepth))
      .reliable()
      .durability_volatile();
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, qos);

    lowstate_subscriber_ = std::make_shared<
      unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>>(
      kLowStateTopic);
    lowstate_subscriber_->InitChannel(
      std::bind(&Go2ImuBridge::low_state_callback, this, std::placeholders::_1),
      10);

    RCLCPP_INFO(
      get_logger(),
      "Listening for %s on %s; publishing %s at up to %.1f Hz",
      kLowStateTopic,
      network_interface.c_str(),
      imu_topic_.c_str(),
      publish_rate_);
  }

private:
  bool should_publish(const std::chrono::steady_clock::time_point now)
  {
    std::lock_guard<std::mutex> lock(publish_mutex_);
    if (has_published_) {
      const auto elapsed = std::chrono::duration<double>(now - last_publish_time_).count();
      if (elapsed < (1.0 / publish_rate_)) {
        return false;
      }
    }

    last_publish_time_ = now;
    has_published_ = true;
    return true;
  }

  void low_state_callback(const void * message)
  {
    if (message == nullptr) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Received a null LowState message");
      return;
    }

    if (!should_publish(std::chrono::steady_clock::now())) {
      return;
    }

    const auto * state =
      static_cast<const unitree_go::msg::dds_::LowState_ *>(message);
    const auto & imu = state->imu_state();
    const auto & quaternion = imu.quaternion();  // Unitree order: w, x, y, z.
    const auto & gyroscope = imu.gyroscope();    // rad/s.
    const auto & accelerometer = imu.accelerometer();  // m/s^2.

    sensor_msgs::msg::Imu message_out;
    message_out.header.stamp = now();
    message_out.header.frame_id = frame_id_;

    message_out.orientation.w = quaternion[0];
    message_out.orientation.x = quaternion[1];
    message_out.orientation.y = quaternion[2];
    message_out.orientation.z = quaternion[3];

    message_out.angular_velocity.x = gyroscope[0];
    message_out.angular_velocity.y = gyroscope[1];
    message_out.angular_velocity.z = gyroscope[2];

    message_out.linear_acceleration.x = accelerometer[0];
    message_out.linear_acceleration.y = accelerometer[1];
    message_out.linear_acceleration.z = accelerometer[2];

    // Super-LIO should estimate orientation instead of trusting LowState's absolute attitude.
    message_out.orientation_covariance[0] = -1.0;

    message_out.angular_velocity_covariance[0] = 0.01;
    message_out.angular_velocity_covariance[4] = 0.01;
    message_out.angular_velocity_covariance[8] = 0.01;

    message_out.linear_acceleration_covariance[0] = 0.1;
    message_out.linear_acceleration_covariance[4] = 0.1;
    message_out.linear_acceleration_covariance[8] = 0.1;

    imu_publisher_->publish(message_out);
  }

  std::string frame_id_;
  std::string imu_topic_;
  double publish_rate_;
  std::mutex publish_mutex_;
  std::chrono::steady_clock::time_point last_publish_time_{};
  bool has_published_{false};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_>
    lowstate_subscriber_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<Go2ImuBridge>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("go2_imu_bridge"), "Failed to start: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
