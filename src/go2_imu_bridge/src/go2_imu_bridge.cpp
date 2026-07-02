#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>

#include <functional>
#include <string>

#define TOPIC_LOWSTATE "rt/lowstate"

class Go2ImuBridge {
public:
  explicit Go2ImuBridge(ros::NodeHandle& nh) {
    ros::NodeHandle pnh("~");
    pnh.param<std::string>("frame_id", frame_id_, "go2_imu");
    pnh.param<double>("publish_rate", publish_rate_, 200.0);

    imu_pub_ = nh.advertise<sensor_msgs::Imu>("/imu/data", 200);

    lowstate_sub_.reset(
      new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE)
    );

    lowstate_sub_->InitChannel(
      std::bind(&Go2ImuBridge::LowStateCallback, this, std::placeholders::_1),
      10
    );

    ROS_INFO("go2_imu_bridge started, publishing /imu/data at %.1f Hz", publish_rate_);
  }

private:
  void LowStateCallback(const void* message) {
    ros::Time now = ros::Time::now();

    if (!last_pub_time_.isZero()) {
      const double dt = (now - last_pub_time_).toSec();
      if (dt < 1.0 / publish_rate_) {
        return;
      }
      if (dt <= 0.0) {
        return;
      }
    }
    last_pub_time_ = now;

    const auto* state = static_cast<const unitree_go::msg::dds_::LowState_*>(message);
    const auto& imu = state->imu_state();

    const auto& q = imu.quaternion();      // Unitree order: w, x, y, z
    const auto& gyro = imu.gyroscope();    // rad/s
    const auto& acc = imu.accelerometer(); // m/s^2

    sensor_msgs::Imu msg;
    msg.header.stamp = now;
    msg.header.frame_id = frame_id_;

    msg.orientation.w = q[0];
    msg.orientation.x = q[1];
    msg.orientation.y = q[2];
    msg.orientation.z = q[3];

    msg.angular_velocity.x = gyro[0];
    msg.angular_velocity.y = gyro[1];
    msg.angular_velocity.z = gyro[2];

    msg.linear_acceleration.x = acc[0];
    msg.linear_acceleration.y = acc[1];
    msg.linear_acceleration.z = acc[2];

    // Tell consumers not to trust the absolute orientation from LowState.
    msg.orientation_covariance[0] = -1.0;

    msg.angular_velocity_covariance[0] = 0.01;
    msg.angular_velocity_covariance[4] = 0.01;
    msg.angular_velocity_covariance[8] = 0.01;

    msg.linear_acceleration_covariance[0] = 0.1;
    msg.linear_acceleration_covariance[4] = 0.1;
    msg.linear_acceleration_covariance[8] = 0.1;

    imu_pub_.publish(msg);
  }

  std::string frame_id_;
  double publish_rate_;
  ros::Time last_pub_time_;
  ros::Publisher imu_pub_;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_sub_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "go2_imu_bridge");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string net;
  pnh.param<std::string>("net", net, "eth10");

  unitree::robot::ChannelFactory::Instance()->Init(0, net);

  Go2ImuBridge bridge(nh);
  ros::spin();

  return 0;
}
