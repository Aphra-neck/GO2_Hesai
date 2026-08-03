# go2_imu_bridge

ROS 2 Humble bridge from the Unitree SDK2 `rt/lowstate` DDS topic to
`sensor_msgs/msg/Imu`.

## Prerequisites

- ROS 2 Humble
- Unitree SDK2 installed as a discoverable CMake package

## Build

```bash
source /opt/ros/humble/setup.bash
cd ~/go2_hesai_ws
colcon build --symlink-install --packages-select go2_imu_bridge
source install/setup.bash
```

## Run

ROS 2 must use a different domain from the Unitree SDK2 CycloneDDS domain 0.
Use the same ROS domain in every terminal that needs to communicate with the
bridge.

```bash
ROS_DOMAIN_ID=30 ros2 run go2_imu_bridge go2_imu_bridge_node --ros-args \
  -p net:=enP8p1s0 \
  -p publish_rate:=200.0 \
  -p frame_id:=go2_imu \
  -p imu_topic:=/imu/data
```

Parameters:

| Name | Default | Description |
| --- | --- | --- |
| `net` | `enP8p1s0` | Interface used by Unitree SDK2 to receive `rt/lowstate`. |
| `publish_rate` | `200.0` | Maximum IMU publish rate in hertz. |
| `frame_id` | `go2_imu` | ROS frame assigned to each IMU message. |
| `imu_topic` | `/imu/data` | ROS 2 topic used for IMU output. |

The publisher offers reliable, volatile QoS with a history depth of 200.
The absolute orientation is copied into the message, but
`orientation_covariance[0]` is set to `-1.0` so downstream estimators can
ignore it and estimate orientation from angular velocity and acceleration.
