# go2_imu_bridge

ROS 2 Humble bridge from the Unitree SDK2 `rt/lowstate` DDS topic to
`sensor_msgs/msg/Imu`.

## Prerequisites

- ROS 2 Humble
- ROS 2 Fast DDS RMW (`ros-humble-rmw-fastrtps-cpp`)
- Unitree SDK2 installed as a discoverable CMake package

## Build

```bash
cd ~/catkin_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select go2_imu_bridge
source install/setup.bash
```

## Run

Unitree SDK2 bundles CycloneDDS. ROS 2 must use Fast DDS in this process and a
different domain from the Unitree SDK2 domain 0. Use the same ROS settings in
every terminal that needs to communicate with the bridge.

```bash
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH}"
RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=30 \
ros2 run go2_imu_bridge go2_imu_bridge_node --ros-args \
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
