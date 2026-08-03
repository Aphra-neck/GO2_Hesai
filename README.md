# Go2 + Hesai XT-16 + Super-LIO (ROS 2)

This branch targets the Unitree Go2 Jetson payload running Ubuntu 22.04 ARM64
and ROS 2 Humble.

## Data flow

```text
Hesai XT-16
  -> hesai_ros_driver
  -> /lidar_points (sensor_msgs/msg/PointCloud2)

Go2 LowState
  -> Unitree SDK2 DDS rt/lowstate
  -> go2_imu_bridge
  -> /imu/data (sensor_msgs/msg/Imu)

/lidar_points + /imu/data
  -> Super-LIO ROS 2
  -> /lio/odom
  -> /lio/cloud_world
```

## Included components

- `src/go2_imu_bridge`: ROS 2 Humble `rclcpp` bridge for Go2 IMU data.
- `src/HesaiLidar_ROS_2.0`: Hesai driver with native ROS 2 support. Its source
  remains unchanged on this branch.
- `src/Super-LIO`: vendored from the upstream `ros2` branch. See
  `src/Super-LIO/UPSTREAM.md` for provenance.

## Prerequisites

```bash
source /opt/ros/humble/setup.bash

sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  libboost-all-dev \
  libyaml-cpp-dev \
  libgoogle-glog-dev \
  libtbb-dev \
  ros-humble-pcl-ros \
  ros-humble-tf2-ros
```

Unitree SDK2 must also be installed so this succeeds:

```bash
cmake --find-package \
  -DNAME=unitree_sdk2 \
  -DCOMPILER_ID=GNU \
  -DLANGUAGE=CXX \
  -DMODE=EXIST
```

This integration disables Super-LIO's optional Livox input by default, so a
Hesai-only build does not require `livox_ros_driver2`. To retain Livox support,
make that package available and build with
`--cmake-args -DSUPER_LIO_WITH_LIVOX=ON`.

## Build

From the repository root:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

LTO is disabled by default to reduce peak linker memory on Jetson. It can be
enabled on a larger machine with `--cmake-args -DSUPER_LIO_ENABLE_LTO=ON`.
If memory is still tight, prefix the build with `MAKEFLAGS=-j2`.

Build only the Go2 bridge while iterating on it:

```bash
colcon build --symlink-install --packages-select go2_imu_bridge
source install/setup.bash
```

## Run components separately

Terminal 1, Hesai XT-16:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run hesai_ros_driver hesai_ros_driver_node --ros-args \
  -p config_path:="$PWD/src/HesaiLidar_ROS_2.0/config/config.yaml"
```

Terminal 2, Go2 IMU:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run go2_imu_bridge go2_imu_bridge_node --ros-args \
  -p net:=eth10 \
  -p publish_rate:=200.0
```

Terminal 3, Super-LIO:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch super_lio hesai.py rviz:=false
```

## Run the complete chain

```bash
chmod +x shell/start_slam.sh
./shell/start_slam.sh
```

Optional environment overrides:

```bash
GO2_NETWORK_INTERFACE=eth10 GO2_IMU_RATE=200.0 RVIZ=false \
  ./shell/start_slam.sh
```

Logs are written to `~/slam_logs` by default. Set `SLAM_LOG_DIR` to change
that directory.

## Validate topics

```bash
ros2 topic hz /lidar_points
ros2 topic hz /imu/data
ros2 topic hz /lio/odom
ros2 topic list | grep '^/lio/'
```

Expected rates:

- `/lidar_points`: approximately 10 Hz
- `/imu/data`: approximately 160-200 Hz
- `/lio/odom`: approximately 10 Hz

Inspect one message from each sensor:

```bash
ros2 topic echo --once /lidar_points
ros2 topic echo --once /imu/data
```

## Go2/XT-16 settings

The Hesai driver configuration is at:

```text
src/HesaiLidar_ROS_2.0/config/config.yaml
```

Current network and timestamp settings:

```yaml
device_ip_address: 192.168.123.20
host_ip_address: 192.168.123.18
udp_port: 2368
device_udp_src_port: 10000
use_timestamp_type: 1
ros_send_point_cloud_topic: /lidar_points
```

`use_timestamp_type: 1` makes the point cloud use the Jetson receive time so
it shares the same clock as the bridge's `/imu/data` messages.

The Super-LIO ROS 2 configuration is at:

```text
src/Super-LIO/src/super_lio/config/hesai.yaml
```

The migrated Go2 values include:

```yaml
lio.ros.lidar_topic: "/lidar_points"
lio.ros.imu_topic: "/imu/data"
lio.sensor.lidar_type: 2
lio.sensor.blind: 0.5
lio.sensor.filter_rate: 1
lio.sensor.voxel_fliter_size: 0.3
lio.sensor.gravity_norm: 9.4188
lio.extrinsic.lidar_imu: [0.171, 0.0, 0.0908,
                          1.0, 0.0, 0.0,
                          0.0, 1.0, 0.0,
                          0.0, 0.0, 1.0]
```

## Troubleshooting

Confirm XT-16 UDP packets reach the Jetson:

```bash
sudo tcpdump -i eth10 -nn 'udp port 2368' -c 10
```

Confirm ROS 2 sees the expected packages:

```bash
ros2 pkg prefix hesai_ros_driver
ros2 pkg prefix go2_imu_bridge
ros2 pkg prefix super_lio
```

If topics exist but Super-LIO receives no samples, inspect QoS:

```bash
ros2 topic info -v /lidar_points
ros2 topic info -v /imu/data
```

The bridge publishes reliable/volatile IMU data. Super-LIO requests
best-effort/volatile IMU data; this offered/requested pair is compatible in
ROS 2 DDS.
