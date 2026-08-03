# GO2_Hesai ROS 2

## 项目简介

本仓库的 `ROS2` 分支用于在 Unitree Go2 的 Jetson 载荷计算机上，将 Hesai
XT-16 激光雷达和 Go2 内部 IMU 接入 ROS 2 Humble，并通过 Super-LIO 输出里程计
和点云地图。

目标环境：

- CPU 架构：ARM64（`aarch64`）
- 操作系统：Ubuntu 22.04
- ROS 版本：ROS 2 Humble
- 雷达：Hesai PandarXT-16
- IMU 来源：Unitree SDK2 DDS 话题 `rt/lowstate`

> 本文档只适用于 `ROS2` 分支。ROS 1 版本请查看 `main` 分支。

## 快速编译

### 1. 获取 ROS2 分支

第一次下载仓库：

```bash
cd ~
git clone --branch ROS2 --single-branch \
  https://github.com/Aphra-neck/GO2_Hesai.git
cd GO2_Hesai
```

如果机器上已经有该仓库：

```bash
cd ~/GO2_Hesai
git fetch origin
git switch ROS2
git pull --ff-only origin ROS2
```

确认当前分支：

```bash
git branch --show-current
```

输出应为：

```text
ROS2
```

### 2. 安装编译依赖

```bash
source /opt/ros/humble/setup.bash

sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  python3-colcon-common-extensions \
  python3-rosdep \
  libboost-all-dev \
  libeigen3-dev \
  libgflags-dev \
  libgoogle-glog-dev \
  libpcl-dev \
  libtbb-dev \
  libyaml-cpp-dev \
  ros-humble-pcl-ros \
  ros-humble-tf2-ros \
  ros-humble-rviz2
```

如果这台系统从未初始化过 `rosdep`：

```bash
sudo rosdep init
rosdep update
```

如果 `rosdep init` 提示已经存在 `20-default.list`，说明此前已完成初始化，直接执行
`rosdep update` 即可。

### 3. 检查 Unitree SDK2

`go2_imu_bridge` 依赖 Unitree SDK2。它不会由 `rosdep` 自动安装，编译前必须保证
CMake 能找到 `unitree_sdk2`：

```bash
cmake --find-package \
  -DNAME=unitree_sdk2 \
  -DCOMPILER_ID=GNU \
  -DLANGUAGE=CXX \
  -DMODE=EXIST
```

如果命令失败，请先按照 Unitree 官方说明安装 SDK2，或把 SDK2 安装前缀加入
`CMAKE_PREFIX_PATH`，然后再编译本仓库。

### 4. 编译整个工作空间

在仓库根目录执行：

```bash
cd ~/GO2_Hesai
source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src -r -y

MAKEFLAGS=-j2 colcon build \
  --symlink-install \
  --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

Jetson 内存有限，因此默认建议使用 `--executor sequential` 和 `MAKEFLAGS=-j2`。
Super-LIO 的 LTO 默认关闭，以降低 ARM64 链接阶段的内存占用。

编译完成后检查三个 ROS 2 包：

```bash
ros2 pkg prefix hesai_ros_driver
ros2 pkg prefix go2_imu_bridge
ros2 pkg prefix super_lio
```

三个命令都能输出安装路径，说明工作空间已被正确加载。

仅重新编译 IMU 桥接器时，可以执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select go2_imu_bridge
source install/setup.bash
```

## 系统数据流

```text
Hesai XT-16
  -> hesai_ros_driver
  -> /lidar_points (sensor_msgs/msg/PointCloud2)

Go2 LowState
  -> Unitree SDK2 DDS: rt/lowstate
  -> go2_imu_bridge
  -> /imu/data (sensor_msgs/msg/Imu)

/lidar_points + /imu/data
  -> Super-LIO ROS 2
  -> /lio/odom
  -> /lio/cloud_world
```

仓库包含：

- `src/HesaiLidar_ROS_2.0`：原生支持 ROS 2 的 Hesai 驱动，本分支没有修改其源码。
- `src/go2_imu_bridge`：使用 `rclcpp` 编写的 Unitree DDS 到 ROS 2 IMU 桥接器。
- `src/Super-LIO`：来自 Super-LIO 上游 `ros2` 分支，并加入 Humble/ARM64
  兼容修正。具体来源见 `src/Super-LIO/UPSTREAM.md`。
- `shell/start_slam.sh`：顺序启动雷达、IMU 桥接器和 Super-LIO 的脚本。

## 一键启动

完成编译后，在仓库根目录执行：

```bash
cd ~/GO2_Hesai
./shell/start_slam.sh
```

脚本会执行以下流程：

1. 启动 Hesai 驱动并等待 `/lidar_points`。
2. 启动 Go2 IMU 桥接器并等待 `/imu/data`。
3. 启动 Super-LIO。
4. 退出时清理所有由脚本启动的 ROS 2 进程。

默认不启动 RViz2。如需显示：

```bash
RVIZ=true ./shell/start_slam.sh
```

可用环境变量：

| 变量 | 默认值 | 作用 |
| --- | --- | --- |
| `GO2_NETWORK_INTERFACE` | `eth10` | Unitree SDK2 接收 LowState 的网卡 |
| `GO2_IMU_RATE` | `200.0` | IMU 最大发布频率，单位 Hz |
| `RVIZ` | `false` | 是否启动 RViz2 |
| `SLAM_LOG_DIR` | `~/slam_logs` | Hesai 和 IMU bridge 日志目录 |

例如：

```bash
GO2_NETWORK_INTERFACE=eth10 \
GO2_IMU_RATE=200.0 \
RVIZ=true \
SLAM_LOG_DIR=~/slam_logs \
./shell/start_slam.sh
```

## 分别启动各组件

以下命令都需要在仓库根目录执行。

终端 1，启动 Hesai XT-16：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run hesai_ros_driver hesai_ros_driver_node --ros-args \
  -p config_path:="$PWD/src/HesaiLidar_ROS_2.0/config/config.yaml"
```

终端 2，启动 Go2 IMU 桥接器：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run go2_imu_bridge go2_imu_bridge_node --ros-args \
  -p net:=eth10 \
  -p publish_rate:=200.0 \
  -p frame_id:=go2_imu \
  -p imu_topic:=/imu/data
```

终端 3，启动 Super-LIO：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch super_lio hesai.py rviz:=false
```

## 检查运行状态

查看话题：

```bash
ros2 topic list | grep -E '^/(lidar_points|imu/data|lio/)'
```

查看频率：

```bash
ros2 topic hz /lidar_points
ros2 topic hz /imu/data
ros2 topic hz /lio/odom
```

正常情况下大致为：

- `/lidar_points`：约 10 Hz
- `/imu/data`：约 160-200 Hz
- `/lio/odom`：约 10 Hz

读取一帧传感器数据：

```bash
ros2 topic echo --once --qos-profile sensor_data /lidar_points
ros2 topic echo --once --qos-profile sensor_data /imu/data
```

检查发布者和订阅者的 QoS：

```bash
ros2 topic info -v /lidar_points
ros2 topic info -v /imu/data
```

IMU bridge 提供 reliable/volatile QoS，Super-LIO 使用
best-effort/volatile 订阅；该 offered/requested 组合在 ROS 2 中兼容。

## 关键配置

### Hesai XT-16

配置文件：

```text
src/HesaiLidar_ROS_2.0/config/config.yaml
```

当前机器狗配置中的关键值：

```yaml
device_ip_address: 192.168.123.20
host_ip_address: 192.168.123.18
udp_port: 2368
device_udp_src_port: 10000
use_timestamp_type: 1
ros_send_point_cloud_topic: /lidar_points
```

`use_timestamp_type: 1` 必须保留。它让点云采用 Jetson 接收时间，从而与
`go2_imu_bridge` 生成的 `/imu/data` 时间戳使用同一系统时钟。

### Super-LIO

Go2 + XT-16 配置文件：

```text
src/Super-LIO/src/super_lio/config/hesai.yaml
```

关键参数：

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

### Go2 IMU bridge

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `net` | `eth10` | Unitree SDK2 使用的网卡 |
| `publish_rate` | `200.0` | IMU 最大发布频率 |
| `frame_id` | `go2_imu` | ROS 2 消息坐标系 |
| `imu_topic` | `/imu/data` | IMU 输出话题 |

bridge 只读取 Go2 LowState 并发布 IMU，不会向机器狗发送控制命令。

## 常见问题

### 找不到 `unitree_sdk2`

典型错误：

```text
Could not find a package configuration file provided by "unitree_sdk2"
```

先查找 SDK2 的 CMake 配置：

```bash
find /usr /usr/local -name 'unitree_sdk2Config.cmake' 2>/dev/null
```

如果已经安装但不在默认路径，把包含该配置的安装前缀加入
`CMAKE_PREFIX_PATH`，重新打开终端后再编译。

### Jetson 编译时内存不足

使用串行构建并限制并发：

```bash
MAKEFLAGS=-j2 colcon build \
  --symlink-install \
  --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### `/lidar_points` 没有数据

确认 `eth10` 地址和雷达 UDP 数据：

```bash
ip address show eth10
sudo tcpdump -i eth10 -nn 'udp port 2368' -c 10
```

正常应能看到 `192.168.123.20:10000` 发往 `192.168.123.18:2368` 的数据。
同时查看驱动日志：

```bash
tail -n 100 ~/slam_logs/hesai.log
```

### `/imu/data` 没有数据

确认网卡名和 bridge 日志：

```bash
ip link show eth10
tail -n 100 ~/slam_logs/go2_imu_bridge.log
```

也可以单独运行 bridge，直接查看报错：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run go2_imu_bridge go2_imu_bridge_node --ros-args -p net:=eth10
```

### `/lio/odom` 没有输出

依次确认：

1. `/lidar_points` 和 `/imu/data` 都持续有数据。
2. 两个话题的时间戳都接近 Jetson 当前系统时间。
3. Super-LIO 确实订阅 `/lidar_points` 和 `/imu/data`。
4. Hesai 配置中的 `use_timestamp_type` 仍为 `1`。

检查连接：

```bash
ros2 node info /super_lio_node
ros2 topic info -v /lidar_points
ros2 topic info -v /imu/data
```

## 上游来源

Super-LIO 来自：

- 仓库：<https://github.com/Liansheng-Wang/Super-LIO.git>
- 分支：`ros2`
- 基准提交：`f89f48d`

本分支在该版本上补充了 Humble 构建依赖、可选 Livox 支持、TBB 显式链接、
ROS 2 参数类型修正及 Go2 + XT-16 配置。
