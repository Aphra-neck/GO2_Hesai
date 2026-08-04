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

本仓库自身已经包含 `src/` 分层，因此应先创建一个全新的空工作空间，再把仓库
直接克隆到工作空间根目录。不要再额外创建 `GO2_Hesai/` 子目录。

下面使用 `~/go2_hesai_ros2_ws` 作为示例。也可以把工作空间命名为
`~/catkin_ws`，但它必须是新建的空目录，绝不能复用已有的 ROS 1 工作空间。

第一次下载仓库：

```bash
cd ~
mkdir go2_hesai_ros2_ws
cd ~/go2_hesai_ros2_ws

git clone --branch ROS2 --single-branch \
  https://github.com/Aphra-neck/GO2_Hesai.git .
```

克隆命令最后的 `.` 表示把仓库内容放进当前空目录。完成后的结构应为：

```text
~/go2_hesai_ros2_ws/
├── .git/
├── src/
├── shell/
└── README.md
```

不应出现下面这种多余嵌套：

```text
~/go2_hesai_ros2_ws/GO2_Hesai/src/
```

如果这个独立工作空间已经克隆过仓库，拉取 ROS2 分支：

```bash
cd ~/go2_hesai_ros2_ws
git fetch origin
git switch ROS2
git pull --ff-only origin ROS2
```

拉取完成后检查当前分支和远端跟踪关系：

```bash
git branch --show-current
git status --short --branch
git branch -vv
```

第一条命令必须输出：

```text
ROS2
```

`git status --short --branch` 应以类似下面的内容开头：

```text
## ROS2...origin/ROS2
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
  ros-humble-rmw-fastrtps-cpp \
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
cd ~/go2_hesai_ros2_ws
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

编译完成后检查六个 ROS 2 包：

```bash
ros2 pkg prefix hesai_ros_driver
ros2 pkg prefix go2_imu_bridge
ros2 pkg prefix super_lio
ros2 pkg prefix utree_dog_msgs
ros2 pkg prefix utree_dog_navigation
ros2 pkg prefix utree_go2_sdk2_bridge
```

六个命令都能输出安装路径，说明工作空间已被正确加载。

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

/lio/odom + /lio/cloud_world + /goal_pose
  -> utree_dog_navigation
  -> /terrain_map
  -> /terrain_costmap
  -> /body_path

/body_path + /lio/odom
  -> utree_go2_sdk2_bridge (default: disabled)
  -> Unitree SDK2 SportClient
```

仓库包含：

- `src/HesaiLidar_ROS_2.0`：原生支持 ROS 2 的 Hesai 驱动，本分支没有修改其源码。
- `src/go2_imu_bridge`：使用 `rclcpp` 编写的 Unitree DDS 到 ROS 2 IMU 桥接器。
- `src/Super-LIO`：来自 Super-LIO 上游 `ros2` 分支，并加入 Humble/ARM64
  兼容修正。具体来源见 `src/Super-LIO/UPSTREAM.md`。
- `src/utree_dog_msgs`：地形规划所需的 ROS 2 自定义消息。
- `src/utree_dog_navigation`：时序地形图和机身 lattice 路径规划器。
- `src/utree_go2_sdk2_bridge`：将 `/body_path` 转换为受限的 Unitree SDK2
  `SportClient` 指令，默认禁止运动。
- `shell/start_slam.sh`：顺序启动雷达、IMU 桥接器和 Super-LIO 的脚本。
- `shell/start_navigation.sh`：启动规划层和项目唯一的 RViz2。
- `shell/start_sdk2_bridge.sh`：安全检查后启动默认禁用的路径执行器。
- `tools/go2-log`：采集、检查、停止和上传有界诊断会话。

## 一键启动

完成编译后，在仓库根目录执行：

```bash
cd ~/go2_hesai_ros2_ws
./shell/start_slam.sh
```

脚本会执行以下流程：

1. 启动 Hesai 驱动并等待 `/lidar_points`。
2. 启动 Go2 IMU 桥接器并等待 `/imu/data`。
3. 启动 Super-LIO。
4. 退出时清理所有由脚本启动的 ROS 2 进程。

感知启动脚本固定使用 `rviz:=false`，不会启动 RViz2。规划层拥有项目中唯一的
RViz；在另一个终端启动：

```bash
cd ~/go2_hesai_ros2_ws
./shell/start_navigation.sh
```

规划脚本会先确认 `/lio/odom` 和 `/lio/cloud_world` 有数据，再启动地形构建、
机身路径规划和规划 RViz。不要同时运行 Super-LIO 自带的第二个 RViz。

可用环境变量：

| 变量 | 默认值 | 作用 |
| --- | --- | --- |
| `GO2_NETWORK_INTERFACE` | `enP8p1s0` | Unitree SDK2 接收 LowState 的网卡 |
| `GO2_IMU_RATE` | `200.0` | IMU 最大发布频率，单位 Hz |
| `SLAM_LOG_DIR` | `~/slam_logs` | Hesai 和 IMU bridge 日志目录 |
| `ROS_DOMAIN_ID` | `30` | ROS 2 domain；与 Unitree SDK 固定使用的 domain 0 隔离 |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | ROS 2 使用 Fast DDS，避免与 Unitree SDK 的 CycloneDDS 冲突 |
| `UNITREE_SDK_LIBRARY_DIR` | `/usr/local/lib` | Unitree SDK配套的 CycloneDDS动态库目录 |

例如：

```bash
GO2_NETWORK_INTERFACE=enP8p1s0 \
GO2_IMU_RATE=200.0 \
SLAM_LOG_DIR=~/slam_logs \
./shell/start_slam.sh
```

### 规划和 RViz

`shell/start_navigation.sh` 默认启动规划 RViz。固定坐标系为 `world`，显示
`/lio/cloud_world`、`/lio/odom`、`/terrain_costmap` 和 `/body_path`，并通过
`/goal_pose` 设置目标。世界点云使用 Best Effort QoS，RViz 累积时间为 10 秒。

如果 RViz 在另一台 ROS 2 计算机运行，Jetson 只启动规划节点：

```bash
PLANNING_RVIZ=false ./shell/start_navigation.sh
```

规划配置已经和 Super-LIO 对齐：

```yaml
map_frame: world
cloud_topic: /lio/cloud_world
odom_topic: /lio/odom
```

坐标关系为 `world -> imu`（Super-LIO 动态发布）、`imu -> base_link`（单位变换）
和 `imu -> hesai_lidar`（平移 `0.171 0 0.0908`，无旋转）。

### SDK2 路径执行器

规划路径验证完成后，可在第三个终端启动 SDK2 bridge：

```bash
./shell/start_sdk2_bridge.sh
```

脚本会拒绝与 RL `/lowcmd` 控制器并行运行；节点运行期间也会持续检查
`/lowcmd` 发布者，一旦发现便立即禁用 SportClient 并停车。节点启动后仍为禁用状态；确认
`/body_path`、`/lio/odom` 和机器人周边安全后，才可显式启用：

`enabled:=true` 启动配置会被拒绝，不能绕过人工授权。路径、里程计或控制参数包含
非有限值、非法四元数或不一致坐标系时，节点也会保持禁用并尝试停车。

```bash
ros2 service call /go2_sdk2_bridge/enable_motion \
  std_srvs/srv/SetBool '{data: true}'
```

随时禁用并停车：

```bash
ros2 service call /go2_sdk2_bridge/enable_motion \
  std_srvs/srv/SetBool '{data: false}'
```

如果 SDK2 尚未确认 `StopMove`，禁用服务会返回失败；节点仍保持禁用并按控制周期
持续重试停车，在确认成功前拒绝再次启用。

不要让 `utree_go2_sdk2_bridge` 和发布 `/lowcmd` 的 RL 控制器同时控制机器人。

### 诊断日志

三个启动脚本都会调用 `tools/go2-log start`。该命令是幂等的：同一会话已在采集时
不会重复创建进程。采集内容包括 Git 版本、ROS/DDS 环境、网络、参数、进程状态、
关键小消息和频率摘要以及 `/rosout` 警告/错误；不会复制完整雷达点云、世界点云或
`TerrainGrid`。

如需使用简短的全局命令，可在 Jetson 安装一次：

```bash
sudo install -m 0755 tools/go2-log /usr/local/bin/go2-log
```

以下示例假定已安装该命令；未安装时把 `go2-log` 替换为 `./tools/go2-log`。

运行期间只查看状态：

```bash
go2-log status
```

先停止 SLAM、规划和 SDK2 bridge，再停止并上传诊断会话：

```bash
go2-log stop
go2-log upload
```

`upload` 会在发现任一雷达、IMU、SLAM、规划、SDK2 或 RL 控制进程时拒绝 Git
push。每个会话最多 100 MiB，本地最多保留 20 个；只有远端提交验证成功的旧会话
才允许被清理。默认上传到公开仓库 `Aphra-neck/G02_log` 的 `main` 分支，并使用：

```bash
export GO2_LOG_PROXY=http://192.168.151.143:7890
```

Git 凭据由系统 credential helper 提供，禁止把 token 写入脚本、URL 或日志。

Windows 本地拉取并分析：

```powershell
git -C D:\G02_log `
  -c http.proxy=http://192.168.151.143:7890 `
  -c https.proxy=http://192.168.151.143:7890 `
  pull --ff-only origin main

python .\tools\analyze_diagnostics.py `
  D:\G02_log\sessions\<session-id>
```

分析器生成 Markdown 报告和 CSV 汇总，保留原始 JSONL。PCD、rosbag 和 core dump
不得进入 Git；用 SCP 分别传到 `D:\GO2_Data\maps`、`D:\GO2_Data\bags` 和
`D:\GO2_Data\cores`。详细约束见 `tools/README.md`。

## 分别启动各组件

以下命令都需要在仓库根目录执行。

Unitree SDK 固定使用 CycloneDDS domain 0。为避免它与同进程内的 ROS 2
CycloneDDS 冲突，ROS 2 使用 Fast DDS和非零 domain。所有手动启动和检查
终端都必须使用相同设置：

```bash
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

`shell/start_slam.sh` 会自动使用该默认值。

终端 1，启动 Hesai XT-16：

```bash
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run hesai_ros_driver hesai_ros_driver_node --ros-args \
  -p config_path:="$PWD/src/HesaiLidar_ROS_2.0/config/config.yaml"
```

终端 2，启动 Go2 IMU 桥接器：

```bash
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source install/setup.bash

export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH}"

ros2 run go2_imu_bridge go2_imu_bridge_node --ros-args \
  -p net:=enP8p1s0 \
  -p publish_rate:=200.0 \
  -p frame_id:=go2_imu \
  -p imu_topic:=/imu/data
```

终端 3，启动 Super-LIO：

```bash
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch super_lio hesai.py rviz:=false
```

## 检查运行状态

检查命令所在终端也必须先执行：

```bash
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

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
| `net` | `enP8p1s0` | Unitree SDK2 使用的网卡 |
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

确认 `enP8p1s0` 地址和雷达 UDP 数据：

```bash
ip address show enP8p1s0
sudo tcpdump -i enP8p1s0 -nn 'udp port 2368' -c 10
```

正常应能看到 `192.168.123.20:10000` 发往 `192.168.123.18:2368` 的数据。
同时查看驱动日志：

```bash
tail -n 100 ~/slam_logs/hesai.log
```

### `/imu/data` 没有数据

确认网卡名和 bridge 日志：

```bash
ip link show enP8p1s0
tail -n 100 ~/slam_logs/go2_imu_bridge.log
```

也可以单独运行 bridge，直接查看报错：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH}"
RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=30 \
  ros2 run go2_imu_bridge go2_imu_bridge_node --ros-args -p net:=enP8p1s0
```

bridge 运行时可在另一个终端确认实际加载的 DDS库：

```bash
bridge_pid="$(pgrep -n -f go2_imu_bridge_node)"
awk '/libddsc|librmw|fastdds|fastrtps/ {print $6}' \
  "/proc/${bridge_pid}/maps" | sort -u
```

`/usr/local/lib/libddsc.so.0` 和 `libddscxx.so.0` 必须同时出现并配套供
Unitree SDK使用；ROS侧应出现 Fast DDS/Fast RTPS相关库，不应再加载
`/opt/ros/humble/lib/aarch64-linux-gnu/libddsc.so.0.10.5`。
`ldd` 只显示直接链接依赖，不能确认运行时动态选择的 RMW实现，因此以上
`/proc` 检查更准确。

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
