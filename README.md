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

下面统一使用当前 Jetson 部署路径 `~/catkin_ws`。首次安装时它必须是新建的空
目录，绝不能复用已有的 ROS 1 工作空间。

第一次下载仓库：

```bash
cd ~
mkdir catkin_ws
cd ~/catkin_ws

git -c http.proxy=http://192.168.151.143:7890 \
  -c https.proxy=http://192.168.151.143:7890 \
  clone --branch ROS2 --single-branch \
  https://github.com/Aphra-neck/GO2_Hesai.git .
```

克隆命令最后的 `.` 表示把仓库内容放进当前空目录。完成后的结构应为：

```text
~/catkin_ws/
├── .git/
├── src/
├── shell/
└── README.md
```

不应出现下面这种多余嵌套：

```text
~/catkin_ws/GO2_Hesai/src/
```

如果这个独立工作空间已经克隆过仓库，拉取 ROS2 分支：

```bash
cd ~/catkin_ws
test "$(git branch --show-current)" = ROS2 || exit 1
git -c http.proxy=http://192.168.151.143:7890 \
  -c https.proxy=http://192.168.151.143:7890 \
  pull --ff-only origin ROS2
```

这些代理设置只作用于单条 Git 命令，不写入全局配置。如果现场网络可直接访问
GitHub，可以省略两个 `-c ...proxy=...` 参数。

代码修改、commit 和 push 只在本地开发 checkout 中完成，目标为 `origin/ROS2`；
Jetson 的 `~/catkin_ws` 只使用上面的 `pull --ff-only` 更新。确认机器人静止且 SLAM、
规划、SDK2 bridge 和日志上传均已停止后，本地 Windows PowerShell 可执行：

```powershell
Set-Location C:\path\to\GO2_Hesai
git status --short --branch
git -c http.proxy=http://192.168.151.143:7890 `
  -c https.proxy=http://192.168.151.143:7890 `
  push origin ROS2
```

公开仓库的 push 仍需要当前开发机具有写权限；使用已有 Git 凭据，不把 token 写进
命令、脚本或 remote URL。

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
  libssl-dev \
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

`go2_imu_bridge` 和 `utree_go2_sdk2_bridge` 依赖 Unitree SDK2。它不会由
`rosdep` 自动安装。先检查 SDK2 的 CMake 配置是否存在：

```bash
sdk2_config="$(find /usr /usr/local \
  -name 'unitree_sdk2Config.cmake' -print -quit 2>/dev/null)"
test -n "$sdk2_config" && printf 'Found: %s\n' "$sdk2_config"
```

如果没有输出，请先按照 Unitree 官方说明安装 SDK2。如果文件存在但后续构建仍找
不到它，把该配置所在的安装前缀加入 `CMAKE_PREFIX_PATH`。不要使用
`cmake --find-package` 判断 SDK2 是否可用；它可能在 `FindThreads` 处产生假阴性。
最终以本节的实际 `colcon build` 是否通过为准。

### 4. 编译整个工作空间

在仓库根目录执行：

```bash
cd ~/catkin_ws
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

编译完成后检查七个 ROS 2 包：

```bash
ros2 pkg prefix hesai_ros_driver
ros2 pkg prefix go2_imu_bridge
ros2 pkg prefix basic
ros2 pkg prefix super_lio
ros2 pkg prefix utree_dog_msgs
ros2 pkg prefix utree_dog_navigation
ros2 pkg prefix utree_go2_sdk2_bridge
```

七个命令都能输出安装路径，说明工作空间已被正确加载。

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
  -> /lio/odom (raw IMU pose)
  -> /lio/cloud_world

/lio/odom
  -> body_odom_adapter (-90 deg yaw)
  -> /lio/body_odom (world -> base_link pose)

/lio/body_odom + /lio/cloud_world + /goal_pose
  -> utree_dog_navigation
  -> /terrain_map
  -> /terrain_costmap
  -> /body_path

/body_path + /lio/body_odom
  -> utree_go2_sdk2_bridge (default: disabled)
  -> Unitree SDK2 SportClient
```

仓库包含：

- `src/HesaiLidar_ROS_2.0`：原生支持 ROS 2 的 Hesai 驱动，本分支没有修改其源码。
- `src/go2_imu_bridge`：使用 `rclcpp` 编写的 Unitree DDS 到 ROS 2 IMU 桥接器。
- `src/Super-LIO`：来自 Super-LIO 上游 `ros2` 分支，并加入 Humble/ARM64
  兼容修正。具体来源见 `src/Super-LIO/UPSTREAM.md`。
- `src/utree_dog_msgs`：地形规划所需的 ROS 2 自定义消息。
- `src/utree_dog_navigation`：IMU 到机身朝向适配、时序地形图和机身 lattice 路径规划器。
- `src/utree_go2_sdk2_bridge`：将 `/body_path` 转换为受限的 Unitree SDK2
  `SportClient` 指令，默认禁止运动。
- `shell/ros2_environment.sh`：统一准备 Jetson 的 Humble、工作空间和 Fast DDS 环境。
- `shell/start_slam.sh`：顺序启动雷达、IMU 桥接器和 Super-LIO 的脚本。
- `shell/start_navigation.sh`：默认无界面启动规划层；只在明确需要 Jetson 本地显示时
  设置 `PLANNING_RVIZ=true`。
- `shell/start_sdk2_bridge.sh`：安全检查后启动默认禁用的路径执行器。
- `tools/go2-log`：采集、检查、停止和上传有界诊断会话。

## 运行拓扑与首次配置

集成运行采用分布式模式：Jetson 只运行传感器、SLAM 和规划节点，WSL2 只运行
项目唯一的 RViz2。这样可以避免 Jetson 本地 RViz 将 `/lio/odom` 和点云处理频率
从约 9 Hz 长期压低到约 3 Hz。

以下运行命令按当前部署路径编写：

- Jetson 仓库：`~/catkin_ws`
- WSL2 配置目录：`~/go2_rviz`

如果仓库位于其他目录，只替换路径，不改变环境变量和启动顺序。

### 网络拓扑

| 端点 | 地址 | 用途 |
| --- | --- | --- |
| Jetson Wi-Fi | `192.168.151.213/24` | ROS 2 数据与 WSL2 通信 |
| WSL2 mirrored | `192.168.151.143/24` | 运行 RViz2 |
| Jetson `enP8p1s0` | `192.168.123.18/24` | Go2 与 Hesai 设备网络 |
| Hesai XT-16 | `192.168.123.20` | UDP 点云发送端 |
| Git 代理 | `192.168.151.143:7890` | Jetson 访问 GitHub |

ROS 2 使用 domain `30` 和 Fast DDS。Unitree SDK2 在 `enP8p1s0` 上独立使用
DDS domain `0`。不要把 ROS 2 切换为 `rmw_cyclonedds_cpp`。

### Fast DDS 静态单播约定

本项目不依赖跨主机 multicast，也不再使用 Fast DDS Discovery Server。两端通过
以下 profile 进行静态单播发现：

```text
Jetson: config/fastdds/jetson_wifi.xml
WSL2:   config/fastdds/wsl2_mirrored.xml
```

每份 `initialPeersList` 必须同时包含远端 IP 和本机 IP。关闭 builtin transports 后，
如果只保留远端 IP，跨主机通信可能成功，但同机 ROS 2 节点会互相发现失败。

Humble/Fast DDS 2.6 使用：

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=<profile-path>
```

Jetson 的三个项目启动脚本会自动 source `shell/ros2_environment.sh`，不需要在每个
启动终端重复导出这些变量。该 helper 使用自身路径定位仓库，不依赖当前 `$PWD`，并
自动加载 Humble、`install/setup.bash`、校验 Jetson profile 和停止旧 ROS daemon。
只有改用另一份 Jetson profile 时才设置 `GO2_FASTDDS_PROFILE=/absolute/path/file.xml`。
WSL2 不使用 Jetson helper，仍按下文加载自己的 `wsl2_mirrored.xml`。

不要同时设置 `ROS_DISCOVERY_SERVER` 或 `ROS_SUPER_CLIENT`。`ROS_SUPER_CLIENT=TRUE`
不是有效地址配置。

### 配置 WSL2 mirrored 网络

在 Windows PowerShell 打开配置文件：

```powershell
notepad.exe "$env:USERPROFILE\.wslconfig"
```

下面内容必须写入文件，不能逐行当作 PowerShell 命令执行：

```ini
[wsl2]
networkingMode=mirrored
dnsTunneling=true
autoProxy=true
firewall=true
```

保存后执行：

```powershell
wsl --shutdown
wsl -l -v
```

`GO2-ROS2-Humble` 应显示 `VERSION 2`。进入 WSL2 后确认 Wi-Fi 地址：

```bash
ip -4 -brief address
```

当前 profile 要求地址中出现 `192.168.151.143/24`。如果 Windows Wi-Fi 地址改变，
必须同步更新两份 XML 和下面的防火墙规则。

### 配置 Windows 入站防火墙

Domain 30 的 Fast DDS participant 端口位于 `14900` 起始范围。本项目配置
`maxInitialPeersRange=64`，Windows 管理员 PowerShell 中允许：

```powershell
New-NetFirewallRule `
  -DisplayName "GO2 ROS2 FastDDS Domain 30" `
  -Direction Inbound `
  -Action Allow `
  -Protocol UDP `
  -LocalPort 14900-15199 `
  -RemoteAddress 192.168.151.213 `
  -Profile Any
```

查看规则：

```powershell
Get-NetFirewallRule -DisplayName "GO2 ROS2 FastDDS Domain 30" |
  Format-List DisplayName,Enabled,Direction,Action
```

### 准备 WSL2 RViz

WSL2 使用 Ubuntu 22.04 和 ROS 2 Humble。只需要标准 RViz 插件，不需要在 WSL2
编译整个仓库。确认依赖：

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix rviz2
ros2 pkg prefix rmw_fastrtps_cpp
```

必须进入 `GO2-ROS2-Humble` 这个 WSL2 发行版。先在 PowerShell 确认并进入：

```powershell
wsl -l -v
wsl -d GO2-ROS2-Humble
```

在 WSL2 中，`lsb_release -ds` 必须显示 Ubuntu 22.04，且
`/opt/ros/humble/setup.bash` 必须存在。本节假定 WSL2、Ubuntu 22.04、WSLg 和
ROS 2 Humble 已安装；若任一前提缺失，先按 Microsoft WSL 与 ROS 2 Humble 官方
安装文档补齐，再继续配置 DDS。不要使用旧的
`RflySim-20.04` WSL1，也不要在该终端 source Foxy、Noetic 或其他工作空间的
`setup.bash`；混合 ROS 发行版可能使 RViz 配置崩溃或加载错误插件。

下载 Fast DDS 与 RViz 配置：

```bash
mkdir -p ~/go2_rviz/config/fastdds ~/go2_rviz/rviz
GO2_RVIZ_CONFIG_REF=f36bc5f556fbc6ac6bcc4bcdfb2f9e068ca529d0

curl --fail --location \
  "https://raw.githubusercontent.com/Aphra-neck/GO2_Hesai/${GO2_RVIZ_CONFIG_REF}/config/fastdds/wsl2_mirrored.xml" \
  -o ~/go2_rviz/config/fastdds/wsl2_mirrored.xml

curl --fail --location \
  "https://raw.githubusercontent.com/Aphra-neck/GO2_Hesai/${GO2_RVIZ_CONFIG_REF}/src/utree_dog_navigation/rviz/hesai_navigation.rviz" \
  -o ~/go2_rviz/rviz/hesai_navigation.rviz
```

`GO2_RVIZ_CONFIG_REF` 固定到本次实机验证通过的配置提交，避免 WSL2 与 Jetson 因
下载时刻不同而使用两个版本。以后有意修改 DDS 或 RViz 配置时，应在完成双端验证后
同步更新这里的提交号。

如果 WSL2 不能直接访问 GitHub，只对下载命令增加
`--proxy http://127.0.0.1:7890`。不要把代理写入仓库。

检查静态 peer：

```bash
grep -n '<address>' ~/go2_rviz/config/fastdds/wsl2_mirrored.xml
```

输出必须同时包含 `192.168.151.213` 和 `192.168.151.143`。

## 日常完整启动

### 0. 启动前检查

- 机器人处于安全静止状态。
- 不运行 RL `/lowcmd` 控制器。
- 不启动 `utree_go2_sdk2_bridge`。
- 不运行旧的 Discovery Server。
- 不在 SLAM、规划或运动期间执行 Git pull、push 或日志 upload。

Jetson 检查残留进程：

```bash
pgrep -af \
  'hesai_ros_driver|go2_imu_bridge|super_lio|utree_dog_navigation|go2_sdk2_bridge|rl_controller|rviz2' \
  || true
```

如果输出旧进程，先回到其原终端按 `Ctrl+C`，不要重复启动第二套节点。

### 1. Jetson 终端 1：启动无界面 SLAM

```bash
cd ~/catkin_ws
./shell/start_slam.sh
```

保持该终端运行。正常启动顺序：

```text
/lidar_points is active.
/imu/data is active.
[3/3] Starting Super-LIO...
... KF init done
... Map init done
```

`start_slam.sh` 固定使用 `rviz:=false`。不要在 Jetson 启动 Super-LIO 自带 RViz。
脚本会自动准备 ROS/Fast DDS 环境，并拒绝在检测到 RL 控制器进程或 `/lowcmd`
publisher 时启动。日常命令不需要在外层 source ROS 或重复 export 环境变量。

### 2. Jetson 终端 2：启动无界面规划

等待 SLAM 出现 `Map init done`，再新开 Jetson 终端：

```bash
cd ~/catkin_ws
./shell/start_navigation.sh
```

该脚本默认 `PLANNING_RVIZ=false`。只有临时改回 Jetson 本地可视化时才使用
`PLANNING_RVIZ=true ./shell/start_navigation.sh`；分布式日常流程不要设置它。

正常情况下会启动：

```text
body_odom_adapter_node
imu_to_base_link_tf
imu_to_hesai_lidar_tf
terrain_mapper_node
body_lattice_planner_node
```

启动瞬间出现一次 `Waiting for odometry` 是正常的；如果持续出现，检查
`/lio/body_odom` 和 Fast DDS profile。

### 3. WSL2 终端：启动唯一 RViz

```bash
source /opt/ros/humble/setup.bash

unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT CYCLONEDDS_URI \
  FASTDDS_DEFAULT_PROFILES_FILE
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/go2_rviz/config/fastdds/wsl2_mirrored.xml"
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
test -r "$FASTRTPS_DEFAULT_PROFILES_FILE" || exit 1

ros2 daemon stop
rviz2 -d "$HOME/go2_rviz/rviz/hesai_navigation.rviz"
```

RViz 配置固定使用：

- Fixed Frame：`world`
- `/lio/cloud_world`：Best Effort、Volatile、Decay Time `10` 秒
- `/lio/body_odom`
- `/terrain_costmap`：Reliable、Transient Local
- `/body_path`：Reliable、Transient Local
- Set Goal：发布 `/goal_pose`

WSLg 可能输出一条 GLSL sampler 警告。只要 RViz 进程仍存活且点云可见，该警告
与 DDS 无关；如果仅代价地图贴图异常，再单独排查 WSLg 渲染。

### 4. 从 WSL2 验证实时数据

不要只依赖 `ros2 topic list`。使用显式类型和 `--no-daemon` 验证真实消息：

```bash
timeout 15 ros2 topic echo --no-daemon --once --qos-profile sensor_data \
  /lio/odom nav_msgs/msg/Odometry --field header

timeout 15 ros2 topic echo --no-daemon --once --qos-profile sensor_data \
  /lio/cloud_world sensor_msgs/msg/PointCloud2 --field header

timeout 15 ros2 topic echo --no-daemon --once --qos-profile sensor_data \
  /lio/cloud_world sensor_msgs/msg/PointCloud2 --field width

timeout 15 ros2 topic echo --no-daemon --once --qos-profile sensor_data \
  /lio/body_odom nav_msgs/msg/Odometry --field header

timeout 15 ros2 topic echo --no-daemon --once \
  --qos-reliability reliable --qos-durability transient_local \
  /terrain_costmap nav_msgs/msg/OccupancyGrid --field header
```

四个消息的 `frame_id` 都应为 `world`，且 `/lio/cloud_world` 的 `width` 必须大于
`0`。这同时验证发现、payload 传输和非空点云，而不只是看到话题名。

### 5. 只验证规划，不执行运动

在 WSL2 RViz 中使用 Set Goal 设置近距离、平地、完全可见的目标。此操作只应生成
`/body_path`，不会自动控制机器人，因为 SDK2 bridge 尚未启动。

检查路径：

```bash
ros2 topic info -v /body_path
timeout 10 ros2 topic echo --no-daemon --once \
  --qos-reliability reliable --qos-durability transient_local \
  /body_path nav_msgs/msg/Path --field header
```

路径必须使用 `world`。在完成障碍穿越、footprint、地图边界和 stale path 验收前，
不要进入自动运动。

### 6. 停止顺序

1. 如果 SDK2 bridge 曾启用，先调用 `enable_motion` 的 `false` 分支并确认停车。
2. 在 SDK2 bridge 终端按 `Ctrl+C`，等待执行器退出。
3. 在 Jetson 规划终端按 `Ctrl+C`，等待规划节点退出。
4. 在 Jetson SLAM 终端只按一次 `Ctrl+C`。如果启用了地图保存，保持终端和电源
   可用，直到出现 `Final map saved to:` 且进程正常退出。
5. 关闭 WSL2 RViz。
6. 确认没有运行进程后，再停止和上传日志。日志采集器不会随 launch 自动停止。

```bash
pgrep -af \
  'hesai_ros_driver|go2_imu_bridge|super_lio|utree_dog_navigation|go2_sdk2_bridge|rl_controller' \
  || true
./tools/go2-log stop
./tools/go2-log upload
```

### 环境变量

| 变量 | 默认值或当前值 | 作用 |
| --- | --- | --- |
| `FASTRTPS_DEFAULT_PROFILES_FILE` | Jetson/WSL2 各自 XML | Fast DDS Wi-Fi 静态 peer |
| `GO2_FASTDDS_PROFILE` | Jetson 仓库内的 `jetson_wifi.xml` | 覆盖 Jetson 启动脚本使用的 profile |
| `ROS_DOMAIN_ID` | `30` | 与 Unitree SDK domain 0 隔离 |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | 避免 ROS 2 与 SDK2 CycloneDDS 冲突 |
| `ROS_LOCALHOST_ONLY` | `0` | 允许跨主机 ROS 2 通信 |
| `GO2_NETWORK_INTERFACE` | `enP8p1s0` | Unitree SDK2 LowState 网卡 |
| `GO2_IMU_RATE` | `200.0` | IMU 最大发布频率，单位 Hz |
| `SLAM_LOG_DIR` | `~/slam_logs` | Hesai 和 IMU bridge 日志目录 |
| `UNITREE_SDK_LIBRARY_DIR` | `/usr/local/lib` | Unitree SDK 配套 DDS 动态库目录 |
| `GO2_BODY_YAW_OFFSET_RAD` | `-1.5707963267948966` | IMU 到 `base_link` 的 yaw 校正 |
| `PLANNING_RVIZ` | `false` | 仅显式设为 `true` 时在 Jetson 启动规划 RViz |

### 坐标与 RViz 所有权

规划配置与 Super-LIO 对齐：

```yaml
map_frame: world
cloud_topic: /lio/cloud_world
odom_topic: /lio/body_odom
```

坐标关系为 `world -> imu`（Super-LIO 动态发布）、`imu -> base_link`
（零平移，yaw `-90 deg`）和 `imu -> hesai_lidar`（平移
`0.171 0 0.0908`，无旋转）。`/lio/body_odom` 保留原始位置和时间戳，将
`/lio/odom` 姿态右乘相同的 `-90 deg` yaw。点云、Super-LIO world 和 Hesai
外参不会被旋转。

现场直行标定得到 `-87.39 deg`，当前仍优先使用机械坐标的标称 `-90 deg`。只有
重复测试确认需要实验值时，才在规划终端覆盖：

```bash
GO2_BODY_YAW_OFFSET_RAD=-1.525243233318 ./shell/start_navigation.sh
```

不要修改 `lio.extrinsic.lidar_imu` 来校正机身方向。

### SDK2 路径执行器

SDK2 bridge 不属于当前日常启动流程。以下所有门槛必须有日志和测试记录后，才可在
第三个 Jetson 终端直接运行启动脚本；它会自动使用与 SLAM/规划相同的 Fast DDS
环境：

- 无头感知加远程 RViz 时，IMU、雷达、里程计和世界点云达到性能基线。
- `world -> imu -> base_link` 方向与实机直行一致，静止漂移通过验收。
- yaw-aware footprint、整段路径碰撞、地图边界和未知区行为已经通过测试。
- 规划失败、输入过期或非法 frame 会发布/处理空路径并清除旧目标。
- `/lowcmd` 互斥、遥控器接管、实体急停和低速 field-test 参数已准备完成。

在这些门槛通过前，不启动 bridge，也不调用 `enable_motion` 的 `true` 分支。

首次实机运动必须有独立遥控器安全员，手始终放在官方接管/急停控制上；不能把 ROS
service 或 DDS 链路当作唯一停车手段。若 disable service 无响应，立即用遥控器接管
或执行实体急停，不等待软件恢复。

```bash
./shell/start_sdk2_bridge.sh
```

脚本会拒绝与 RL `/lowcmd` 控制器并行运行；节点运行期间也会持续检查
`/lowcmd` 发布者，一旦发现便立即禁用 SportClient 并停车。节点启动后仍为禁用状态；确认
`/body_path`、`/lio/body_odom` 和机器人周边安全后，才可显式启用：

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

实机上传当前使用对 `Aphra-neck/G02_log` 具有写权限的 GitHub Deploy Key，并通过
SSH 443 端口连接。首次外场使用前必须在所有机器人进程停止时完成预检：

```bash
git -C ~/G02_log remote -v
GIT_TERMINAL_PROMPT=0 git -C ~/G02_log \
  push --dry-run origin HEAD:refs/heads/main
```

第二条命令必须在不改变远端的情况下成功；它验证的是写权限，而公开仓库的
`ls-remote` 只能验证读权限。如果命令报告 authentication 或 permission 错误，说明
写入认证没有配置完成，不能在采集结束后临时处理。Deploy Key、credential helper
均可作为认证来源，
但禁止把 token 或私钥写入脚本、远端 URL、报告或日志。`GO2_LOG_PROXY` 只作用于
HTTP(S) Git；当前 SSH remote 不经过该 HTTP 代理。

Windows 本地拉取并分析：

```powershell
git -C D:\G02_log `
  -c http.proxy=http://192.168.151.143:7890 `
  -c https.proxy=http://192.168.151.143:7890 `
  pull --ff-only origin main

# 先替换为本机 GO2_Hesai checkout 的实际路径
Set-Location C:\path\to\GO2_Hesai
python .\tools\analyze_diagnostics.py `
  D:\G02_log\sessions\<session-id>
```

分析器生成 Markdown 报告、话题/进程/SDK2 汇总和
`body_odometry_audit.csv`，自动核对实机原始与校正里程计的时间戳、frame、四元数及
yaw 偏置，并保留原始 JSONL。PCD、rosbag 和 core dump
不得进入 Git；用 SCP 分别传到 `D:\GO2_Data\maps`、`D:\GO2_Data\bags` 和
`D:\GO2_Data\cores`。详细约束见 `tools/README.md`。

## 分别启动各组件

本节只用于定位单个节点故障，日常运行优先使用前面的启动脚本，因为脚本还负责重复
进程检查、启动顺序和诊断采集。以下命令都需要在 Jetson 仓库根目录执行。

Unitree SDK 固定使用 CycloneDDS domain 0。为避免它与同进程内的 ROS 2
CycloneDDS 冲突，ROS 2 使用 Fast DDS 和非零 domain。每个手动诊断终端先执行：

```bash
cd ~/catkin_ws
source ./shell/ros2_environment.sh
```

启动脚本中的环境不会回写其父终端，所以独立执行 `ros2 topic` 或单组件诊断时仍要
source 该 helper。省略 profile 会使分布式发现失效；profile 只含远端 peer 时，还会
使同机 publisher 与 subscriber 互相发现失败。

终端 1，启动 Hesai XT-16：

```bash
ros2 run hesai_ros_driver hesai_ros_driver_node --ros-args \
  -p config_path:="$PWD/src/HesaiLidar_ROS_2.0/config/config.yaml"
```

终端 2，启动 Go2 IMU 桥接器：

```bash
export LD_LIBRARY_PATH="/usr/local/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

ros2 run go2_imu_bridge go2_imu_bridge_node --ros-args \
  -p net:=enP8p1s0 \
  -p publish_rate:=200.0 \
  -p frame_id:=go2_imu \
  -p imu_topic:=/imu/data
```

终端 3，启动 Super-LIO：

```bash
ros2 launch super_lio hesai.py rviz:=false
```

## 检查运行状态

检查命令所在终端也必须先执行：

```bash
cd ~/catkin_ws
source ./shell/ros2_environment.sh
```

查看话题：

```bash
ros2 topic list --no-daemon | grep -E '^/(lidar_points|imu/data|lio/)'
```

查看频率：

```bash
ros2 topic hz /lidar_points
ros2 topic hz /imu/data
ros2 topic hz /lio/odom
ros2 topic hz /lio/body_odom
```

正常情况下大致为：

- `/lidar_points`：约 10 Hz
- `/imu/data`：约 160-200 Hz
- `/lio/odom`：约 10 Hz
- `/lio/body_odom`：与 `/lio/odom` 相同

读取一帧传感器数据：

```bash
ros2 topic echo --once --qos-profile sensor_data /lidar_points
ros2 topic echo --once --qos-profile sensor_data /imu/data
ros2 topic echo --once --qos-profile sensor_data /lio/body_odom
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

#### 可选地图保存

当前默认 `lio.map.save_map: false`，因此不会自动生成 PCD。需要保存时，必须先停止
SLAM，在源码配置中改为 `true`，再重新构建并核对安装后的运行配置：

```bash
cd ~/catkin_ws
source /opt/ros/humble/setup.bash
nano src/Super-LIO/src/super_lio/config/hesai.yaml
# 把 lio.map.save_map 改为 true，保存并退出
colcon build --symlink-install --packages-select super_lio
source install/setup.bash
grep -n 'lio.map.save_map: true' \
  install/super_lio/share/super_lio/config/hesai.yaml || exit 1
```

配置 `save_map_dir: "map"` 和 `map_name: "map.pcd"` 的最终输出为：

```text
~/catkin_ws/src/Super-LIO/src/super_lio/map/map.pcd
```

新一轮保存会清空 `map/PCD/` 临时分片并覆盖同名 `map.pcd`，所以启动前先备份旧图。
如果旧图存在，可在启动 SLAM 前执行：

```bash
map_dir=~/catkin_ws/src/Super-LIO/src/super_lio/map
test ! -f "$map_dir/map.pcd" || \
  mv "$map_dir/map.pcd" \
    "$map_dir/map.pcd.before-$(date -u +%Y%m%dT%H%M%SZ)"
```

SLAM 启动后确认运行参数确实为 `true`：

```bash
ros2 param get /super_lio_node lio.map.save_map
```

结束时在 SLAM 终端只按一次 `Ctrl+C`，等待 `Final map saved to:` 和正常退出。日志中的
保存提示不足以证明 PCD 写盘成功，还必须检查本轮新文件非空、修改时间和哈希：

```bash
map_file=~/catkin_ws/src/Super-LIO/src/super_lio/map/map.pcd
test -s "$map_file" || exit 1
stat "$map_file"
sha256sum "$map_file"
```

PCD 不进入 Git。停止所有机器人进程后，从 Windows PowerShell 用 SCP 下载：

```powershell
New-Item -ItemType Directory -Force D:\GO2_Data\maps
$stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
scp unitree@192.168.151.213:/home/unitree/catkin_ws/src/Super-LIO/src/super_lio/map/map.pcd `
  "D:\GO2_Data\maps\map-$stamp.pcd"
Get-FileHash "D:\GO2_Data\maps\map-$stamp.pcd" -Algorithm SHA256
```

PowerShell 的 SHA-256 必须与 Jetson `sha256sum` 一致。

成功取回地图后，在下一次日常启动前把源码参数恢复为 `false` 并重新构建。否则后续
停机仍会保存和覆盖 PCD：

```bash
cd ~/catkin_ws
source /opt/ros/humble/setup.bash
nano src/Super-LIO/src/super_lio/config/hesai.yaml
# 把 lio.map.save_map 恢复为 false，保存并退出
colcon build --symlink-install --packages-select super_lio
source install/setup.bash
grep -n 'lio.map.save_map: false' \
  install/super_lio/share/super_lio/config/hesai.yaml || exit 1
```

下一次 SLAM 启动后，`ros2 param get /super_lio_node lio.map.save_map` 必须输出
`Boolean value is: False`。

### Go2 IMU bridge

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `net` | `enP8p1s0` | Unitree SDK2 使用的网卡 |
| `publish_rate` | `200.0` | IMU 最大发布频率 |
| `frame_id` | `go2_imu` | ROS 2 消息坐标系 |
| `imu_topic` | `/imu/data` | IMU 输出话题 |

bridge 只读取 Go2 LowState 并发布 IMU，不会向机器狗发送控制命令。

## 常见问题

### `ros2: command not found`

每个新终端都必须重新加载 ROS 2；只设置 `ROS_DOMAIN_ID` 不会把 `ros2` 加入
`PATH`。Jetson 诊断终端使用项目 helper：

```bash
cd ~/catkin_ws
source ./shell/ros2_environment.sh
command -v ros2
```

WSL2 则只执行 `source /opt/ros/humble/setup.bash`，不要 source Jetson helper、工作空间
或 Foxy/Noetic 环境。最后一条应输出 `/opt/ros/humble/bin/ros2`。

### 只能看到 `/parameter_events` 和 `/rosout`

这表示当前 participant 没有发现远端节点。先不要调 RViz 或 QoS，按顺序检查：

```bash
env | grep -E '^(ROS_|RMW_|FAST|CYCLONE)'
test -r "$FASTRTPS_DEFAULT_PROFILES_FILE"
grep -n '<address>' "$FASTRTPS_DEFAULT_PROFILES_FILE"
ip -4 -brief address
ros2 daemon stop
ros2 topic list --no-daemon
```

两端 profile 都必须同时包含 `192.168.151.213` 与 `192.168.151.143`，且 whitelist
地址必须属于本机。不要启动 `fastdds discovery`，不要设置 `ROS_DISCOVERY_SERVER`，
也不要把 multicast 测试当作静态单播配置的通过条件。

### Hesai 日志持续有 raw frame，但启动脚本等待 `/lidar_points` 超时

这说明雷达 UDP 和解码正常，故障位于 Jetson 本机 ROS 2 participant 发现。确认启动
横幅显示仓库内的 `jetson_wifi.xml`，且该 profile 同时包含远端 peer 和本机
`192.168.151.213`。结束残留的驱动/probe 进程后重新运行脚本；helper 会停止旧
daemon。不要重复启动多个 Hesai 驱动。

### 能看到话题或 endpoint，但收不到点云

`ros2 topic list` 只证明 discovery 成功，不证明用户数据已经送达。WSL2 先检查：

```bash
timeout 15 ros2 topic hz /lio/odom
timeout 15 ros2 topic hz /lio/cloud_world
ros2 topic info -v /lio/cloud_world
```

`/lio/cloud_world` 的 publisher 与 RViz subscriber 都应使用 Best Effort、Volatile。
Humble 的 `ros2 topic hz` 不能显式选择订阅 QoS，因此它只作为频率参考；上文带
`--qos-profile sensor_data` 的显式 `topic echo` 才是 payload 连通性的判据。
如果 endpoint 可见但 `hz` 无数据，检查 Windows `14900-15199/UDP` 入站规则和 QoS；
日常远程显示不订阅原始 `/lidar_points`，只显示 `/lio/cloud_world`。

### RViz 输出 OpenGL、Stereo 或 GLSL 警告

`Stereo is NOT SUPPORTED` 本身无害。只要 RViz 持续运行且点云可见，WSLg 的 GLSL
sampler 警告也不是 DDS 故障。如果默认 RViz 正常但项目配置崩溃，确认当前是
Ubuntu 22.04 + Humble，清除 Foxy/Noetic overlay，并重新下载 ROS2 分支的最新
`hesai_navigation.rviz`。

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
cd ~/catkin_ws
source ./shell/ros2_environment.sh
export LD_LIBRARY_PATH="/usr/local/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
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

如果单独启动时出现 `free(): invalid pointer`，先停止所有重复的 IMU bridge，再使用
`shell/start_slam.sh` 的标准启动流程。重点核对 ROS 侧为 `rmw_fastrtps_cpp`，且
`LD_LIBRARY_PATH` 最前面是 `/usr/local/lib` 中配套的 Unitree
`libddsc.so.0`/`libddscxx.so.0`；不要混用 ROS 自带的 CycloneDDS 运行库。

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

### `/lio/body_odom` 没有输出或方向不正确

`/lio/body_odom` 由规划 launch 中的 `body_odom_adapter` 发布。依次确认：

```bash
ros2 node info /body_odom_adapter
ros2 topic info -v /lio/body_odom
ros2 param get /body_odom_adapter yaw_offset
ros2 run tf2_ros tf2_echo imu base_link
```

默认 `yaw_offset` 和 `imu -> base_link` yaw 都应为
`-1.5707963267948966 rad`。如果两者不一致，停止规划和 SDK2 bridge 后重新启动
`shell/start_navigation.sh`；不要通过修改 `lio.extrinsic.lidar_imu` 修正机身方向。

## 上游来源

Super-LIO 来自：

- 仓库：<https://github.com/Liansheng-Wang/Super-LIO.git>
- 分支：`ros2`
- 基准提交：`f89f48d`

本分支在该版本上补充了 Humble 构建依赖、可选 Livox 支持、TBB 显式链接、
ROS 2 参数类型修正及 Go2 + XT-16 配置。
