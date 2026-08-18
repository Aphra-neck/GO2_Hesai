# GO2_Hesai ROS 2

## 项目简介

本仓库的 `ROS2-2D-navigation` 分支用于在 Unitree Go2 的 Jetson 载荷计算机上，将 Hesai
XT-16 激光雷达和 Go2 内部 IMU 接入 ROS 2 Humble，并通过 Super-LIO 输出里程计
和点云地图。

### 二维导航阶段状态（2026-08-11）

`ROS2-2D-navigation` 阶段已经在 Jetson 实机完成以下闭环验证：XT-16 与
Super-LIO 世界系点云、带三维射线清除的确认障碍地图、二维投影与机身 footprint
膨胀、WSL2 分布式 RViz、`/body_path` 规划，以及 SDK2 低速路径执行。二维模式会在
点云或里程计过期时撤销路径，运动桥仍需每次进程启动后由操作员显式授权一次。

该结论只覆盖操作员确认的平地、机器狗站立状态和当前低速实测范围。terrain 模式、
楼梯/坡地以及未来接入 Go2 下装雷达后的融合仍属于下一阶段，不能由本次结果外推。

目标环境：

- CPU 架构：ARM64（`aarch64`）
- 操作系统：Ubuntu 22.04
- ROS 版本：ROS 2 Humble
- 雷达：Hesai PandarXT-16
- IMU 来源：Unitree SDK2 DDS 话题 `rt/lowstate`

> 本文档只适用于 `ROS2-2D-navigation` 分支。ROS 1 版本请查看 `main` 分支。

## 快速编译

### 1. 获取 ROS2 二维导航分支

本仓库自身已经包含 `src/` 分层，因此应先创建一个全新的空工作空间，再把仓库
直接克隆到工作空间根目录。不要再额外创建 `GO2_Hesai/` 子目录。

下面统一使用当前 Jetson 部署路径 `~/catkin_ws`。首次安装时它必须是新建的空
目录，绝不能复用已有的 ROS 1 工作空间。

第一次下载仓库：

```bash
cd ~
mkdir catkin_ws
cd ~/catkin_ws

git -c http.proxy=http://192.168.151.145:7890 \
  -c https.proxy=http://192.168.151.145:7890 \
  clone --branch ROS2-2D-navigation --single-branch \
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

如果这个独立工作空间已经克隆过仓库，先确认机器人节点和诊断会话均已停止，再拉取并增量
构建 ROS2 二维导航分支：

```bash
cd ~/catkin_ws
test "$(git branch --show-current)" = ROS2-2D-navigation || exit 1
git -c http.proxy=http://192.168.151.145:7890 \
  -c https.proxy=http://192.168.151.145:7890 \
  pull --ff-only origin ROS2-2D-navigation

source /opt/ros/humble/setup.bash
MAKEFLAGS=-j2 colcon build \
  --symlink-install \
  --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`git pull` 只更新源码，不能替代编译。上面的完整工作空间增量构建会让 colcon 检查所有包，
避免某次更新涉及 Hesai、IMU、Super-LIO、规划或 SDK2 bridge 时遗漏对应二进制；未改变的构建
产物仍由 colcon 复用。它也确保本轮 freshness、watchdog 和空路径撤销逻辑进入 `install/`。
构建失败时不要启动机器人节点，也不要通过只修改 YAML 绕过旧二进制。
更新后若使用可选的全局 `go2-log` 副本，还必须按“诊断日志”一节重新安装；标准流程始终使用
仓库内的 `./tools/go2-log`。

这些代理设置只作用于单条 Git 命令，不写入全局配置。如果现场网络可直接访问
GitHub，可以省略两个 `-c ...proxy=...` 参数。

代码修改、commit 和 push 只在本地开发 checkout 中完成，目标为
`origin/ROS2-2D-navigation`；
Jetson 的 `~/catkin_ws` 只使用上面的 `pull --ff-only` 更新。确认机器人静止且 SLAM、
规划、SDK2 bridge 和日志上传均已停止后，本地 Windows PowerShell 可执行：

```powershell
Set-Location C:\path\to\GO2_Hesai
git status --short --branch
git -c http.proxy=http://192.168.151.145:7890 `
  -c https.proxy=http://192.168.151.145:7890 `
  push origin ROS2-2D-navigation
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
ROS2-2D-navigation
```

`git status --short --branch` 应以类似下面的内容开头：

```text
## ROS2-2D-navigation...origin/ROS2-2D-navigation
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
  -> /flat_obstacle_filtered_map_3d (world XYZ confirmed/cleared obstacles)
  -> /flat_obstacle_inflated (2D footprint clearance)
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
| WSL2 mirrored | `192.168.151.145/24` | 运行 RViz2 |
| Jetson `enP8p1s0` | `192.168.123.18/24` | Go2 与 Hesai 设备网络 |
| Hesai XT-16 | `192.168.123.20` | UDP 点云发送端 |
| Git 代理 | `192.168.151.145:7890` | Jetson 访问 GitHub |

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

当前 profile 要求地址中出现 `192.168.151.145/24`。如果 Windows Wi-Fi 地址改变，
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
GO2_RVIZ_CONFIG_REF=c7b060e13da89c165adb2bd05d76bfb9ce00f4e7

curl --fail --location \
  "https://raw.githubusercontent.com/Aphra-neck/GO2_Hesai/${GO2_RVIZ_CONFIG_REF}/config/fastdds/wsl2_mirrored.xml" \
  -o ~/go2_rviz/config/fastdds/wsl2_mirrored.xml

curl --fail --location \
  "https://raw.githubusercontent.com/Aphra-neck/GO2_Hesai/${GO2_RVIZ_CONFIG_REF}/src/utree_dog_navigation/rviz/flat_obstacle_navigation.rviz" \
  -o ~/go2_rviz/rviz/flat_obstacle_navigation.rviz
```

`GO2_RVIZ_CONFIG_REF` 固定到当前双端地址配置提交，避免 WSL2 与 Jetson 因
下载时刻不同而使用两个版本。以后有意修改 DDS 或 RViz 配置时，应在完成双端验证后
同步更新这里的提交号。

如果 WSL2 不能直接访问 GitHub，只对下载命令增加
`--proxy http://127.0.0.1:7890`。不要把代理写入仓库。

检查静态 peer：

```bash
grep -n '<address>' ~/go2_rviz/config/fastdds/wsl2_mirrored.xml
```

输出必须同时包含 `192.168.151.213` 和 `192.168.151.145`。

## 日常完整启动

### 0. 启动前检查

- 机器狗已经正常站立并保持静止；禁止在趴下或起身过程中启动二维规划。
- 官方遥控器安全员已经就位，周边留有低速测试空间。
- 不运行 RL `/lowcmd` 控制器。
- 不启动 `utree_go2_sdk2_bridge`。
- 不运行旧的 Discovery Server。
- 不在 SLAM、规划或运动期间执行 Git pull、push 或日志 upload。

Jetson 检查残留进程：

```bash
pipeline_process_status=0
pipeline_processes="$(pgrep -af \
  'hesai_ros_driver|go2_imu_bridge|super_lio|utree_dog_navigation|go2_sdk2_bridge|rl_controller|rviz2|fastdds.*discovery' \
  2>&1)" || pipeline_process_status=$?

case "${pipeline_process_status}" in
  0) echo "ERROR: pipeline processes are still running"; echo "${pipeline_processes}" ;;
  1) echo "OK: no pipeline process remains" ;;
  *) echo "ERROR: process query failed (${pipeline_process_status})"; echo "${pipeline_processes}" ;;
esac
```

该检查不会退出当前终端。只有输出 `OK` 才继续；如果输出旧进程，先回到其原终端按
`Ctrl+C`，不要重复启动第二套节点；查询本身失败也必须先处理，不能当作没有进程。

### 1. Jetson 终端 1：启动无界面 SLAM

当前已实机验证的 XT-16 二维平地流程使用 `dense=true`。该开关只选择 Super-LIO 发布给规划与
可视化的完整去畸变帧，本身不构成运动授权；每次启动仍必须检查新鲜里程计、三维确认障碍、
二维膨胀层和实际 `/body_path`，最后由操作员单独 arm SDK2 bridge。日常启动使用：

```bash
cd ~/catkin_ws
GO2_LIO_DENSE_OUTPUT=true ./shell/start_slam.sh
```

保持该终端运行。该脚本启动的 Super-LIO 固定使用 `rviz:=false`。正常停止时最后在
这个终端按一次 `Ctrl+C`；脚本会先结束自己的传感器/SLAM 子进程，再自动执行本轮日志的
`stop -> repair -> upload`。只有该脚本自己创建的诊断会话才会被自动收尾；如果规划、
SDK2 bridge、其他机器人进程或 `/lowcmd` publisher 仍在运行，整个自动收尾会被跳过，
活动诊断会话保持采集，待所有机器人进程停止后再人工处理。

#### 首次部署或相关配置变更后的 dense A/B

只有首次部署，或修改 Super-LIO、点云发布、DDS/RViz 负载路径及相关硬件后，才重新执行完整
A/B。第一轮必须保持 Hesai 配置中的 `lio.output.dense: false`，并用显式开关让日志标记基线：

```bash
cd ~/catkin_ws
GO2_LIO_DENSE_OUTPUT=false ./shell/start_slam.sh
```

完成基线连续起点检查后，无论结果是否通过，都停止该轮全部节点和日志会话，再用下面的
命令新开第二轮，让 `/lio/cloud_world` 发布完整去畸变帧：

```bash
cd ~/catkin_ws
GO2_LIO_DENSE_OUTPUT=true ./shell/start_slam.sh
```

该开关只改变 Super-LIO 完成估计后的点云输出，不改变内部配准使用的
`lio.sensor.voxel_fliter_size: 0.3`；`lio.output.pub_step` 必须保持 `1`。完整 XT-16
帧会增加 Jetson 的点云变换、序列化和 DDS 负载，首次 A/B 时先关闭 WSL2 RViz 的
PointCloud 显示，再核对 `/lio/odom` 和 `/lio/cloud_world` 频率。不要同时修改其他参数，
也不要调整坡度、台阶、粗糙度或通行度阈值。

保持该终端运行。正常启动顺序：

```text
/lidar_points is active.
/imu/data is active.
[3/3] Starting Super-LIO...
... KF init done
... Map init done
```

`Map init done` 后在另一个 Jetson 终端确认本轮实际参数；A/B 第一轮 dense 必须为 `False`，
第二轮对照必须为 `True`，当前日常单轮必须为 `True`，`pub_step` 始终必须为 `1`：

```bash
cd ~/catkin_ws
source ./shell/ros2_environment.sh
ros2 param get /super_lio_node lio.output.dense
ros2 param get /super_lio_node lio.output.pub_step
```

`start_slam.sh` 固定使用 `rviz:=false`。不要在 Jetson 启动 Super-LIO 自带 RViz。
脚本会自动准备 ROS/Fast DDS 环境，并拒绝在检测到 RL 控制器进程或 `/lowcmd`
publisher 时启动。日常命令不需要在外层 source ROS 或重复 export 环境变量。

### 2. Jetson 终端 2：启动无界面规划

等待 SLAM 出现 `Map init done`，确认机器狗仍然站立、静止，再新开 Jetson 终端。
当前二维日常流程必须显式使用 `flat_obstacle`，不要省略模式与平地确认：

```bash
cd ~/catkin_ws
GO2_PLANNING_MODE=flat_obstacle \
GO2_FLAT_GROUND_CONFIRMED=true \
GO2_MAP_CAPTURE=false \
PLANNING_RVIZ=false \
./shell/start_navigation.sh
```

#### 可选：保存实际参与二维规划的三维障碍地图

`flat_obstacle` 模式在投影为二维障碍层之前维护经过裁剪、多帧确认和三维射线清除的体素地图。
需要对照真实场景检查预处理效果时，可临时打开 PCD 记录器：

```bash
cd ~/catkin_ws
GO2_PLANNING_MODE=flat_obstacle \
GO2_FLAT_GROUND_CONFIRMED=true \
GO2_MAP_CAPTURE=true \
PLANNING_RVIZ=false \
./shell/start_navigation.sh
```

记录功能默认关闭。开启后，它按规划地图的源时间 epoch 和时间戳保存 CloudCompare 可读取的
binary PCD；即使 SLAM 源时钟回退并开启新地图 epoch，也不会把新地图误判成旧帧。默认最多
`120` 张或 `100 MiB`，输出到 `~/go2_map_exports/<session>/`。每个会话还包含：

- `session.json`：代码提交、格式、上限和实际配置文件 SHA-256；
- `navigation_config.yaml`：本轮实际启动配置的副本；
- `manifest.jsonl`：每张 PCD 的时间戳、点数、坐标系和字节数；
- `result.json`：正常停止、达到上限或写盘失败的最终状态。

零障碍地图也会保存为合法的零点 PCD，以便识别过滤过度。写盘失败会使记录器非零退出，
但不会伪装成正常完成。录制会增加 Jetson 的磁盘和 DDS 负载，只在诊断时打开；迭代结束后
恢复默认的 `GO2_MAP_CAPTURE=false`。

只有在运动、SDK2 bridge、规划和 SLAM 全部停止后才导出。先在 Jetson 找到最新会话并生成校验：

```bash
map_session="$(find "$HOME/go2_map_exports" -mindepth 1 -maxdepth 1 -type d \
  -printf '%T@ %p\n' | sort -nr | head -n 1 | cut -d' ' -f2-)"
test -n "$map_session" || exit 1
find "$map_session" -maxdepth 1 -type f -name '*.pcd' -print0 \
  | sort -z | xargs -0 -r sha256sum > "$map_session/SHA256SUMS"
printf 'map_session=%s\n' "$map_session"
```

再从 Windows PowerShell 下载到大数据目录：

```powershell
New-Item -ItemType Directory -Force D:\GO2_Data\maps
scp -r unitree@192.168.151.213:/home/unitree/go2_map_exports/<session> `
  D:\GO2_Data\maps\
```

这些 PCD 及其临时文件不得加入主仓库或 `G02_log`，也不得通过 `go2-log upload` 上传。

该脚本默认 `PLANNING_RVIZ=false`。只有临时改回 Jetson 本地可视化时才使用
`PLANNING_RVIZ=true ./shell/start_navigation.sh`；分布式日常流程不要设置它。

`verified-flat-start` 同样默认关闭。只有后续重新研究 terrain 近场盲环，且机器人正常站立、
保持静止并执行无运动验证时，才可显式启动该规划私有的近场补全与 Jetson 本地规划 RViz：

```bash
cd ~/catkin_ws
GO2_VERIFIED_FLAT_START=true PLANNING_RVIZ=true ./shell/start_navigation.sh
```

执行该命令前必须关闭 WSL2 RViz，避免同时运行两个 RViz。该命令不会启动或授权
`utree_go2_sdk2_bridge`；SDK2 bridge 与 RL controller 必须继续保持关闭。分布式验证仍使用
`PLANNING_RVIZ=false` 和唯一的 WSL2 RViz，只显式设置 `GO2_VERIFIED_FLAT_START=true`。

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

规划层按传感器时间戳而不是消息到达时间判断输入是否可用。默认运行契约如下：

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `terrain_mapper.resolution` | `0.20 m` | 真实地形地图每个栅格单元的边长 |
| `body_lattice_planner.motion_step` | `0.20 m` | lattice 单次运动 primitive 的步长 |
| `map_frame` | `world` | 地图、目标和里程计必须使用的世界坐标系 |
| `body_frame` | `base_link` | `/lio/body_odom.child_frame_id` 必须使用的机身坐标系 |
| `max_map_age` | `1.0 s` | 允许规划使用的最大地形图年龄 |
| `max_odom_age` | `0.5 s` | 允许规划使用的最大机身里程计年龄 |
| `timestamp_future_tolerance` | `0.2 s` | 跨主机时钟误差的最大未来容差 |
| `input_watchdog_rate` | `10 Hz` | 没有新回调时检查活动路径的频率 |
| `cloud_stale_warning_age` | `1.0 s` | mapper 对点云 source stamp 或接收间隔发出诊断警告的阈值 |
| `verified_flat_start.enabled` | `false` | 仅显式无运动验证时启用本次 plan 私有的起点近场补全 |

`resolution` 和 `motion_step` 是两个独立参数；当前值恰好同为 `0.20 m`，不能用修改其中一个
代替另一个。watchdog 周期还必须满足
`1 / input_watchdog_rate <= min(max_map_age, max_odom_age)`；默认周期 `0.1 s` 小于
`0.5 s` 的最严格输入年龄。违反该关系时规划节点会拒绝启动，而不是降低安全性继续运行。

这些值是安全边界，不应通过放宽它们掩盖 SLAM 或 DDS 停更。mapper 的点云警告只用于区分
“没有收到新 cloud”和“仍收到带旧 source stamp 的积压 cloud”；它不会伪造新时间戳。
规划器在开始搜索、搜索过程中和发布前检查输入。任何 stale、future、frame mismatch、畸形地图
或规划失败都会撤销已经发布的活动路径。

规划节点启动后，在另一个 Jetson 终端核对运行中的参数，而不是只查看源码 YAML：

```bash
cd ~/catkin_ws
source ./shell/ros2_environment.sh

ros2 param get /terrain_mapper resolution
ros2 param get /body_lattice_planner motion_step
ros2 param get /body_lattice_planner max_map_age
ros2 param get /body_lattice_planner max_odom_age
ros2 param get /body_lattice_planner input_watchdog_rate
```

应依次得到 `0.2`、`0.2`、`1.0`、`0.5` 和 `10.0`。节点不存在、参数读取失败或数值不一致时，
停止本轮并检查 pull、build 和启动输出，不要继续发目标。

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
rviz2 -d "$HOME/go2_rviz/rviz/flat_obstacle_navigation.rviz"
```

RViz 配置固定使用：

- Fixed Frame：`world`
- `/lio/cloud_world`：Best Effort、Volatile、Decay Time `10` 秒，显示默认关闭
- `/flat_obstacle_filtered_map_3d`：Reliable、Volatile，红色三维确认障碍，默认开启
- `/flat_obstacle_filtered_points`：Best Effort、Volatile，橙色当前帧过滤结果，默认关闭
- `/flat_obstacle_inflated`：Reliable、Transient Local，紫色二维膨胀安全区，默认开启
- `/flat_obstacle_raw`：Reliable、Transient Local，未膨胀二维障碍，默认关闭
- `/lio/body_odom`
- `/body_path`：Reliable、Transient Local
- Set Goal：发布 `/goal_pose`

红色点不是另一套独立建图：它们来自 Super-LIO 世界点云，经高度/范围裁剪、多帧确认和
世界系三维清除后形成；紫色单元是这些确认障碍投影到二维后按 Go2 footprint 膨胀的结果。
`GO2_MAP_CAPTURE` 只控制 PCD 诊断写盘，不参与障碍判断，因此开关它不应改变 RViz 地图或路径。

WSLg 可能输出一条 GLSL sampler 警告。只要 RViz 进程仍存活且点云可见，该警告
与 DDS 无关；如果仅代价地图贴图异常，再单独排查 WSLg 渲染。

#### RViz PointCloud 负载复测

`Super-LIO World Cloud` 现在默认关闭。2026-08-07 的 OFF -> ON -> OFF 静止对照中，手动开启该
显示会让 Jetson `/lio/odom` 从约 `8 Hz` 立即下降到约 `4 Hz`，再次关闭后恢复到约 `8 Hz`。
因此日常规划不要打开世界点云；`/lio/body_odom`、红色三维确认障碍、紫色二维膨胀层和
`/body_path` 保持显示即可。

首次 dense A/B 必须关闭 `Super-LIO World Cloud` 显示。选定 dense 模式并通过平地起点门禁后，
如果要判断 WSL2 RViz 点云显示是否加重 Jetson/DDS 负载，使用三个完全独立的静止会话：

1. `PointCloud OFF`；
2. `PointCloud ON`；
3. `PointCloud OFF recovery`。

三轮只切换 RViz 中 `Super-LIO World Cloud` 的 `Enabled` 复选框。不要修改 Best Effort、
Volatile、Depth `5` 或 Decay Time `10`，不要发目标，也不要启动 SDK2 bridge 或 RL controller。
每轮保持相同代码、dense 模式、机器人姿态和场景，至少覆盖三个日志采集周期。每轮结束后先停止
规划，再在 SLAM 终端按一次 `Ctrl+C` 并等待该轮自动日志收尾；只有自动收尾被关闭、复用已有
会话或终端明确报告跳过/失败时才人工执行 stop/repair/upload。机器人节点运行期间不要执行 Git
或日志上传。

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
  --qos-reliability reliable --qos-durability volatile \
  /flat_obstacle_filtered_map_3d sensor_msgs/msg/PointCloud2 --field header

timeout 15 ros2 topic echo --no-daemon --once \
  --qos-reliability reliable --qos-durability transient_local \
  /flat_obstacle_inflated nav_msgs/msg/GridCells --field header
```

上述消息的 `frame_id` 都应为 `world`，且 `/lio/cloud_world` 的 `width` 必须大于
`0`。二维模式仍发布 `/terrain_map` 作为 mapper 到 planner 的内部输入；WSL2 无需安装
自定义消息插件来显示它，操作员使用上面的标准 PointCloud2/GridCells 图层。规划节点全部
启动后若仍看到 `Unknown topic '/terrain_map'`，应检查 mapper 状态和 DDS 发现，不能当作正常现象。
这些检查同时验证发现、payload 传输和非空点云，而不只是看到话题名。

### 5. 只验证规划，不执行运动

当前 `flat_obstacle` 日常流程以三维确认障碍层、二维膨胀层和非空 `/body_path` 为
直接操作门禁；内部 `/terrain_map` 仍供 planner 和诊断器使用，但 terrain 拓扑统计不能
替代二维障碍层的现场核对。先确认以下三个 topic 都有发布者，再从 WSL2 RViz 发一个
近距离目标并检查非空路径：

```bash
cd ~/catkin_ws
source ./shell/ros2_environment.sh

ros2 topic info /flat_obstacle_filtered_map_3d
ros2 topic info /flat_obstacle_inflated
ros2 topic info /body_path

timeout 15 ros2 topic echo --no-daemon --once \
  --qos-reliability reliable --qos-durability transient_local \
  /body_path nav_msgs/msg/Path --field poses
```

当前 `flat_obstacle` 日常流程不再运行 10 组 `planner-series`，也不启用
`verified-flat-start`。出现路径或地图异常时，只做一次有针对性的 `planner-check` 留证；
该诊断结果用于定位内部 `/terrain_map`，不能替代 RViz 中三维障碍、二维膨胀层和实际路径的
现场核对，也不能单独批准运动。后面的 10 组 A/B 只保留为 terrain 阶段的历史复现实验。

设置目标前，确保机器人静止并准备好遥控器急停。不要启动 SDK2 bridge 或 RL controller；
下面的 `planner-check` 会自动检查相关进程、ROS 图和 `/lowcmd` 发布者，无法明确确认安全
状态时直接拒绝采集。

Jetson 和 WSL2 必须使用同步的系统时钟且 `use_sim_time` 设置一致，否则跨主机发布的
目标可能被检查器标记为 `goal_stale` 或 `goal_stamp_from_future`。运行中的
`body_lattice_planner` 自身也会检查 map/odom 时间新鲜度、未来时间戳、map/odom/goal frame
和 odom child frame；检查器从运行节点读取同一组 freshness 参数，不能用本地默认值替代。

发目标前，先在 Jetson 检查终端执行：

```bash
date --iso-8601=ns
ros2 param get /terrain_mapper use_sim_time
ros2 param get /body_lattice_planner use_sim_time

timeout 10 ros2 topic delay --window 50 /lio/body_odom
echo "jetson_delay_exit=$?"
```

再新开一个 WSL2 终端，按上面的 RViz 步骤加载 Humble 和同一 Fast DDS 环境后立即执行：

```bash
date --iso-8601=ns
ros2 param get /rviz2 use_sim_time

timeout 10 ros2 topic delay --window 50 /lio/body_odom
echo "wsl_delay_exit=$?"
```

两端 `date` 只用于排除日期、时区或秒级明显偏移，不能单独证明满足 `0.2 s` 容差。三个
`use_sim_time` 都应为 `False`。两端的 `ros2 topic delay` 都检查 `/lio/body_odom`；Jetson
结果给出本机基线，WSL2 结果还包含跨主机 clock skew 和传输延迟。收到样本后被 `timeout`
结束时退出 `124` 是正常的，但必须实际打印 delay 统计，不得出现低于 `-0.2 s` 的负延迟，
正延迟应保持在 `0.5 s` odom 新鲜度预算内。没有样本或越界时停止在这里。即使预检通过，
运行中的 planner 仍会对每个 RViz 目标执行同样的 timestamp 检查；出现 `goal_stale` 或
`goal_stamp_from_future` 日志、或没有生成新鲜路径时都不启动运动。只有异常定位时才额外运行
一次 `planner-check`，它不是二维日常流程必须重复的第二套门禁。

`start_slam.sh` 已启动诊断会话。在第三个 Jetson 终端使用统一日志入口，不要直接运行
底层 Python 检查器：

```bash
cd ~/catkin_ws
./tools/go2-log status
```

需要定位 terrain 输入或二维模式的内部地图时，`planner-check` 会自动加载项目 ROS/Fast DDS
环境，确认 SDK2/RL 控制进程未运行且
`/lowcmd` 无发布者，并从运行中的 mapper/planner 读取 mapper 与 planner 各自的坡度阈值、
观测帧数、粗糙度、通行度、台阶高度和吸附半径。任一安全检查、ROS 图查询或参数读取失败
都会 fail closed，不会用默认值伪造结果。检查结果原子追加到当前会话的
`planner_input_inspections.jsonl`；诊断结论返回 `2` 也会正常写入，不代表程序崩溃。

只检查当前起点、不等待 RViz 目标时运行：

```bash
./tools/go2-log planner-check --no-goal
```

当前二维流程若只需确认输入是否存在，可运行上面这一次 `--no-goal` 检查；不要因此再扩展为
10 组采样。只有将来重新进入 terrain/verified-flat-start 研究并需要复现旧基线时，才使用
下面的连续起点与 dense A/B 实验。

平地门禁只允许机器人四足正常承重站立、机身处于标称工作高度并保持静止。趴伏、坐姿、起身或
姿态切换期间会改变雷达离地高度、机身遮挡和近场地面几何，采集结果无效，不能用于判断规划器
是否通过。SDK2 bridge 和 RL controller 在整个检查期间必须保持关闭。

#### Terrain 历史诊断：站立 XT-16 近场盲环与 verified-flat-start

本小节不是当前 `flat_obstacle` 的启动步骤，也不是二维运动门禁；日常运行直接跳到本节后面的
一次目标检查。这里保留旧 terrain 输入为何失败以及如何复现实验的证据，供后续接入 Go2 下装
雷达并切回 terrain 模式时使用。

会话 `20260807T092515Z-unitree-jetson-payload-3123` 在 RViz 世界点云关闭、机器人站立静止时，
连续 10 次得到 `start_has_no_valid_cell_in_snap_radius`。`0.55 m` 起点圆内的 24 个格全部为
`observation_count=0`，最近原始观测约为 `0.86-0.95 m`，最近 planner-valid 格稳定在约
`1.21 m`。因此本轮失败发生在坡度、粗糙度或通行度门槛之前，不能通过放宽这些阈值解决。

terrain mapper 的 self-filter 曾直接在 `world` 轴上应用 length/width，机器人转向后会错误交换
机身纵横向过滤范围。现在 mapper 先校验 `/lio/body_odom` 四元数并保存 body yaw，再把点相对
机器人位置的 `dx/dy` 旋转到 body 坐标后应用 self-filter。该修复消除了随 yaw 变化的错误过滤，
但不会消除 XT-16 由安装高度、垂直视场和机身遮挡形成的物理近场盲环。

`verified-flat-start` 是严格受限的起点兜底，不是通用未知区规划：

- 默认关闭，而且只在普通 `start_snap_radius` 找不到实测有效起点时评估；
- overlay 只属于当前一次 `plan()`，不会写回或发布为 `/terrain_map`；
- 只补全当前起点 `fill_radius` 内 observation、高程、坡度和通行度全部 unknown 的格；
- 只使用外部实测 planner-valid 环带，必须同时通过最少观测数、扇区覆盖、高程范围、平面坡度、
  RMSE 和最大残差检查；默认 profile 要求 8 个扇区中至少 7 个有支撑；
- Python 诊断先用四邻接与 step-height 做有界拓扑检查，要求 inferred 连通区接入至少一个
  实测支撑格；C++ planner 随后独立使用真实 motion primitives 检查 inferred 起点能否接入
  实测 planner-valid 格。两层检查都不通过时拒绝，且 Python 拓扑检查不等同于 A* 可达性；
- goal 始终只吸附实测 planner-valid 格，永不使用 inferred 格；
- 搜索从 inferred 起点前缀进入第一个实测有效格后，不能再次返回 inferred 区域。

即使这些检查全部通过，盲环中的真实障碍仍未被传感器观测。该功能当前只批准静止、短距离、
无运动的规划链路验证；不能据此启动 SDK2 bridge，也不能把生成的 `/body_path` 当作运动批准。

历史复现实验命令如下，不要在当前二维日常启动中执行：

```bash
./tools/go2-log planner-series --no-goal --samples 10 --interval 1.0
```

该命令顺序执行 10 次完整安全检查，每次只把有界摘要写入当前诊断会话，不保存完整
`TerrainGrid` 数组。单帧诊断返回 `2` 时继续采集并最终返回 `2`；采集或安全检查返回
`1` 时立即停止。比较 `GO2_LIO_DENSE_OUTPUT=false/true` 时，除该开关外保持场景、
机器人姿态、`pub_step=1` 和规划配置完全一致。

重新研究 terrain 模式时，该历史起点门禁必须同时满足以下条件，缺一项都不要发目标：

- `planner-series` 最终退出码为 `0`；
- 普通模式的 10 个样本全部为 `diagnosis=start_ready_waiting_for_goal`；显式启用
  verified-flat-start 时，普通实测起点仍可返回该诊断并报告 `status=not_needed`。只有普通吸附
  失败且兜底实际生效的样本才应返回
  `diagnosis=start_ready_with_verified_flat_start_waiting_for_goal`，并报告
  `verified_flat_start.status=applied` 与 `exact_start_inferred=true`；
- 每个样本都为 `snapped=true`、`valid_in_snap_radius>=1` 且
  `start_component_cells>0`；
- 每个 `status=applied` 样本还必须满足
  `verified_flat_start.connected_support_cells>=1`；
- 没有 capture error、frame mismatch、stale 或 timestamp future 诊断。

仅在重新研究 terrain 模式时，首次/变更后 A/B 的 `dense=false` 和 `dense=true` 两轮都必须
采集。第一轮结束后，先在规划终端
按 `Ctrl+C`，再在 SLAM 终端按 `Ctrl+C` 并等待自动日志收尾完成。第二轮
必须创建新的诊断会话；`go2-log` 会拒绝在同一活动会话内切换 dense 模式，避免两组证据
混在一起。所有机器人进程停止前不要上传日志。

terrain A/B 完成两轮后，只能选择通过上述门禁的模式进入目标验证。两轮都通过时优先使用
`dense=false`，因为它的 Jetson 与 DDS 负载更低；只有 `dense=true` 通过时才使用完整帧。
两轮都未通过时停止在这里，关闭节点并上传两场日志，不要发目标。若选定模式不是当前
正在运行的第二轮模式，先正常停止第二轮和日志会话，再用选定模式新开一轮，并重新通过
一次 `planner-series` 门禁。

在该 terrain 历史实验中，`dense=true` 是当时的点云输出候选，但最新站立会话的普通
`0.55 m` 起点门禁为 10/10 失败。terrain 模式必须以对应会话的 `planner-series` 结果为准；
只有显式 verified-flat-start 门禁通过时，才能继续该轮静止、短距离、无运动目标验证。如果
后续 A/B 结果改变，必须保留独立日志并更新本节，不能在运行中的会话里临时切换模式。

当前二维模式出现异常时只执行下面的一次目标检查；terrain 研究则要先通过上面的专用门禁：

```bash
./tools/go2-log planner-check --goal-timeout 60
```

`planner-check` 会先创建目标订阅，并等待最多 10 秒确认兼容的 `/goal_pose` 发布者；只有
终端实时出现 `Goal listener ready` 后，60 秒目标窗口才开始计时。若发布者发现或 RViz
目标等待超时，它会重新抓取一组新鲜的 `/terrain_map` 和 `/lio/body_odom`，记录
`goal_publisher_discovery_timeout` 或 `goal_wait_timeout` 以及原始的
`start_map_diagnosis`，并以诊断返回码 `2` 结束。这样日志仍包含全图和起点附近各拒绝层
统计；该兜底流程不会发布目标、路径或任何运动命令。

等待终端实时出现 `Goal listener ready with N publisher(s)` 后，再在 WSL2 RViz 中使用
Set Goal 设置 `0.5-1.0 m` 内、平地、完全可见的目标。检查器会先释放初始大地图和
里程计订阅，再创建 `/goal_pose` 订阅并确认兼容发布者，随后才开始目标倒计时；收到目标
后再分别抓取一条比各自初始样本更新的 `/terrain_map` 和 `/lio/body_odom`。两条消息并非
时间同步的数据对。检查器不会发布目标、路径或运动命令。
启用 verified-flat-start 也不会改变 goal 规则：目标仍必须落在实测 planner-valid 区域，
不会吸附到任何 inferred 起点格。

检查器会分别统计整张地图、起点/目标吸附方框以及规划器实际使用的欧氏圆半径。吸附
结论只使用圆半径内的候选；方框只保留为诊断上下文。`observation_below_min`
较多表示单元格在积分窗口内被观测的帧数不足；`elevation_known` 明显多于
`features_known` 表示已有高程，但 `0.20 m` 网格在 X 或 Y 方向缺少可计算地形特征的邻格；
`slope_over` 是 planner 坡度门槛，`mapper_slope_at_or_above` 和 `roughness_over`
是 mapper 生成通行度时的门槛，`traversability_low` 是 planner 通行度门槛。
`hard_reject_candidate` 只表示坡度和粗糙度均严格低于 mapper 阈值、但通行度仍为零，通常指向
高度差或单格垂直跨度硬拒绝；现有 `TerrainGrid` 无法继续区分这两种原因。
这些计数是可重叠的层统计，不是互斥分类。补洞逻辑也可能让观测帧数不足的格子拥有高程，
因此不要把观测、高程和地形特征计数当成严格单调漏斗，更不要仅凭一个计数修改安全阈值。

新日志中的 `start_has_no_valid_cell_in_snap_radius`、
`goal_has_no_valid_cell_in_snap_radius` 和
`start_and_goal_continuous_ground_disconnected` 分别用于区分起点无有效格、目标无有效格和
连续地面不连通；`*_frame_mismatch`、`*_stale` 或 `*_stamp_from_future` 表示坐标契约或
时间新鲜度不满足，`*_elevation_invalid_for_ground_topology` 表示吸附格缺少有效高程。
旧日志可能保留名称 `*_snap_square`。新检查器与 C++ 规划器均从原始端点世界坐标到候选
格中心计算真实欧氏距离，避免机器人跨过栅格边界时仅因整数格偏移而改变结论。XT-16 平地
配置只对当前机器人起点使用 `start_snap_radius=0.55 m`，RViz 目标仍使用
`snap_radius=0.50 m`；该修改没有放宽坡度、粗糙度、通行度或台阶阈值。
此操作只应让规划节点生成 `/body_path`，不会自动控制机器人，因为 SDK2 bridge 尚未启动。

退出码 `0` 只表示起点检查通过，或起终点属于同一连续地面区域；其他诊断结论返回 `2`，
采集/消息错误返回 `1`。使用 `--json` 时仍应读取 `diagnosis`；即使退出码为 `0`，也不代表
可以执行运动。

检查路径：

```bash
ros2 topic info -v /body_path
timeout 10 ros2 topic echo --no-daemon --once \
  --qos-reliability reliable --qos-durability transient_local \
  /body_path nav_msgs/msg/Path --field poses
echo "path_echo_exit=$?"
```

不要只检查 `header`：非空 `poses` 才表示当前存在活动路径；空 `poses` 表示规划器因输入过期、
坐标系不匹配或规划失败而撤销了 transient-local 旧路径。如果该进程启动后从未成功规划，
`/body_path` 可以完全没有消息，此时 `timeout` 返回 `124` 是正常结果。有效路径的
`header.stamp` 使用 map/odom 中较旧的因果时间戳，而不是规划完成时的 `now()`；路径 frame
必须为 `world`。

检查器报告
`same_continuous_ground_component_not_planner_approval` 只说明起点和目标位于同一个
四邻域有效地形区域，且相邻格高程有限、台阶高度未超限。这个拓扑结果不复现
规划器的航向格和 `0.20 m` motion primitive，也不检查 footprint 或整段碰撞，因此既不
保证规划成功，也不是运动安全批准。当前二维低速运动只能在本 README 顶部记录的阶段门禁
已经通过、并再次满足下一节的现场检查与显式 arm 后进行；不能仅凭该拓扑结论运动。

### 6. Jetson 终端 3/4：启动并一次授权 SDK2 bridge

只有在 RViz 障碍层与测试目标路径均符合现场、官方遥控器安全员已就位且没有
`/lowcmd` 发布者时才进入运动。Jetson 终端 3 启动 bridge：

```bash
cd ~/catkin_ws
./shell/start_sdk2_bridge.sh
```

保持终端 3 运行。bridge 每次进程启动后都默认为 disarmed；不能通过参数绕过人工授权。
新开 Jetson 终端 4，必须先加载 ROS 2 环境，再显式 arm 一次：

```bash
cd ~/catkin_ws
source ./shell/ros2_environment.sh

ros2 service call /go2_sdk2_bridge/enable_motion \
  std_srvs/srv/SetBool '{data: true}'
```

授权可以发生在新路径到达前；此时 bridge 停车并等待。此后每次在 RViz 发布一个新的有效
目标，规划器生成新鲜 `/body_path` 后会自动执行，不需要为每个目标重复调用 `data: true`。
正常到达目标或收到规划器显式发布的新鲜空路径时，bridge 会停车但保留本次授权并等待
下一条新路径。路径缓存超时、里程计超时或其他安全故障会立即停车并解除授权，排除原因后
必须重新显式 arm，不能让中断前的旧目标自动恢复。
配置参数 `enabled` 是只读的启动保护并始终为 `false`，不代表运行期 armed 状态；以 service
响应和 bridge 终端中的 `armed`/`waiting for a path` 日志为准。

标准 YAML 的速度上限为 `vx=0.60 m/s`、`vy=0.35 m/s`、
`yaw_rate=0.80 rad/s`。launch 默认只加载 YAML，不再重复覆盖这些值；启动脚本会逐项打印
最终值及来源。需要为单次运行调整时，只能在启动前显式设置 `GO2_MAX_VX`、
`GO2_MAX_VY` 或 `GO2_MAX_YAW_RATE`，未设置的项继续采用 YAML。节点运行后这些参数保持只读。
由于 `max_vx` 同时限制前进和后退，配置能力边界取 SDK 较小的反向极限 `2.5 m/s`；
`max_vy` 和 `max_yaw_rate` 的能力边界分别为 `1.0 m/s` 与 `4.0 rad/s`。这些边界只是拒绝
非法配置的 SDK 接口范围，不是建议现场直接使用的速度。
局部航向误差达到 `45 deg` 时进入只转不平移，降到 `15 deg` 以内才恢复平移，避免阈值附近
反复切换。

里程计缺失/过期、非法或异常时间戳路径、SDK2 停车失败、检测到 `/lowcmd` 发布者，或人工
调用 `data: false` 都会 disarm；处理原因后必须重新显式授权。任何时候都可人工禁用并停车：

```bash
cd ~/catkin_ws
source ./shell/ros2_environment.sh

ros2 service call /go2_sdk2_bridge/enable_motion \
  std_srvs/srv/SetBool '{data: false}'
```

二维规划优先让机头朝向局部路径段：长距离反向/横向运动会承担持续航向代价，直角转折在
真正到达拐点后使用下一段航向；运动桥在航向误差较大时只转向，进入对齐范围后才恢复平移。
bridge 对每条新路径维护有界、单调的进度，不会因 U 形回折段离机器人更近就跳到未来段；
偏离当前段或原地转向点超过 `0.05 m`、或产生非规划的负 `vx` 时会停车并解除授权。多级
原地转向按规划姿态逐级执行。这里跟随局部路径切线而不是始终朝最终目标，因此不会破坏
必须先前进再转弯的绕障路径；单个栅格的小幅全向修正仍保留。不要通过修改
`world -> imu -> base_link` 的 `-90 deg` 关系调这个行为。

到达终点后，bridge 会锁存本轮原始 `/goal_pose` 的时间戳 generation；规划器因地图更新对
同一轮目标重复规划或重新吸附到相邻格时仍不会再次启动。操作员在 RViz 新发的目标具有新的
generation，即使几何终点与上次完全相同，也会在保持 armed 的情况下作为新任务自动执行，
不需要先 `data: false` 再 `data: true`。

### 7. 停止顺序

1. 如果 SDK2 bridge 曾启用，先调用 `enable_motion` 的 `false` 分支并确认停车。
2. 在 SDK2 bridge 终端按 `Ctrl+C`，等待执行器退出。
3. 在 Jetson 规划终端按 `Ctrl+C`，等待规划节点退出；若设置了 `GO2_MAP_CAPTURE=true`，
   还要等 PCD recorder 写完 `result.json`。
4. 在 Jetson SLAM 终端只按一次 `Ctrl+C`。只有另行启用了 Super-LIO 的
   `lio.map.save_map=true` 时才等待 `Final map saved to:`；无论是否保存 SLAM 地图，都要
   等待诊断 stop/repair/upload 结果。
5. 关闭 WSL2 RViz。
6. 确认没有残留机器人进程。正常情况下无需再次手动处理日志；如果自动收尾被跳过或失败，
   会话会被保留，再按提示手动重试。

```bash
pipeline_process_status=0
pipeline_processes="$(pgrep -af \
  'hesai_ros_driver|go2_imu_bridge|super_lio|utree_dog_navigation|go2_sdk2_bridge|rl_controller' \
  2>&1)" || pipeline_process_status=$?

case "${pipeline_process_status}" in
  0) echo "ERROR: pipeline processes are still running"; echo "${pipeline_processes}" ;;
  1) echo "OK: no pipeline process remains" ;;
  *) echo "ERROR: process query failed (${pipeline_process_status})"; echo "${pipeline_processes}" ;;
esac

./tools/go2-log status
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
| `GO2_LIO_DENSE_OUTPUT` | 脚本默认 `false` | 当前二维日常流程显式设为 `true`；只有相关感知配置变更或重做 terrain 实验时才运行 false/true A/B |
| `GO2_LOG_AUTO_FINALIZE` | `true` | `start_slam.sh` 正常退出时自动收尾并上传它自己创建的诊断会话；设为 `false` 时改为人工处理 |
| `SLAM_LOG_DIR` | `~/slam_logs` | Hesai 和 IMU bridge 日志目录 |
| `UNITREE_SDK_LIBRARY_DIR` | `/usr/local/lib` | Unitree SDK 配套 DDS 动态库目录 |
| `GO2_SDK2_BRIDGE_CONFIG` | 仓库内 `go2_sdk2_bridge.yaml` | SDK2 bridge 参数文件；标准速度默认值只在该 YAML 中维护 |
| `GO2_MAX_VX` | 未设置 | 仅为本次 bridge 启动显式覆盖 `max_vx`，启动摘要会标明来源 |
| `GO2_MAX_VY` | 未设置 | 仅为本次 bridge 启动显式覆盖 `max_vy`，启动摘要会标明来源 |
| `GO2_MAX_YAW_RATE` | 未设置 | 仅为本次 bridge 启动显式覆盖 `max_yaw_rate`，启动摘要会标明来源 |
| `GO2_BODY_YAW_OFFSET_RAD` | `-1.5707963267948966` | IMU 到 `base_link` 的 yaw 校正 |
| `GO2_LIDAR_OFFSET_X/Y/Z` | `0.171 / 0 / 0.0908` | 同时驱动 XT-16 静态 TF 与三维清除射线原点，仅安装外参复测后修改 |
| `PLANNING_RVIZ` | `false` | 仅显式设为 `true` 时在 Jetson 启动规划 RViz |
| `GO2_VERIFIED_FLAT_START` | `false` | 仅显式设为 `true` 时启用当前 plan 私有的 verified-flat-start |
| `GO2_PLANNING_MODE` | `terrain` | 平地二维障碍导航显式设为 `flat_obstacle` |
| `GO2_FLAT_GROUND_CONFIRMED` | `false` | 机器狗已站立且平地条件已由操作员确认时显式设为 `true` |
| `GO2_MAP_CAPTURE` | `false` | 临时记录规划前的三维确认体素地图 |
| `GO2_MAP_CAPTURE_DIR` | `~/go2_map_exports` | PCD 会话根目录，启动脚本禁止放在 Git 工作区内 |
| `GO2_MAP_CAPTURE_MAX_SNAPSHOTS` | `120` | 单次记录的最大地图数量 |
| `GO2_MAP_CAPTURE_MAX_MB` | `100` | 单次记录的最大 PCD 总量，单位 MiB |

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

SDK2 bridge 已进入当前二维低速实测流程，但仍是默认 disarmed 的独立安全边界。以下门槛
满足后才可在第三个 Jetson 终端运行启动脚本；它会自动使用与 SLAM/规划相同的 Fast DDS
环境：

- 无头感知加远程 RViz 时，IMU、雷达、里程计和世界点云达到性能基线。
- `world -> imu -> base_link` 方向与实机直行一致，静止漂移通过验收。
- yaw-aware footprint、整段路径碰撞、地图边界和未知区行为已经通过测试。
- 规划失败、输入过期或非法 frame 会发布/处理空路径并清除旧目标。
- `/lowcmd` 互斥、遥控器接管、实体急停和低速 field-test 参数已准备完成。

在这些门槛通过前，不启动 bridge，也不调用 `enable_motion` 的 `true` 分支。二维阶段通过
不代表 terrain、楼梯或更高速度已经获准。

首次实机运动必须有独立遥控器安全员，手始终放在官方接管/急停控制上；不能把 ROS
service 或 DDS 链路当作唯一停车手段。若 disable service 无响应，立即用遥控器接管
或执行实体急停，不等待软件恢复。

```bash
cd ~/catkin_ws
./shell/start_sdk2_bridge.sh
```

脚本会拒绝与 RL `/lowcmd` 控制器并行运行；节点运行期间也会持续检查
`/lowcmd` 发布者，一旦发现便立即禁用 SportClient 并停车。节点启动后仍为 disarmed；确认
`/body_path`、`/lio/body_odom` 和机器人周边安全后，才可显式 arm：

`enabled:=true` 启动配置会被拒绝，不能绕过人工授权。路径、里程计或控制参数包含
非有限值、非法四元数或不一致坐标系时，节点也会保持禁用并尝试停车。

```bash
cd ~/catkin_ws
source ./shell/ros2_environment.sh

ros2 service call /go2_sdk2_bridge/enable_motion \
  std_srvs/srv/SetBool '{data: true}'
```

该调用只需在 bridge 进程启动后执行一次。正常到达、规划器显式空路径或等待新目标时会
停车并保持 armed；后续新 goal generation 的新鲜路径会自动恢复执行，即使操作员再次选择
相同的几何终点也无需重复 arm。路径或里程计超时、非法输入、路径跟踪偏差及其他安全故障会
disarm，必须排除故障后重新调用。

随时禁用并停车：

```bash
cd ~/catkin_ws
source ./shell/ros2_environment.sh

ros2 service call /go2_sdk2_bridge/enable_motion \
  std_srvs/srv/SetBool '{data: false}'
```

如果 SDK2 尚未确认 `StopMove`，禁用服务会返回失败；节点仍保持禁用并按控制周期
持续重试停车，在确认成功前拒绝再次启用。

不要让 `utree_go2_sdk2_bridge` 和发布 `/lowcmd` 的 RL 控制器同时控制机器人。

### 诊断日志

三个启动脚本都会调用仓库内的 `./tools/go2-log start`。该命令是幂等的：同一会话已在采集时
不会重复创建进程。采集内容包括 Git 版本、ROS/DDS 环境、网络、参数、进程状态、
关键小消息和频率摘要以及 `/rosout` 警告/错误；不会复制完整雷达点云、世界点云或
`TerrainGrid`。

每轮还会写入以下有界健康摘要：

- `process_health.csv`：各组件 PID、进程状态、区间 CPU、RSS 和线程数；
- `topic_timing.csv`：`/lio/odom` 与 `/lio/body_odom` 的 5 秒 header/接收时序摘要；
- `system_health.csv`：系统负载、可用内存、swap 和最高可读温度；
- `network_health.csv`：非回环网卡状态、流量、错误和丢包计数；
- `hesai_summary.csv`：Hesai 日志尾部的有界 raw-frame、点数、包数和警告/错误摘要。

标准流程始终从仓库根目录调用当前版本：

```bash
cd ~/catkin_ws
./tools/go2-log status
```

如确实需要简短的全局命令，可以安装副本，但每次 `git pull` 后都必须再次执行安装命令，
不能让新版采集器与旧版 stop/upload 混用：

```bash
cd ~/catkin_ws
sudo install -m 0755 tools/go2-log /usr/local/bin/go2-log
```

正常停止顺序必须是 bridge、规划、最后 SLAM。`start_slam.sh` 在 `Ctrl+C` 后会自动停止、
修复并上传它自己创建的诊断会话；为让上传通过，按 `Ctrl+C` 前必须先确认其他机器人进程
已退出。只有关闭自动收尾、复用了已有会话，或终端明确报告跳过/失败时才手动执行：

```bash
cd ~/catkin_ws
source ./shell/ros2_environment.sh

./tools/go2-log stop
./tools/go2-log repair
GO2_LOG_PROXY=http://192.168.151.145:7890 ./tools/go2-log upload
```

自动收尾用于防止新的会话继续积压，但不会在下一次启动时擅自上传历史会话。旧的 stale
状态或已经达到 `20` 个未上传会话时，`start_slam.sh` 仍会 fail closed；保持所有机器人
进程停止，先按上面的人工流程修复并上传至少一个会话，再重新启动。失败的上传不会删除数据。

如果旧会话因采集器文本文件末尾出现 NUL 字节而被 `upload` 拒绝，保持所有机器人
进程停止，显式修复该会话后再上传：

```bash
cd ~/catkin_ws
./tools/go2-log repair <session-id>
./tools/go2-log upload <session-id>
```

`repair` 只接受“完整 UTF-8 行之后连续到文件末尾”的 NUL 后缀；中间 NUL、未完整
行、符号链接、超限文件或异常修复记录仍会被拒绝。损坏原件会先按字节原样保存到
`~/go2_logs/quarantine/<session-id>/`，修复证据写入会话内的
`repair_manifest.jsonl`。该命令不会放宽正常上传的二进制和大型文件校验。

`upload` 会在发现任一雷达、IMU、SLAM、规划、SDK2 或 RL 控制进程时拒绝 Git
push。每个会话最多 100 MiB，本地最多保留 20 个；只有远端提交验证成功的旧会话
才允许被清理。默认上传到公开仓库 `Aphra-neck/G02_log` 的 `main` 分支，并使用：

```bash
export GO2_LOG_PROXY=http://192.168.151.145:7890
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
  -c http.proxy=http://192.168.151.145:7890 `
  -c https.proxy=http://192.168.151.145:7890 `
  pull --ff-only origin main

# 先替换为本机 GO2_Hesai checkout 的实际路径
Set-Location C:\path\to\GO2_Hesai
python .\tools\analyze_diagnostics.py `
  D:\G02_log\sessions\<session-id>
```

分析器生成 `analysis/report.md`、`analysis/topic_rates_summary.csv`、
`analysis/process_health_summary.csv`、`analysis/sdk2_command_summary.csv`、
`analysis/body_odometry_audit.csv`、`analysis/planner_input_summary.csv` 和
`analysis/planner_input_series_summary.csv`。存在相应原始输入时，还会生成
`analysis/topic_timing_summary.csv`、`analysis/system_health_summary.csv`、
`analysis/network_health_summary.csv` 和 `analysis/hesai_driver_summary.csv`。planner 汇总展开
每次实机规划检查的全图、起点与目标局部层统计；
原始 `planner_input_inspections.jsonl` 保持不变。分析器同时核对实机原始与校正里程计的
时间戳、frame、四元数及 yaw 偏置。PCD、rosbag 和 core dump
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
lio.output.dense: false
lio.output.pub_step: 1
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

两端 profile 都必须同时包含 `192.168.151.213` 与 `192.168.151.145`，且 whitelist
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
Ubuntu 22.04 + Humble，清除 Foxy/Noetic overlay，并重新下载
`ROS2-2D-navigation` 分支固定提交的 `flat_obstacle_navigation.rviz`。

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
