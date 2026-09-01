# 部署与日常运行

本文档是 `ROS2` 分支唯一的现场启动手册。以下命令默认 Jetson 工作空间为
`~/catkin_ws`。RViz 是可选的显示端：可以在 Jetson 本机启动，也可以在需要时使用
WSL2 远程显示，或者完全不启动。

## 1. 拉取与编译

必须先停止正在运行的 SLAM、规划和 SDK2 bridge；如果有 RViz 实例，也先关闭它，再执行：

```bash
cd ~/catkin_ws

test "$(git branch --show-current)" = ROS2 || exit 1
test -z "$(git status --porcelain)" || {
  echo "ERROR: working tree is not clean"
  git status --short
  exit 1
}

git -c http.proxy=http://192.168.151.132:7890 \
  -c https.proxy=http://192.168.151.132:7890 \
  pull --ff-only origin ROS2

source /opt/ros/humble/setup.bash
MAKEFLAGS=-j2 colcon build \
  --symlink-install \
  --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
git log -1 --oneline
```

Git 拉取失败、工作区不干净或编译失败时停止，不要用 `reset --hard` 覆盖现场配置。

## 2. Jetson 终端 1：传感器与 SLAM

```bash
cd ~/catkin_ws
GO2_LIO_DENSE_OUTPUT=true ./shell/start_slam.sh
```

等待 `KF init done` 和 `Map init done`。该脚本只写本地 `~/slam_logs` 进程输出，
不会启动 go2-log，也不会访问或上传任何外部日志仓库。
如需额外诊断，必须按 [可选的本地诊断与日志系统](LOCAL_DIAGNOSTICS.md) 另行建立，且不能作为本启动步骤的依赖。

## 3. Jetson 终端 2：二维地图与规划

```bash
cd ~/catkin_ws
GO2_PLANNING_MODE=flat_obstacle \
GO2_FLAT_GROUND_CONFIRMED=true \
GO2_MAP_CAPTURE=false \
PLANNING_RVIZ=false \
./shell/start_navigation.sh
```

日常交付只使用 `flat_obstacle`。`terrain_mapper_node` 同时承载当前平地障碍地图，
因此它仍是正式运行必需节点，并不代表正在启用旧 terrain 实验模式。
本节命令采用无界面默认模式；需要在 Jetson 本机看图时，将 `PLANNING_RVIZ=false` 改为
`true`，不需要为此另开 WSL2。

## 4. 可选的可视化：Jetson 本机、WSL2 或不启动

可视化不是地图、规划或运动执行的前置条件。`start_navigation.sh` 默认使用
`PLANNING_RVIZ=false`，这只表示不启动 RViz，不会关闭地图、规划器或 `/goal_pose`。
三种模式只能选择一种，不要同时启动 Jetson 本机 RViz 和 WSL2 RViz。

### 4.1 选项 A：直接在 Jetson 本机可视化

Jetson 有图形桌面或有效 `DISPLAY` 时，在 Jetson 的规划终端使用：

```bash
cd ~/catkin_ws
GO2_PLANNING_MODE=flat_obstacle \
GO2_FLAT_GROUND_CONFIRMED=true \
GO2_MAP_CAPTURE=false \
PLANNING_RVIZ=true \
./shell/start_navigation.sh
```

该模式由导航 launch 在 Jetson 本机启动 `rviz2`，使用项目内的
`flat_obstacle_navigation.rviz` 配置，不需要 WSL2、镜像网络或额外的分布式可视化终端。
如果通过无图形 SSH 登录，`DISPLAY` 不可用时不要选此模式。

### 4.2 选项 B：只启动规划，不启动可视化

没有显示需求，或准备使用其他上位机时，在 Jetson 的规划终端使用：

```bash
cd ~/catkin_ws
GO2_PLANNING_MODE=flat_obstacle \
GO2_FLAT_GROUND_CONFIRMED=true \
GO2_MAP_CAPTURE=false \
PLANNING_RVIZ=false \
./shell/start_navigation.sh
```

此时仍会发布 `/flat_obstacle_inflated`、`/body_path` 等正式话题；可以使用命令行或其他 ROS 2
客户端向 `/goal_pose` 发布目标。

### 4.3 选项 C：使用 WSL2 远程 RViz

只有在需要 WSL2 显示端时，才在 Jetson 规划终端选择 `PLANNING_RVIZ=false`，然后在 WSL2
另开终端执行：

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

WSL2 这里只负责显示和发布 RViz 的 `Set Goal`，不是机器狗运行的必要节点。确认
`/lio/body_odom`、`/flat_obstacle_filtered_map_3d`、`/flat_obstacle_inflated` 和
`/body_path` 正常后再进入运动步骤。

## 5. Jetson 终端 3：标准 SDK2 bridge

```bash
cd ~/catkin_ws
./shell/start_sdk2_bridge.sh
```

启动输出必须显示：

```text
Control: direct SportClient::Move refresh at 200.0 Hz
Fixed translation speed: 0.40 m/s
Arc turn: Move(0.40,0,+/-0.60)
```

## 6. Jetson 终端 4：核对并授权

```bash
cd ~/catkin_ws
source ./shell/ros2_environment.sh

ros2 param get /go2_sdk2_bridge command_rate
ros2 param get /go2_sdk2_bridge translation_speed
ros2 param get /go2_sdk2_bridge rotation_speed

ros2 service call /go2_sdk2_bridge/enable_motion \
  std_srvs/srv/SetBool '{data: true}'
```

三个参数应依次为 `200.0`、`0.4`、`0.6`。授权后通过已选择的 Jetson/WSL2 RViz
`Set Goal`，或其他 ROS 2 客户端发布 `/goal_pose`；同一 bridge 进程内的新目标不需要重复授权。

## 7. 停止顺序

先停车并解除授权：

```bash
cd ~/catkin_ws
source ./shell/ros2_environment.sh
ros2 service call /go2_sdk2_bridge/enable_motion \
  std_srvs/srv/SetBool '{data: false}'
```

然后依次：

1. bridge 终端 `Ctrl+C`；
2. 规划终端 `Ctrl+C`；
3. SLAM 终端 `Ctrl+C`；
4. 如果启用了 Jetson 本机 RViz，关闭它；如果启用了 WSL2 RViz，关闭它；未启用可视化时跳过。

不需要执行 go2-log stop、repair 或 upload。当前仓库没有正式日志上传步骤；未来开发者若要上传，
必须使用自己维护的、独立于本运行链的日志系统，并遵守 [可选的本地诊断与日志系统](LOCAL_DIAGNOSTICS.md)。
