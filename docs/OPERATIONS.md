# 部署与日常运行

本文档是 `ROS2` 分支唯一的现场启动手册。以下命令默认 Jetson 工作空间为
`~/catkin_ws`，WSL2 RViz 配置位于 `~/go2_rviz`。

## 1. 拉取与编译

必须先停止 SLAM、规划、SDK2 bridge 和 RViz，再执行：

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

## 4. WSL2：唯一 RViz

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

确认 `/lio/body_odom`、`/flat_obstacle_filtered_map_3d`、
`/flat_obstacle_inflated` 和 `/body_path` 正常后再进入运动步骤。

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

三个参数应依次为 `200.0`、`0.4`、`0.6`。授权后通过 RViz `Set Goal`
发布 `/goal_pose`；同一 bridge 进程内的新目标不需要重复授权。

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
4. 关闭 WSL2 RViz。

不需要执行 go2-log stop、repair 或 upload。
