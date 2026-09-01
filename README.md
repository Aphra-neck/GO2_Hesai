# GO2 Hesai ROS 2 二维导航

本仓库的 `ROS2` 分支是 Unitree Go2 + Hesai XT-16 + ROS 2 Humble 的二维导航交付分支。
当前实机闭环由 Super-LIO、平地障碍地图、机身 lattice 规划器和 Unitree SDK2
`SportClient::Move()` 路径执行器组成。

## 当前稳定基线

- 实机稳定提交：`fbfb19c`
- 稳定标签：`go2-stable-fbfb19c`
- 固定平移速度：`0.40 m/s`
- 固定弧线转向角速度：`0.60 rad/s`
- SDK2 Move 刷新：`200 Hz`
- 终点位置容差：`0.15 m`

这个标签是整理前的完整回退点。后续文档整理、历史归档或日志系统退役不改变上述
地图、规划和运动参数。

## 正式运行链

```text
Hesai XT-16 -> /lidar_points ┐
Go2 LowState -> /imu/data    ├-> Super-LIO -> /lio/odom + /lio/cloud_world
                             └-> body odom adapter -> /lio/body_odom

/lio/cloud_world + /lio/body_odom + /goal_pose
  -> flat-obstacle mapper
  -> /flat_obstacle_filtered_map_3d + /flat_obstacle_inflated
  -> body lattice planner
  -> /body_path
  -> standard SDK2 path bridge
  -> SportClient::Move(0.40, 0, +/-0.60)
```

正式流程只使用：

- `shell/start_slam.sh`
- `shell/start_navigation.sh`
- `shell/start_sdk2_bridge.sh`
- `src/utree_go2_sdk2_bridge/launch/go2_sdk2_bridge.launch.py`

无避障 direct bridge、go2-log、运动卡住探针和历史研究材料已经移入
`deprecated_reference/`，不参与构建、安装、启动或运行。

RViz 不是正式运行链的必需节点。可在 Jetson 本机启动、在 WSL2 中远程启动，或完全不启动；
三种方式只选一种，具体命令见 [部署与日常运行](docs/OPERATIONS.md) 的可视化章节。

## 文档入口

- [部署与日常运行](docs/OPERATIONS.md)
- [交付基线与回退](docs/DELIVERY_BASELINE.md)
- [工作空间保留清单](docs/WORKSPACE_MANIFEST.md)
- [SDK2 路径执行原理](docs/go2_sdk2_motion_bridge_execution.md)
- [可选的本地诊断与日志系统](docs/LOCAL_DIAGNOSTICS.md)
- [停用参考资料说明](deprecated_reference/README.md)

## 目标平台

- Jetson ARM64，Ubuntu 22.04
- ROS 2 Humble
- Unitree SDK2
- Hesai PandarXT-16
- ROS 2 domain `30`，`rmw_fastrtps_cpp`
- Unitree SDK2 domain `0`，网卡默认 `enP8p1s0`

## 最重要的运行约束

- 机器狗必须正常站立，官方遥控器安全员必须在场。
- 不得同时运行 RL `/lowcmd` 控制器和 SDK2 SportClient bridge。
- 每次 bridge 进程启动后必须由操作员显式调用一次 `enable_motion=true`。
- 正常到达目标后 bridge 保持授权并持续发送零速 Move，等待新的 `/goal_pose`。
- 外部日志仓库、上传状态和历史 go2-log 会话不会参与或阻断正式运行链。

如需日志采集或上传，开发者必须自行建立独立的本地诊断系统；不得把日志系统重新接入正式启动链。
建立、校验、远端配置和维护要求见 [可选的本地诊断与日志系统](docs/LOCAL_DIAGNOSTICS.md)。
