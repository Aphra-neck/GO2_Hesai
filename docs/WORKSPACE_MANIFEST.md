# 工作空间保留清单

## 正式运行必须保留

| 路径 | 用途 |
| --- | --- |
| `config/fastdds/` | Jetson 与 WSL2 Fast DDS 静态单播配置 |
| `shell/ros2_environment.sh` | Humble、工作空间和 DDS 环境 |
| `shell/start_slam.sh` | Hesai、Go2 IMU、Super-LIO |
| `shell/start_navigation.sh` | 机身里程计适配、地图和规划 |
| `shell/start_sdk2_bridge.sh` | 标准 `/body_path` SDK2 执行器 |
| `src/HesaiLidar_ROS_2.0/` | XT-16 ROS 2 驱动 |
| `src/go2_imu_bridge/` | Unitree LowState 到 ROS IMU |
| `src/Super-LIO/` | 激光惯性里程计与世界点云 |
| `src/utree_dog_msgs/` | 当前 TerrainGrid 内部消息 |
| `src/utree_dog_navigation/` | flat-obstacle 地图、机身规划与 RViz |
| `src/utree_go2_sdk2_bridge/` | 正式 SDK2 路径执行器 |

`terrain_mapper_node`、`TerrainGrid` 和名称中带 `terrain` 的部分代码仍被当前
flat-obstacle 管线复用，不能仅凭名称删除或移动。

## 开发与交付验证保留

- `tools/test_start_slam_cleanup.sh`
- `tools/test_start_navigation.sh`
- `tools/test_start_sdk2_bridge.sh`
- `tools/test_terrain_navigation_launch.py`
- 各 ROS 2 包的当前单元测试
- `docs/` 中的正式运行、基线和执行原理文档

## 生成物：不提交、不归档

- `build/`、`install/`、`log/`
- `build_codex/`、`install_codex/`、`log_codex/`
- `.pytest_cache/`、`__pycache__/`
- PCD、bag、运行日志和地图导出

这些内容由编译或运行重新生成，不能放入 `deprecated_reference/`。

## 已停用但保留参考

- `deprecated_reference/direct_bridge/`：无避障 direct bridge 与旧 SDK worker；
- `deprecated_reference/go2_log/`：已退役日志、上传和诊断分析系统；
- `deprecated_reference/sdk2_motion_stall_probe/`：历史运动卡住探针；
- `deprecated_reference/historical_research/`：SDK2 调研材料；
- `deprecated_reference/historical_docs/`：整理前的完整 README。

`deprecated_reference/COLCON_IGNORE` 阻止 colcon 发现整个归档树。这些目录不得加入
CMake、launch、启动脚本或日常测试发现路径。
