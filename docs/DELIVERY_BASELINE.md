# 交付基线与回退

## 冻结点

| 项目 | 值 |
| --- | --- |
| 实机验证提交 | `fbfb19cd07213185fc4c63fabbd1ef45a8271520` |
| Git 标签 | `go2-stable-fbfb19c` |
| 分支 | `ROS2` |
| 平移速度 | `0.40 m/s` |
| 弧线角速度 | `0.60 rad/s` |
| Move 刷新率 | `200 Hz` |
| 路径前缀长度 | `0.35 m` |
| 前缀采样数 | `8` |
| 折扣 | `0.95` |
| 终点位置容差 | `0.15 m` |

现场评价：该版本除速度仍可进一步研究外，闭环运行稳定；`0.50/0.75` 的实验版本
`20f273e` 过于激进，容易碰撞，不作为交付速度。

## 回退原则

回退标签包含整理前的完整代码，不依赖后续归档目录。任何后续效果下降都先停止机器人、
保持工作区干净，再从稳定标签创建独立恢复分支：

```bash
cd ~/catkin_ws
git fetch --tags origin
git switch -c recovery/fbfb19c go2-stable-fbfb19c

source /opt/ros/humble/setup.bash
MAKEFLAGS=-j2 colcon build \
  --symlink-install \
  --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

不要用 `git reset --hard` 处理含现场配置的工作区。返回日常分支时先确认无本地改动，
再执行 `git switch ROS2`。

## 整理后的行为保证

- 不改变 flat-obstacle 地图、膨胀层、footprint 或 lattice 规划逻辑。
- 不改变标准 SDK2 bridge 的路径近似和 `0.40/0.60` 指令。
- go2-log、上传仓库和历史会话状态不再是启动依赖。
- `deprecated_reference/` 不参与 colcon 构建和 ROS 2 安装。
- 未来日志采集和上传必须由开发者自行维护，并保持为独立的可选系统；规范见
  `docs/LOCAL_DIAGNOSTICS.md`。
