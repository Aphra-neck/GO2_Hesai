# Go2 SDK2 运动卡住同步探针

这个探针用于回答三个彼此独立的问题：

1. ROS bridge 的 `/sdk2_command` 是否连续，还是发生了发布间隙；
2. 指令连续时，`/lio/body_odom` 和 `rt/sportmodestate` 是否产生运动响应；
3. 是否出现原生遥控器输入与反馈变化的时间重叠，以及偏航响应符号是否与指令一致。

探针是只读工具。它只创建四个订阅，不创建 ROS 或 SDK2 发布器，不创建服务客户端，也不调用
任何运动、停车、遥控器仲裁、姿态或步态接口：

| 数据 | 来源 | 用途 |
| --- | --- | --- |
| 机体系速度指令 | `/sdk2_command` | 判断成功 `Move()` RPC 的诊断流、频率和间隙 |
| 世界系机体位姿 | `/lio/body_odom` | 计算实际平移和偏航变化 |
| 宇树高层运动状态 | `rt/sportmodestate` | 读取运动状态码、估计速度和偏航速度 |
| 原生遥控器数据 | `rt/lowstate.wireless_remote` | 定位拨杆开始、结束和峰值 |

C++ SDK2 读取器使用 `std::chrono::steady_clock`，Python ROS 订阅器使用
`time.monotonic_ns()`。在同一台 Linux Jetson 上，两者都以系统单调时钟为基准；合并后的
`events.jsonl` 因而不依赖 ROS 时间、NTP 或消息头时间戳。消息头时间戳仍被保留作辅助证据。
`/sdk2_command` 是 worker 观测到 `Move()` RPC 成功后发布的完成流，因此间隙包含 SDK
调用时间和最多一个 ROS 控制周期的 completion 轮询延迟；它不能表示 `Move()`
的原始入队时刻，也不能单独证明物理运动。

## 运行

先正常启动 SLAM、`flat_obstacle` 规划层和 `go2_sdk2_bridge`。不要同时运行 direct bridge、
simple-nav executor 或 `/lowcmd` 控制器。然后在一个新的 Jetson 终端运行：

```bash
cd ~/catkin_ws
export GO2_FASTDDS_PROFILE="$PWD/config/fastdds/jetson_wifi.xml"

bash ./tools/run_sdk2_motion_stall_probe.sh --duration 45
```

脚本会在权限为 `0700` 的随机临时目录
`/tmp/go2-sdk2-motion-stall-build.XXXXXX` 中构建仓库的只读 C++ 读取器，并在退出时删除该
构建目录，然后等待四个数据源完成 DDS 发现。它要求 SLAM/规划启动流程已建立有效的
`go2-log` active-session，
且该会话的 collector PID、命令行、未上传/未结束标记和容量余量均通过检查。脚本在
采集前直接预留权限为 `0700` 的证据子目录，Python 通过持有的 directory fd 写入，
不跟随采集期间替换的符号链接。Python 在 DDS 发现和采集循环的每个最多 `50 ms` 检查点
重新核对 active-session 文件、预期会话、结束/上传标记、collector PID、进程存活和
`/proc/<pid>/cmdline` 身份。看到 `READY` 后，才开始 45 秒采集。

wrapper 只允许覆盖 `--duration` 和 `--discovery-timeout`；reader、网卡和输出目录由脚本
固定，命令行缩写或重复参数不能覆盖它们。单次采集最多保留 `120000` 个事件或
`24 MiB` 原始事件 JSON；reader 标准错误和解析错误使用独立的 `1 MiB` 有界缓冲区，报告与
诊断文本合计最多 `4 MiB`。触发事件或诊断上限时会保存已采集的主事件证据和明确的截断原因，
但结果标记为 `insufficient_capture`，不作为排除卡住的依据。

若采集期间 collector 停止、active-session 改变，或会话出现 `.uploaded` / `ended_at.txt`，
探针会立即终止 reader 和 ROS 订阅，原子发布已捕获的证据文件，并在 `report.json` 写入
`capture_invalidated`。这类运行返回非零，`required_streams_complete=false` 且
`stall.status=insufficient_capture`；保留下来的数据只能用于追查采集失效，不能作为运动桥
正常或卡住的结论。探针不会自行调用 `go2-log start` 或 `go2-log stop`。

探针不会授权机器狗。在另一个已经加载项目 ROS 环境的终端中，按正常安全流程明确授权：

```bash
cd ~/catkin_ws
export GO2_FASTDDS_PROFILE="$PWD/config/fastdds/jetson_wifi.xml"
source ./shell/ros2_environment.sh

ros2 service call /go2_sdk2_bridge/enable_motion \
  std_srvs/srv/SetBool '{data: true}'
```

随后在 RViz 发布一个全新的、可见平地上的短距离目标。采集期间原生遥控器必须保持居中，
不得在 bridge 仍 armed 时用遥控器同时发送运动命令。若必须人工接管，先调用
`enable_motion data: false`；紧急情况下始终以现场安全为先，本次样本按中断处理，不能用于
证明“遥控器唤醒”。采集完成后结果保存在当前诊断会话内：

```text
~/go2_logs/sessions/<active-session>/sdk2_motion_stall_YYYYMMDDTHHMMSSZ_PID/
```

目录包含原始排序事件 `events.jsonl`、机器可读 `report.json` 和 SDK2 读取器标准错误。运行
wrapper 不允许覆盖 `--output-dir`，避免诊断证据绕过会话保留和上传规则。

## 结果解释

终端摘要分别给出 `command`、`odom`、`sport` 和 `remote` 的样本数、平均频率、最大接收间隙
及独立状态。默认间隙界限为：指令 250 ms、里程计 500 ms、sport 250 ms。遥控器数据即使
内部相邻帧正常，只要没有覆盖非零指令区间也会标记为 `partial`，不能用于遥控器相关结论。

- `stall=observed`：非零指令持续至少 750 ms，但 body odom 没有达到运动响应阈值。这是“有
  指令但没有物理位姿响应”的证据，不等同于电机级故障结论。`report.json` 还分别保存
  `odom_episodes`、`sport_episodes` 和两者同时停滞的 `combined_episodes`，因此 sport 先回显速度
  时不会掩盖 odom 卡住。
- `stall=not_observed`：只有在非零命令至少连续覆盖同一个 `750 ms` 观测窗口且
  所需反馈完整时才会给出；单个或过短非零命令只能得到
  `insufficient_active_command`。
- `remote_wake=observed`：捕获数据中，拨杆前已有持续卡住，并且从卡住观测窗口、拨杆偏转
  直到回中后的响应确认，bridge 的非零指令始终连续；sport 或 odom 响应持续至少
  `0.75 s`。它只分析已经发生的事件，不是让操作员在两条控制
  路径同时激活时主动复现实验；该结果也不能单独证明遥控器仲裁就是根因。
- `remote_wake=remote_only_motion`：只在拨杆偏转期间检测到运动；回中后的完整 `2 s` 观察窗内
  bridge 非零指令保持连续，且 sport 和 odom 均未继续报告运动。若回中后 bridge 指令归零，
  只能得到 `not_observed` 或 `insufficient_*`，不能用零命令区间证明“仅遥控器运动”。这个状态
  只能证明原生遥控器本身有效，不能证明它唤醒了 SDK2 自动控制。
- `yaw_sign=odom_sport_disagree`：sport 偏航速度与指令同号，而 body odom 的偏航变化主要反号。
  优先检查机体坐标修正、偏航定义和时间对齐，不应通过反向 SDK 指令猜测修复。
- `yaw_sign=mismatch`：至少一个反馈源积累了 0.05 rad 以上的反号证据，且反号比例达到 70%。
- `insufficient_*`：本次没有足够的非零指令、反馈变化或完整数据，不能对相应问题下结论。

离线回放不需要 ROS 2 或 SDK2，可在仓库中运行：

```bash
python3 ./tools/sdk2_motion_stall_probe.py \
  --replay ~/go2_logs/sessions/<session-id>/sdk2_motion_stall_*/events.jsonl
```

静态安全约束和现场症状分类由下面的快速回归测试覆盖：

```bash
python3 ./tools/test_sdk2_motion_stall_probe.py
```
