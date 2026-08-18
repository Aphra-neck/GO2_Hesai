# Go2 官方 SDK2 运动、控制权与导航接口核对

核对日期：2026-08-18

本结论仅使用宇树官方文档和官方 `unitree_sdk2` 源码。官方 SDK 本地检出为
`unitreerobotics/unitree_sdk2` 的提交
[`21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b`](https://github.com/unitreerobotics/unitree_sdk2/tree/21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b)。

## 结论

1. 当前工程的执行架构是正确的：Super-LIO、地形图和路径规划保留在 ROS 2 中，
   `utree_go2_sdk2_bridge` 将 `/body_path` 和 `/lio/body_odom` 转成 Go2
   `SportClient::Move(vx, vy, vyaw)`。需要继续修改和验证的是这个桥接器，而不是规划器，也不是
   换成宇树官方 SLAM 导航服务。
2. Go2 Edu 软件版本 `>= V1.1.6` 应使用当前 V2.0 高层运控接口。当前运控模式名为
   `mcf`。官方当前高层示例直接初始化并调用 `SportClient`，没有把
   `MotionSwitcherClient::SelectMode()` 或 `ReleaseMode()` 作为前置步骤。
3. `Move()` 是机体系速度接口。最新命令维持 1 秒，运控不替调用者滤波；因此本工程以 20 Hz
   发送经过限幅/滤波的速度，并在结束时调用 `StopMove()`，符合官方接口语义。
4. `BalanceStand()` 的作用是从“站立锁定”切到“平衡站立”，不是每次 `Move()` 前都必须重复
   调用。官方状态表中 `error_code=1002` 是站立锁定，`1013` 是平衡站立。因此仅在检测到
   `1002` 时请求一次 `BalanceStand()` 并等待状态离开 `1002`，是有官方依据的状态门控。
5. `SwitchJoystick(false)` 是官方提供的原生遥控器仲裁接口；官方明确说明关闭响应后，推动原生
   遥控器摇杆不会干涉当前程序。本工程在取得 SDK2 运动控制权前关闭遥控器响应，并在停车、
   禁用、故障和退出时先 `StopMove()`、再 `SwitchJoystick(true)`，是解决“RPC 返回成功且
   `/sdk2_command` 非零，但实体不走”这一仲裁缺口的正确修改方向。
6. `ReleaseMode()` 在官方 Go2 低层电机示例中用于关闭高层运控服务，然后才发布
   `rt/lowcmd`。高层 `SportClient` 桥接器不应调用它；否则会关闭桥接器依赖的运控服务。
7. 宇树官方 SLAM 导航服务是另一套端到端栈。官方明确说明其 SDK2、`unitree_slam`、雷达驱动
   和测试程序占用 CycloneDDS，并与已初始化的 ROS/ROS2 环境存在二进制冲突。它不能作为
   当前 ROS 2 Humble + Super-LIO 管线中的一个可并行组件。

## 官方接口事实

### 1. 版本与 `mcf`

宇树的[运控服务接口 V2.0](https://support.unitree.com/home/zh/developer/Motion_Services_Interface_V2.0)
写明：V2.0 文档适用于 Go2 Edu，软件版本要求 `>= V1.1.6`，并要求同步更新
[`unitree_sdk2`](https://github.com/unitreerobotics/unitree_sdk2)。版本低于 `V1.1.6` 时，官方让
用户改查旧版运控切换、高层运动和 AI 运动文档。官方原始正文快照：
[`6_814_zh`](https://doc-cdn.unitree.com/6/814/zh/6_814_zh)。

宇树的[运控切换服务接口](https://support.unitree.com/home/zh/developer/Motion%20Switcher%20Service%20Interface)
给出的模式名表是：

| Go2 软件版本 | 官方模式名 |
| --- | --- |
| `>= V1.1.6` | `mcf` |
| `< V1.1.6` | `ai`、`normal`、`advanced` |

原始正文快照：[`6_261_zh`](https://doc-cdn.unitree.com/6/261/zh/6_261_zh)。

这两个页面合起来支持以下判断：看到 `CheckMode(..., name)` 返回 `mcf` 时，机器人已经在新版
运控体系中。V2.0 的高层调用方式和官方当前 Go2 示例都没有要求先执行 `SelectMode("mcf")`。
因此，不应仅因为机器人没有物理运动，就在高层桥接器中试探性地反复
`SelectMode()`/`ReleaseMode()`。

### 2. `SportClient` 初始化

V2.0 文档给出的调用入口为：

```cpp
unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
unitree::robot::go2::SportClient sport_client;
sport_client.SetTimeout(10.0f);
sport_client.Init();
```

其中 DDS domain 为 `0`，并显式传入连接机器人的网卡。官方当前示例也采用同样顺序：

- [`go2_sport_client.cpp:36`](https://github.com/unitreerobotics/unitree_sdk2/blob/21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b/example/go2/go2_sport_client.cpp#L36-L42)：设置超时并初始化 `SportClient`。
- [`go2_sport_client.cpp:141`](https://github.com/unitreerobotics/unitree_sdk2/blob/21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b/example/go2/go2_sport_client.cpp#L141-L155)：先用 domain `0` 和网卡初始化 `ChannelFactory`，再创建控制对象和周期线程。
- [`sport_client.hpp:31`](https://github.com/unitreerobotics/unitree_sdk2/blob/21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b/include/unitree/robot/go2/sport/sport_client.hpp#L31-L52)：官方头文件声明 `Init`、`BalanceStand`、`StopMove`、`Move` 和 `SwitchJoystick`。

### 3. `Move(vx, vy, vyaw)`

V2.0 文档定义：

- `vx`、`vy`、`vyaw` 是机体坐标系下的纵向、横向和偏航速度。
- 官方范围分别为 `vx [-2.5, 3.8] m/s`、`vy [-1.0, 1.0] m/s`、
  `vyaw [-4, 4] rad/s`。
- 运控部分不对 `Move` 指令滤波。
- 最新 `Move` 指令维持 1 秒。
- 官方建议调用者自行滤波并持续发送；不再使用 `Move` 时发送 `Move(0,0,0)` 或
  `StopMove()`。

官方当前 Go2 示例在周期线程的 `velocity_move` 分支直接调用
[`Move(0.3, 0, 0.3)`](https://github.com/unitreerobotics/unitree_sdk2/blob/21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b/example/go2/go2_sport_client.cpp#L69-L71)，
控制步长为
[`0.005 s`](https://github.com/unitreerobotics/unitree_sdk2/blob/21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b/example/go2/go2_sport_client.cpp#L131-L138)。
这支持“周期刷新速度”的使用方式，但官方没有要求固定为该示例的 200 Hz；本工程的 20 Hz
仍远快于 1 秒保持窗口。

### 4. `BalanceStand()` 与状态机

V2.0 文档把 `BalanceStand()` 定义为“解除锁定”：从正常站立模式切换到平衡站立模式，解除
关节电机锁定，并维持机身姿态和高度平衡。

同一文档的 `rt/sportmodestate` 状态表定义：

| `error_code` | 官方状态名称 |
| --- | --- |
| `100` | 灵动 |
| `1001` | 阻尼 |
| `1002` | 站立锁定 |
| `1013` | 平衡站立 |
| `1015` | 常规行走 |

所以 `1002` 不是 SDK RPC 错误，而是运动状态机状态。当前官方 Go2 示例把
`BalanceStand()` 和 `Move()` 放在不同测试分支，`Move()` 分支没有无条件先调用
`BalanceStand()`：见
[`go2_sport_client.cpp:56`](https://github.com/unitreerobotics/unitree_sdk2/blob/21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b/example/go2/go2_sport_client.cpp#L56-L71)。
因此，准确策略是“锁定时解锁并确认状态转换”，不是“每个控制周期都先
`BalanceStand()`”。

### 5. `StopMove()`

V2.0 文档定义 `StopMove()` 为停止当前运动，并把 Go2 内部绝大多数运动参数恢复为默认值；
`Move` 章节也明确要求停止使用速度接口时发送零速度或 `StopMove()`。官方示例在
`stop_move` 和默认分支都调用
[`StopMove()`](https://github.com/unitreerobotics/unitree_sdk2/blob/21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b/example/go2/go2_sport_client.cpp#L105-L110)。

### 6. `SwitchJoystick(bool)`

V2.0 文档的定义非常明确：

- `SwitchJoystick(true)`：响应原生遥控器。
- `SwitchJoystick(false)`：不响应原生遥控器。
- 关闭响应后，推动遥控器摇杆不会干涉当前程序运行。

官方文档只定义该接口的功能，没有规定它必须在每次 `Move()` 前调用，也没有提供一个包含
`SwitchJoystick`、`BalanceStand`、`Move`、`StopMove` 的唯一组合示例。因此，下面的生命周期是
本项目从官方接口语义推导出的控制权策略，而不是伪装成官方原话：

1. 等待操作员显式授权、新鲜路径和有效里程计；空闲时保持原生遥控器可用。
2. 首次执行 SDK2 运动前调用一次 `SwitchJoystick(false)`，确认成功后才继续。
3. 如果状态为 `1002`，调用 `BalanceStand()`，等待状态离开站立锁定；其他可执行状态不重复
   调用。
4. 按控制周期发送经限幅和滤波的 `Move(vx, vy, vyaw)`。
5. 到达目标、路径为空/过期、里程计过期、显式禁用、异常或进程退出时，先调用
   `StopMove()`，再调用 `SwitchJoystick(true)`。
6. RPC 报错或回复丢失时不能证明副作用没有发生；保持保守的“可能仍在运动/可能仍屏蔽遥控器”
   状态，并重试停车与恢复遥控器，直到确认成功。
7. 新进程启动时同样先执行一次 `StopMove()`，确认后再执行 `SwitchJoystick(true)`，恢复上一个
   进程被强制结束后可能遗留的一秒速度保持和遥控器仲裁状态。

第 6、7 点是本项目的失效安全设计，不是 SDK 文档对传输语义的保证。

### 7. `MotionSwitcherClient`

官方头文件公开了
[`CheckMode`、`SelectMode`、`ReleaseMode`、`SetSilent`、`GetSilent`](https://github.com/unitreerobotics/unitree_sdk2/blob/21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b/include/unitree/robot/b2/motion_switcher/motion_switcher_client.hpp#L15-L27)。
官方切换服务文档也逐一说明这些 RPC，但没有把 `SelectMode()`/`ReleaseMode()` 列为新版
V2.0 `SportClient` 调用的前置步骤。

需要特别区分高层与低层控制：官方 Go2 低层示例在创建 `rt/lowcmd` 发布器后，循环
`CheckMode()` 并调用 `ReleaseMode()`，日志文字就是“Shut down motion control-related
service”：见
[`go2_stand_example.cpp:116`](https://github.com/unitreerobotics/unitree_sdk2/blob/21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b/example/go2/go2_stand_example.cpp#L116-L142)。
这正是低层电机控制接管前的动作，不是高层 `SportClient` 桥接器的启动步骤。

因此本项目应保持：

- 运行时可以只读 `CheckMode()`，确认新版机器人报告 `mcf`。
- 高层桥接器不调用 `ReleaseMode()`。
- 不把 `SelectMode("mcf")` 当作“机器不走”的试错修复；如果 `mcf` 已激活，该调用没有解决
  遥控器仲裁缺口。
- 继续禁止与任何 `/lowcmd` 发布器并行运行。

## 官方 SLAM/导航服务与本项目的关系

宇树的[SLAM 导航服务接口](https://support.unitree.com/home/zh/developer/SLAM%20and%20Navigation_service)
说明：

- 仅支持带拓展坞且使用宇树官方购入 MID-360/XT16 的 EDU 机器狗。
- 要求最新 SLAM 包和固件 `>= V1.1.7`，建议用于小于 `25 m x 25 m`、特征丰富的静态室内
  平地。
- 服务名是 `slam_operate`，版本 `1.0.0.1`；例如 API `1102` 是位姿导航，`1201`/`1202`
  是暂停/恢复导航。
- 服务运行在拓展坞 PC 上，使用自己的 `unitree_slam`、雷达驱动和 CycloneDDS 配置。
- 官方明确警告：功能测试例程与 ROS2 存在二进制冲突；SDK2、`unitree_slam`、雷达驱动和
  `keyDemo` 占用 CycloneDDS，不能在初始化过 ROS/ROS2 的环境中启动。

官方原始正文快照：[`6_111_zh`](https://doc-cdn.unitree.com/6/111/zh/6_111_zh)。

这套服务能够独立完成宇树定义的建图和导航，但不是当前 Hesai + Super-LIO + 自研地形规划
管线的执行 API。若切换到它，就相当于替换整个感知和导航栈，而不是修复当前桥接器。当前问题
已经能看到有效 `/body_path` 和非零 `/sdk2_command`，所以没有依据替换规划栈。

## 与本仓库实现的对应关系

本仓库的
`src/utree_go2_sdk2_bridge/include/utree_go2_sdk2_bridge/go2_sdk2_bridge_node.hpp`
把节点职责直接定义为“把几何 body path 转成有界的 Go2 `SportClient` 速度命令”。实现中的
关键位置是：

- `src/utree_go2_sdk2_bridge/src/go2_sdk2_bridge_node.cpp:93`：以配置的 domain 和网卡初始化
  SDK2 `ChannelFactory`，然后构造、设置超时并初始化 `SportClient`。
- 同文件 `:105`：订阅 `/body_path`；`:111`：订阅 `/lio/body_odom`；`:114`：创建
  `/sdk2_command` 诊断发布器。
- 同文件 `:679`：执行前取得原生遥控器控制权；`:689`：实际调用
  `SportClient::Move()`；`:705` 以后只有在 SDK 返回成功后才发布 `/sdk2_command`。
- 同文件 `:777`：停车和控制权释放；先确认 `StopMove()`，再确认
  `SwitchJoystick(true)`。

因此 `/sdk2_command` 非零只证明桥接器计算出了命令且 `Move()` RPC 返回了 `0`；它不是关节
已经产生物理运动的反馈。既然规划路径、命令计算和 RPC 返回都已通过，剩余排查面就是 SDK2
服务状态、原生遥控器仲裁以及机器人侧运动状态，不应继续修改地形图或路径规划算法。

## 可执行判断

| 问题 | 结论 |
| --- | --- |
| 应修改规划器吗？ | 否；已有有效路径和非零桥接命令。 |
| 应改用官方 `slam_operate` 吗？ | 否；它是冲突且互斥的另一套完整导航栈。 |
| 应在桥接器里调用 `ReleaseMode()` 吗？ | 否；官方低层示例用它关闭高层运控服务。 |
| `mcf` 是否异常？ | 否；它是 `>= V1.1.6` 的官方运控模式名。 |
| `Move()` 用法是否正确？ | 正确；机体系速度、周期刷新、结束停车。 |
| 当前最关键缺口是什么？ | 程序运动期间显式屏蔽原生遥控器，并在所有终止路径恢复它。 |
| 实际修改位置在哪里？ | `src/utree_go2_sdk2_bridge/src/go2_sdk2_bridge_node.cpp` 的 SDK2 控制权生命周期。 |

## 来源清单

1. 宇树官方，《运控服务接口 V2.0》：
   <https://support.unitree.com/home/zh/developer/Motion_Services_Interface_V2.0>
2. 宇树官方，《运控切换服务接口》：
   <https://support.unitree.com/home/zh/developer/Motion%20Switcher%20Service%20Interface>
3. 宇树官方，《SLAM 导航服务接口》：
   <https://support.unitree.com/home/zh/developer/SLAM%20and%20Navigation_service>
4. 宇树官方 `unitree_sdk2` 源码，固定提交：
   <https://github.com/unitreerobotics/unitree_sdk2/tree/21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b>
