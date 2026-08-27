# Go2 官方 SDK2 运动、控制权与导航接口核对

核对日期：2026-08-19

接口事实仅使用宇树官方文档和官方 `unitree_sdk2` 源码；手倒立事件小节另外引用本仓库历史
和操作员现场观察，并与官方事实分开标注。官方 SDK 本地检出为
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
3. `Move()` 是机体系速度接口。最新命令维持 1 秒，运控不替调用者滤波；因此本工程按官方
   示例的 `5 ms / 200 Hz` 周期直接刷新固定速度，在 armed 等待和正常完成时持续发送零速 `Move()`，并在
   SDK 调用失败、路径/里程计输入超时、`/lowcmd` 冲突、显式禁用或退出时调用 `StopMove()`。
4. `BalanceStand()` 的作用是从“站立锁定”切到“平衡站立”，但官方速度示例没有把它列为
   `Move()` 的前置调用。当前 bridge 不自动调用它，也不根据 `rt/sportmodestate` 自动选择
   姿态或步态；操作员仍必须在 arm 前通过原生遥控器或官方 App 确认普通可行走站立状态。
5. `SwitchJoystick(false)` 是官方提供的原生遥控器仲裁接口，但官方没有要求速度控制程序必须
   调用它。当前 bridge 不调用该接口，原生遥控器保持可用；自动执行期间不得同时发送普通
   摇杆运动命令。
6. `ReleaseMode()` 在官方 Go2 低层电机示例中用于关闭高层运控服务，然后才发布
   `rt/lowcmd`。高层 `SportClient` 桥接器不应调用它；否则会关闭桥接器依赖的运控服务。
7. 宇树官方 SLAM 导航服务是另一套端到端栈。官方明确说明其 SDK2、`unitree_slam`、雷达驱动
   和测试程序占用 CycloneDDS，并与已初始化的 ROS/ROS2 环境存在二进制冲突。它不能作为
   当前 ROS 2 Humble + Super-LIO 管线中的一个可并行组件。

## 两篇官方运动文档如何用于当前 bridge

用户给出的两篇文章不是两套需要同时执行的启动步骤：

- [《运控服务接口 V2.0》](https://support.unitree.com/home/zh/developer/Motion_Services_Interface_V2.0)
  是 Go2 Edu 软件 `>= V1.1.6` 的当前接口说明；该页本身把旧软件版本引向旧版运控切换、
  [《高层运动控制》](https://support.unitree.com/home/zh/developer/High_motion_control)和 AI 运动文档。
- `High_motion_control` 因而只能作为旧固件调用面的兼容参考，不能覆盖 V2.0 的版本选择规则。
  现场只读探针已经得到 `CheckMode(...)=mcf`；这与 V2.0 运控体系一致，但不能单凭模式名推导
  出完整固件版本，仍须单独核对机器人固件。

当前 bridge 可以而且应当参考 V2.0，但只参考高层速度执行部分。对应关系如下：

| 官方接口职责 | 当前 bridge 对应 | 决策 |
| --- | --- | --- |
| domain `0`、指定机器狗网卡初始化 SDK2 | `ChannelFactory::Init(domain_id_, network_interface_)` | 已一致 |
| 初始化 `go2::SportClient` | 构造、`SetTimeout()`、`Init()` | 已一致 |
| 发送机体系 `vx`、`vy`、`vyaw` | 把 ROS `/body_path` 与 `/lio/body_odom` 转为 `Move(...)` | 架构正确 |
| 周期刷新速度 | 控制定时器每 `5 ms` 直接同步提交 `Move(...)`，默认 `200 Hz` | 已一致 |
| 停止与等待 | armed 等待/正常完成发送 `Move(0,0,0)`；禁用、故障和退出调用 `StopMove()` | 已一致 |
| 原生遥控器是否响应 | 可选 `SwitchJoystick(bool)` | 不是 `Move()` 前置步骤；当前不调用 |
| 高层/低层运控切换 | `MotionSwitcherClient` | 高层 bridge 不 `ReleaseMode()`，不发布 `/lowcmd` |

官方当前
[`go2_sport_client.cpp`](https://github.com/unitreerobotics/unitree_sdk2/blob/21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b/example/go2/go2_sport_client.cpp#L36-L71)
是决定调用顺序的最直接源码证据：初始化完成后，`velocity_move` 分支直接周期调用 `Move()`；
它没有先调用 `StandUp()`、`BalanceStand()`、`SwitchJoystick()`、`SelectMode()` 或
`ReleaseMode()`。因此这两篇文章可以指导桥接器的 SDK 调用面，但不能支持把这些状态改变 RPC
作为“不运动”的试错前置步骤。

这两篇文档也不能把 `/sdk2_command` 当成物理运动反馈。当前 bridge 仅在同步
`Move()` RPC 返回成功后发布该诊断话题；其时间戳是调用完成时间，不是电机已经执行的
反馈。因此频率下降可以表示 SDK RPC 变慢，但不证明电机已经执行。剩余问题应从遥控器
仲裁、机器人固件和运行时 SDK2/API 版本继续核对，而不是继续修改规划器。

## SDK、服务 API 与固件版本边界

本次固定核对的官方 `unitree_sdk2` 提交中：

- CMake 工程版本是
  [`2.0.0`](https://github.com/unitreerobotics/unitree_sdk2/blob/21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b/CMakeLists.txt#L1-L2)。
- Go2 sport 服务 API 常量是
  [`1.0.0.1`](https://github.com/unitreerobotics/unitree_sdk2/blob/21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b/include/unitree/robot/go2/sport/sport_api.hpp#L12-L16)。
- Go2 robot-state 服务 API 常量也是
  [`1.0.0.1`](https://github.com/unitreerobotics/unitree_sdk2/blob/21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b/include/unitree/robot/go2/robot_state/robot_state_api.hpp#L13-L20)。
- 通用客户端公开
  [`GetApiVersion()` 和 `GetServerApiVersion()`](https://github.com/unitreerobotics/unitree_sdk2/blob/21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b/include/unitree/robot/client/client.hpp#L21-L31)，
  官方 Go2 robot-state 示例会比较两者。这只能核对客户端与机器人服务端的**服务 API 版本**。
- Go2 的
  [`RobotStateClient`](https://github.com/unitreerobotics/unitree_sdk2/blob/21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b/include/unitree/robot/go2/robot_state/robot_state_client.hpp#L27-L40)
  没有 `GetPkgVersion()`；不能把 `GetServerApiVersion()` 冒充完整机器人固件版本，也不能借用
  B2 专用接口来查询 Go2。

`SportClient` 构造函数的 `enableLease` 默认值为 `false`，当前 bridge 与官方 Go2 示例都使用
默认构造。这个事实只能说明二者的客户端构造方式一致，不能据此断言原生遥控器当前拥有或
失去了控制权。官方可见的遥控器接口是 `SwitchJoystick(bool)`；当前头文件没有对应的状态 getter。
只读订阅 `rt/wirelesscontroller` 可以证明遥控器帧和摇杆值到达 SDK2 DDS，但不能单独证明
sport 服务正在响应这些帧。

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
本工程直接采用该示例的 `5 ms / 200 Hz` 刷新周期，以排除低频刷新差异。

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
| `1016` | 常规跑步 |
| `1017` | 常规续航 |

所以 `1002` 不是 SDK RPC 错误，而是运动状态机状态。当前官方 Go2 示例把
`BalanceStand()` 和 `Move()` 放在不同测试分支，`Move()` 分支没有无条件先调用
`BalanceStand()`：见
[`go2_sport_client.cpp:56`](https://github.com/unitreerobotics/unitree_sdk2/blob/21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b/example/go2/go2_sport_client.cpp#L56-L71)。
因此，官方资料只能证明 `BalanceStand()` 的含义，不能证明自定义速度 bridge 必须自动调用
它。当前实现恢复为官方 `velocity_move` 示例的直接调用面；正常可行走站立状态由操作员在
arm 前建立和确认。

普通速度导航不需要先选择 `StaticWalk()`、`TrotRun()` 或 `EconomicGait()`。官方当前 Go2
示例的 `velocity_move` 分支直接调用 `Move(0.3, 0, 0.3)`；这三个函数是独立的可选模式接口，
[`API ID`](https://github.com/unitreerobotics/unitree_sdk2/blob/21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b/include/unitree/robot/go2/sport/sport_api.hpp#L42-L44)
分别为 `1061`、`1062`、`1063`，官方状态表对应的 `1015`、`1016`、`1017` 分别表示
常规行走、常规跑步和常规续航。官方资料没有把其中任何一个列为 `Move()` 的前置条件。因此
桥接器不应为了修复“不运动”而自动选择这些步态。

当前 bridge 不订阅或门控 `rt/sportmodestate`，不会把状态码解释成自动姿态或步态操作。
独立同步探针用于比较命令与反馈，诊断结果也不能触发 `BalanceStand()`、特殊动作或模式切换。

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
`SwitchJoystick`、`BalanceStand`、`Move`、`StopMove` 的唯一组合示例。当前实现采用已经实机
运动过、也最接近官方 `velocity_move` 示例的最小生命周期：

1. 操作员先用原生遥控器或官方 App 把机器人置于普通可行走站立状态，并确认没有 `/lowcmd`
   发布者。
2. bridge 在 disarmed 时不发送运动 RPC；armed、等待新路径时周期发送零速 `Move()`。
3. 执行期间每 `5 ms` 刷新组合速度命令 `Move(0.40, 0, ±0.60)`；小角度迟滞持续
   保留非零偏航分量，避免回到实机响应不稳定的零偏航平移命令。
4. 正常到达时继续发送零速 `Move()` 并等待新路径；SDK 调用失败、输入超时、`/lowcmd`
   冲突、显式禁用或退出时，对本进程可能发出的运动调用 `StopMove()`。
5. bridge 不调用 `SwitchJoystick()`。原生遥控器保持响应，安全员在自动执行期间不同时发送
   普通摇杆运动命令，只在需要人工接管或急停时操作。

如果进程被强制结束，SDK 文档说明最后一条 `Move()` 最多可能保持 1 秒。当前启动过程不会用
额外姿态、步态或仲裁 RPC 猜测恢复机器人状态，因此实体急停和原生遥控器仍是必要边界。

### 7. 手倒立事件：已知事实、未证实推断和处置边界

一次实体测试中，机器人出现了外观类似手倒立的姿态。该次测试没有同步保存
`rt/sportmodestate.error_code`，现有附件中也没有同一时间窗的 `error_code=2011` 记录；因此
“当时确实进入 2011”不能作为已证实事实。下面严格区分官方事实和项目推断。

**官方资料确认的事实：**

- V2.0 状态表把 `error_code=2011` 命名为“倒立”，把 `2009` 命名为“跳跃跑”：见官方正文
  快照 [`6_814_zh`](https://doc-cdn.unitree.com/6/814/zh/6_814_zh)。这个映射只解释回读值，
  不能反向证明没有被同步采集的那次事件就是 2011。
- 官方固定提交中的 API ID 分别是 `BalanceStand=1002`、`Move=1008`、
  `SwitchJoystick=1027`、`HandStand=2044`、`FreeJump=2047`：见
  [`sport_api.hpp`](https://github.com/unitreerobotics/unitree_sdk2/blob/21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b/include/unitree/robot/go2/sport/sport_api.hpp#L13-L49)。
  `HandStand(bool)` 和 `FreeJump(bool)` 也是与 `Move()`、`SwitchJoystick(bool)` 分开的公开函数：
  见
  [`sport_client.hpp`](https://github.com/unitreerobotics/unitree_sdk2/blob/21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b/include/unitree/robot/go2/sport/sport_client.hpp#L40-L67)。
- `Move()` 只发送机体系速度；最新指令保持 1 秒，且运控不替调用者滤波。官方示例的
  `velocity_move` 分支直接调用 `Move(0.3, 0, 0.3)`，没有调用手倒立或跳跃 API：见
  [`go2_sport_client.cpp`](https://github.com/unitreerobotics/unitree_sdk2/blob/21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b/example/go2/go2_sport_client.cpp#L56-L71)。
- 所核对文档对 `SwitchJoystick(false)` 的定义是不响应原生遥控器；文档没有说明它会进入倒立
  或跳跃，也没有规定它必须位于 `BalanceStand()` 之前。
- 官方 Go2 `SportModeState_` IDL 只把 `gait_type` 声明为 `uint8_t`，所核对的 V2.0 文档和
  SDK 源码没有给出可用于安全判定的 Go2 枚举映射：见
  [`SportModeState_.hpp`](https://github.com/unitreerobotics/unitree_sdk2/blob/21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b/include/unitree/idl/go2/SportModeState_.hpp#L35)。
  因此 `gait_type` 不能单独区分或阻止潜在倒立；有官方状态名称映射的是 `error_code`。

**项目已知事实：**

- 仓库提交
  [`f944d57`](https://github.com/Aphra-neck/GO2_Hesai/commit/f944d57b3f7191e1440ae51bbe27578d43387220)
  曾在释放 `1002` 站立锁定前调用 `SwitchJoystick(false)`；提交
  [`0dd3fd9`](https://github.com/Aphra-neck/GO2_Hesai/commit/0dd3fd92eefc89cb77e7757e3dd1772279ba6767)
  随后完整回退了该修改。这是本仓库历史事实，不是两者存在因果关系的证据。
- 提交 `24ed083` 又加入 `BalanceStand()` 和运动状态白名单；提交 `bab8ae1` 加入首次运动前
  `SwitchJoystick(false)`、等待新状态帧和启动恢复。在该版本的现场日志中，屏蔽原生遥控器后
  机器人回报 `error_code=2009 (jump run)`，bridge 随即停车。这个顺序证明新增路径被执行，
  但仍不能单独证明 `SwitchJoystick(false)` 导致了 `2009`。
- 外观类似手倒立的姿态来自操作员现场观察，没有同一时间窗的 `error_code=2011` 采样；不能
  把这次姿态直接认定为官方状态 `2011`。

**仍属推断：**

- 机器人可能在桥接器接管前已经处于某个潜伏的特殊步态，或者 SDK/固件对仲裁调用的实际行为
  与所核对的官方源码版本不同。
- `SwitchJoystick(false)` 可能只切断了操作员当时的遥控干预，使异常姿态更难立即恢复；现有
  证据不能证明该 RPC 触发了跳跃跑或手倒立。
- 本地核对的是官方提交 `21d0a3b...`。在取得 Jetson 已安装头文件、动态库以及机器人固件的
  版本或哈希前，不能假定运行端与该提交完全一致。

因此当前桥接器的运动调用只保留 `Move(...)` 和 `StopMove()`；在自动恢复、启动、停车、禁用、故障和
退出路径中均不得调用 `BalanceStand(...)`、`SwitchJoystick(...)`、`HandStand(...)`、
`FreeJump(...)` 或任何步态/模式切换 API。发现特殊姿态时由操作员使用原生遥控器或官方
App 处理；不要用相反参数调用特殊动作 API 猜测复位。

### 8. `MotionSwitcherClient`

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

- 独立诊断可以只读 `CheckMode()`，确认新版机器人报告 `mcf`；生产 bridge 不依赖它切换模式。
- 高层桥接器不调用 `ReleaseMode()`。
- 不把 `SelectMode("mcf")` 当作“机器不走”的试错修复；如果 `mcf` 已激活，该调用没有解决
  SDK/固件行为差异。
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

- 节点构造阶段以配置的 domain 和网卡初始化 SDK2 `ChannelFactory`，构造
  `SportClient`，并订阅 `/body_path` 与 `/lio/body_odom`。
- 控制定时器直接同步调用 `SportClient::Move()`，按配置频率持续刷新；正常等待路径和
  到达终点都发送 `Move(0, 0, 0)`，不经过异步命令 mailbox。
- 只有 `Move()` RPC 返回成功才发布 `/sdk2_command`；`StopMove()` 只用于 SDK/输入/低层
  冲突/显式停止和退出路径。
- 生产源码没有 `BalanceStand()`、`SwitchJoystick()`、特殊动作、步态或模式切换调用；该约束由
  `test/test_official_move_surface.py` 锁定。

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
| 当前恢复策略是什么？ | 运动调用保持官方 `Move/StopMove`；不自动切换姿态、步态、遥控器或运动模式。 |
| 还需实机确认什么？ | Jetson 安装的 SDK2/固件组合在直接调用面下是否恢复物理运动。 |
| 实际修改位置在哪里？ | `src/utree_go2_sdk2_bridge/src/go2_sdk2_bridge_node.cpp` 的 SDK2 调用面。 |

## 来源清单

1. 宇树官方，《运控服务接口 V2.0》：
   <https://support.unitree.com/home/zh/developer/Motion_Services_Interface_V2.0>
2. 宇树官方，《高层运动控制》：
   <https://support.unitree.com/home/zh/developer/High_motion_control>
3. 宇树官方，《运控切换服务接口》：
   <https://support.unitree.com/home/zh/developer/Motion%20Switcher%20Service%20Interface>
4. 宇树官方，《SLAM 导航服务接口》：
   <https://support.unitree.com/home/zh/developer/SLAM%20and%20Navigation_service>
5. 宇树官方 `unitree_sdk2` 源码，固定提交：
   <https://github.com/unitreerobotics/unitree_sdk2/tree/21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b>
6. 本仓库，`Harden SDK2 motion ownership`：
   <https://github.com/Aphra-neck/GO2_Hesai/commit/f944d57b3f7191e1440ae51bbe27578d43387220>
7. 本仓库，对上述提交的完整回退：
   <https://github.com/Aphra-neck/GO2_Hesai/commit/0dd3fd92eefc89cb77e7757e3dd1772279ba6767>

操作员现场观察没有可链接的结构化会话记录，未作为官方接口事实来源。
