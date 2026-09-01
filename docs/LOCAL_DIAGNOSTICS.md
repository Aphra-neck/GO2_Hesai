# 可选的本地诊断与日志系统

## 先看结论

当前正式运行链是完全本地的：

- `shell/start_slam.sh` 不访问 `G02_log`，不启动、停止、修复或上传 `go2-log` 会话；
- `shell/start_navigation.sh` 不访问 `G02_log`，也不上传规划或地图日志；
- `shell/start_sdk2_bridge.sh` 不访问 `G02_log`，也不上传运动日志；
- 日志仓库私有化、网络不可用、SSH key 缺失或日志上传失败，都不能阻断雷达、SLAM、规划或 SDK2 bridge 启动。

旧的 `go2-log` 实现已经放在
`deprecated_reference/go2_log/`，只用于历史参考，不是当前工具，也不保证从归档位置直接运行。
不要把它重新接回正式启动脚本。

如果后续开发者需要采集或上传日志，必须在自己的工作空间中另行建立一个“可选、只读、手动上传”的日志系统，
并由该开发者自己负责代码、权限、存储、维护和数据清理。本文件是建立该系统时的最低要求和建议流程。

## 1. 不可违反的边界

新日志系统必须满足以下条件：

1. **独立启动。** 它不能由三个正式启动脚本隐式调用，也不能成为任何 ROS 节点的启动前置条件。
2. **只读采集。** 允许读取 ROS 话题、节点状态、参数、进程输出和系统指标；不得发布 `/lowcmd`、
   `/sdk2_command`、`/goal_pose`，不得调用运动授权、停止或模式切换服务。
3. **失败隔离。** 采集器崩溃、磁盘不足、格式校验失败、远端不可达或上传失败时，只报告日志故障，
   不得杀死或阻塞雷达、SLAM、规划器和 SDK2 bridge。
4. **本地优先。** 先在本地完成封存和校验，再由操作员明确执行上传；运行中的机器人不执行 Git 操作。
5. **无凭据入库。** 私钥、token、密码、完整 SSH 配置、代理认证信息和含凭据的 URL 不得写入代码、
   manifest、环境快照或日志内容。
6. **有界采集。** 每个文件和每个会话都要有大小上限、超时和轮转策略。原始点云、rosbag/MCAP、网卡抓包、
   核心转储等大文件默认不采集到 Git 日志仓库。

## 2. 推荐的工作空间布局

日志系统的实现和数据要与正式源码、构建产物分开：

```text
~/catkin_ws/
├── tools/local_diagnostics/       # 开发者自己维护的采集器和校验器
└── .local_diagnostics/            # 本地会话数据，不提交到项目仓库
    ├── sessions/<session-id>/
    ├── quarantine/                # 待人工检查的异常文件
    └── locks/
```

也可以把采集器放在工作空间外的独立私有仓库；关键要求是它不能修改正式启动脚本的行为。
`.local_diagnostics/` 已列入忽略规则。建立后应立即确认：

```bash
cd ~/catkin_ws
mkdir -p tools/local_diagnostics .local_diagnostics/{sessions,quarantine,locks}
git check-ignore -v .local_diagnostics/sessions
git status --short
```

`git status` 不应出现运行会话、点云、bag、抓包或凭据文件。

## 3. 最小实现内容

### 3.1 `start`：只创建本地会话

每次会话至少记录：

- UTC 会话 ID、主机名、启动和结束时间；
- 当前工作空间 Git commit、分支和工作区是否干净；
- ROS 版本、domain、RMW、网络接口和关键参数；
- 参与运行的节点/进程清单及 PID；
- 低频 topic 频率、接收间隔和源时间戳摘要；
- 采集器自身版本、配置哈希和 schema 版本。

会话目录应在创建时使用唯一锁，避免两个采集器同时写同一个会话。文件先写临时名，完整落盘后再原子改名。
高频话题只记录摘要或限量样本，不要把完整 `/lidar_points` 或 `/lio/cloud_world` 写入 Git。

### 3.2 `stop`：封存而不是删除

停止时应：

1. 只停止自己创建的采集进程；
2. 写入结束时间、退出原因和丢包/截断计数；
3. 校验 JSON/JSONL/CSV/文本编码和行完整性；
4. 生成 `manifest.json` 和每个文件的 SHA-256；
5. 将会话标记为 `stopped` 或 `invalid`。

异常终止的会话先保留在本地或 `quarantine/`，不要用宽泛的 `rm -rf` 自动清理，也不要为了满足大小限制
静默改写已经封存的证据。

### 3.3 `upload`：明确的人工动作

上传命令必须是独立的、显式的子命令或独立脚本。建议只接受已经标记为 `stopped` 且校验通过的会话。
上传前至少检查：

```bash
cd ~/catkin_ws

# 下面两项是示例检查；按实际采集器实现替换，但不得省略安全确认。
ros2 topic info --no-daemon --spin-time 3 -v /lowcmd
pgrep -af '[r]l_controller|[u]tree_go2_rl_controller|[g]o2_sdk2_bridge_node'

# 确认所有机器人进程已经停止后，再检查会话状态和 manifest
find .local_diagnostics/sessions/<session-id> -maxdepth 1 -type f -print
sha256sum -c .local_diagnostics/sessions/<session-id>/manifest.sha256
```

上传期间不得运行 RL `/lowcmd` 控制器或 SDK2 bridge。上传失败只保留本地会话并返回错误，不能影响下一次正式启动。

## 4. 建立独立远端仓库（可选）

需要远端共享时，开发者应使用自己有权限管理的私有仓库或对象存储；不要默认使用现有的 `G02_log`，
也不要把远端地址、账号或 key 写死进正式项目。

下面是 SSH Git 远端的配置模板，所有尖括号内容都必须替换成开发者自己的值：

```bash
LOG_REPO="$HOME/go2_diagnostics_remote"
LOG_REMOTE='git@github.com:<ORG>/<PRIVATE_LOG_REPO>.git'
LOG_KEY="$HOME/.ssh/<diagnostics_deploy_key>"

test -r "$LOG_KEY" || { echo "missing diagnostics SSH key" >&2; exit 1; }
git clone "$LOG_REMOTE" "$LOG_REPO"
git -C "$LOG_REPO" config core.sshCommand \
  "ssh -i $LOG_KEY -o IdentitiesOnly=yes -o BatchMode=yes"
git -C "$LOG_REPO" remote -v
GIT_TERMINAL_PROMPT=0 git -C "$LOG_REPO" ls-remote origin HEAD
```

建议为日志仓库使用仅限该仓库的 deploy key，并在远端单独授予写权限。不要复用未获授权的项目 key，
不要把私钥复制到工作空间或提交到 Git。若网络环境只能通过 SSH over 443，应由开发者在本机 SSH 配置中管理，
而不是修改正式 ROS 启动脚本。

将会话复制到远端仓库后，建议按以下顺序操作：

```bash
rsync -a -- "$HOME/catkin_ws/.local_diagnostics/sessions/<session-id>/" \
  "$LOG_REPO/sessions/<session-id>/"
git -C "$LOG_REPO" add "sessions/<session-id>"
git -C "$LOG_REPO" diff --cached --check
git -C "$LOG_REPO" commit -m "Add diagnostics session <session-id>"
git -C "$LOG_REPO" push origin HEAD
git -C "$LOG_REPO" rev-parse HEAD
```

实际使用时应先确认远端 URL、分支和仓库归属，再执行 `push`。远端 commit 能够通过 `ls-remote` 或网页核对后，
才能按保留策略删除本地副本。超过 Git 平台单文件或仓库限制的原始数据，应转移到 SCP、S3、MinIO 或其他
专用对象存储，并在 manifest 中记录外部对象的 URI、大小和 SHA-256；不要强行塞进 Git。

## 5. 维护要求

日志系统的维护者至少要定期完成以下工作：

- **版本和 schema：** 给采集器和 manifest 标记版本；字段变化时递增 schema，并保留解析兼容性说明；
- **启动独立性测试：** 用假的 ROS graph/进程 fixture 验证“没有日志工具、日志工具退出、远端不可用”三种情况
  都不影响三个正式启动脚本；
- **容量与保留：** 配置单会话、单文件、总目录上限和保留天数/数量；清理前先验证已上传 commit 和校验和；
- **完整性：** 定期抽查 manifest、SHA-256、时间戳、UTF-8 和截断记录；发现异常时移入 quarantine；
- **权限轮换：** 定期轮换 deploy key，撤销离职人员和旧设备权限，检查远端仓库可见性；
- **隐私审查：** 上传前扫描私钥头、token、密码、代理 URL、局域网凭据和不应外传的原始传感器数据；
- **故障处理：** 记录“采集失败”和“上传失败”本身，但不得把失败转换为运动停止或节点启动失败；
- **文档同步：** 日志系统的命令、存储位置和远端发生变化时，只更新本可选文档和日志系统自己的 README，
  不要把依赖重新写入正式启动手册。

## 6. 交付前检查清单

后续开发者准备把日志系统交给别人使用前，应能逐项回答“是”：

- 正式启动脚本在没有日志工具时仍能正常运行；
- 日志工具不发布控制话题、不调用运动服务；
- 每个会话有上限、状态、manifest 和校验和；
- 上传是人工触发，并且只处理已停止、已校验的会话；
- 私钥、token、密码和代理认证信息没有进入仓库或日志；
- 大型原始数据有独立的对象存储方案；
- 远端权限和仓库归属已由维护者确认；
- 上传失败不会解除运动授权、杀死机器人进程或阻塞下一次启动；
- 归档和删除策略经过实际恢复演练。

本文件描述的是未来开发者自行维护的附加系统，不改变当前交付基线和正式启动命令。
