# go2-log 已退役

go2-log、会话修复、规划输入分析和 GitHub 上传链已经退出正式运行。

当前 SLAM、感知、规划和 SDK2 bridge：

- 不检查 G02_log 仓库；
- 不创建 go2-log 会话；
- 不执行 stop、repair 或 upload；
- 不会因日志缺失、仓库私有化、网络故障或上传失败而拒绝启动。

本目录中的脚本及测试保留原始结构供历史参考，不保证从归档位置直接运行。

如需重新建立诊断系统，请阅读正式文档
[`docs/LOCAL_DIAGNOSTICS.md`](../../docs/LOCAL_DIAGNOSTICS.md)，自行创建独立实现，
不要直接恢复本目录内容，也不要把上传逻辑接回正式启动脚本。
