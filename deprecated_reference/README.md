# 已停用参考资料——禁止用于正式运行

> **本目录中的所有内容均已停用。它们不参与构建、安装、启动和运行，仅供历史参考。**

根目录的 `COLCON_IGNORE` 会阻止 colcon 发现本目录中的任何历史内容。

规则：

- 不得从正式 shell、launch、CMake 或日常测试引用本目录；
- 不保证归档脚本在当前位置仍可直接执行；
- 不得把本目录中的 direct bridge 用于有障碍环境；
- 不得恢复 go2-log 对 SLAM、规划或运动启动的依赖；
- 若未来要重新启用某项内容，应建立独立分支、重新评审并完成实机验证。

归档分类：

- `direct_bridge/`：绕过 `/body_path` 和障碍规划的旧实验执行器；
- `go2_log/`：旧诊断采集、修复、分析和 GitHub 上传链；
- `sdk2_motion_stall_probe/`：用于定位早期 SDK2 卡住问题的同步探针；
- `historical_research/`：SDK2 接口研究记录；
- `historical_docs/`：整理前的完整项目 README。

正式交付入口始终是仓库根目录的 `README.md` 和 `docs/OPERATIONS.md`。
