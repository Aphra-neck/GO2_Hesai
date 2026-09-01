# Active verification tools

本目录只保留正式运行链的启动与 launch 回归：

```bash
bash tools/test_start_slam_cleanup.sh
bash tools/test_start_navigation.sh
bash tools/test_start_sdk2_bridge.sh
python3 tools/test_terrain_navigation_launch.py
```

go2-log、诊断分析器、运动卡住探针和无避障 direct bridge 测试已经迁入
`deprecated_reference/`，不属于交付回归。
