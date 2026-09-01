# Upstream source

This directory is vendored from:

- Repository: <https://github.com/Liansheng-Wang/Super-LIO.git>
- Branch: `ros2`
- Commit: `f89f48d` (`fix: relocation issue, fixes #29`)

The `src/super_lio/config/hesai.yaml` file carries the Go2 + XT-16 settings
from this repository: `/lidar_points`, `/imu/data`, measured gravity, filtering,
and the LiDAR-to-IMU extrinsic.

Integration patches on top of upstream:

- Livox input is optional and disabled by default (`SUPER_LIO_WITH_LIVOX=OFF`).
- ROS 2 launch/runtime dependencies and direct `tf2_ros` dependency are declared.
- TBB is discovered and linked explicitly instead of relying on transitive linkage.
- Humble-compatible ament dependency declarations and the relocation config path are fixed.
- The PCL runtime dependency uses the distribution-neutral `libpcl-dev` package
  instead of the Ubuntu 24.04-specific `libpcl-common1.14` package.
- LTO is disabled by default to reduce ARM64 link memory; enable it with
  `SUPER_LIO_ENABLE_LTO=ON` when the target has enough memory.
