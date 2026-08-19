# SDK2 Direct Goal Bridge

This is a separate experimental SDK2 bridge. The normal terrain mapper,
lattice planner, obstacle displays, inflation layer, and RViz remain running
unchanged. The original `go2_sdk2_bridge_node` must not run at the same time.

The direct bridge consumes only `/goal_pose` and the existing
`/lio/body_odom`. It does not subscribe to `/body_path`, terrain maps, or
planner diagnostics. Planner failures and path clearing therefore do not stop
an accepted direct-bridge goal.

The `/body_path` still visible in RViz is planner diagnostics only. It is not
the route sent to the robot and may differ from the direct bridge's Manhattan
route, including around obstacles.

For a diagonal goal it builds a Manhattan route with at most one right-angle
waypoint. It aligns the body heading before each translation leg, reaches the
goal, and then aligns to the goal yaw.

Explicit `~/enable_motion`, rejection of an active `/lowcmd` publisher, and a
wall-clock odometry receive watchdog remain. This bridge is obstacle-unaware
even while the terrain map is displayed; use it only for short open-ground
commissioning.

Run the normal SLAM and terrain-navigation commands first. Then start only this
bridge instead of `start_sdk2_bridge.sh`:

```bash
cd ~/catkin_ws
export GO2_FASTDDS_PROFILE="$PWD/config/fastdds/jetson_wifi.xml"
source ./shell/ros2_environment.sh
./shell/start_sdk2_direct_bridge.sh
```

After updating from the earlier temporary simple-navigation commit, rebuild
`utree_go2_sdk2_bridge` and verify that
`install/utree_go2_sdk2_bridge/lib/utree_go2_sdk2_bridge/go2_sdk2_direct_bridge_node`
exists. Do not run the obsolete `go2_sdk2_simple_nav_node` artifact if an
incremental install left it behind.

Arm it in another terminal:

```bash
cd ~/catkin_ws
export GO2_FASTDDS_PROFILE="$PWD/config/fastdds/jetson_wifi.xml"
source ./shell/ros2_environment.sh
ros2 service call /go2_sdk2_direct_bridge/enable_motion \
  std_srvs/srv/SetBool '{data: true}'
```
