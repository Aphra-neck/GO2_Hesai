# SDK2 Obstacle-Unaware Direct Goal Bridge (Experimental)

This is not the operational navigation bridge. Normal obstacle-aware operation
uses `./shell/start_sdk2_bridge.sh` and follows the planner-produced
`/body_path`. This is a separate experimental SDK2 bridge. The normal terrain mapper,
lattice planner, obstacle displays, inflation layer, and RViz remain running
unchanged. The original `go2_sdk2_bridge_node` must not run at the same time.

This experimental executor still performs `Move()` and `StopMove()` synchronously
inside its single-threaded ROS executor. An SDK RPC lasting up to its `0.5 s`
timeout can delay its odometry and control callbacks. The serialized SDK worker
and nonblocking-callback guarantee described for the standard `/body_path` bridge
do not apply here; do not use this direct bridge to validate the operational
stall fix.

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

After a goal completes, the bridge remains armed and refreshes
`Move(0, 0, 0)` at the command rate. This keeps the SDK2 locomotion stream
active for the next goal without an intervening `StopMove()`. Call
`~/enable_motion` with `data: false` to issue `StopMove()` and stop the
direct bridge's SDK2 motion stream. Do not use the native remote for routine driving while the
direct bridge remains armed; it does not call `SwitchJoystick()` automatically.

The pure geometry/controller simulation can be run without ROS or a Go2:

```bash
cd ~/catkin_ws
python3 tools/simulate_sdk2_direct_bridge.py
```

It checks long right-angle and axis-aligned routes, an overshot waypoint,
nonzero initial heading, two goals in sequence, and a 0.75 s odometry gap.
Every successful case must keep the command interval at or below 0.05 s.
The final `yaw_bias_plus_pi_over_2` and `yaw_sign_inverted` cases are
intentionally expected not to reach: they are diagnostic contrasts for a
body-odometry frame/yaw offset or an inverted physical yaw response, not
controller success criteria. This simulation does not emulate SDK2 firmware
state, joystick ownership, or DDS transport.

Run the normal SLAM and flat-obstacle planning commands first. Only for explicit
short-range, open-ground commissioning, acknowledge that this executor has no
obstacle avoidance and start it instead of `start_sdk2_bridge.sh`:

```bash
cd ~/catkin_ws
export GO2_FASTDDS_PROFILE="$PWD/config/fastdds/jetson_wifi.xml"
source ./shell/ros2_environment.sh
GO2_ALLOW_OBSTACLE_UNAWARE_DIRECT_BRIDGE=true \
  ./shell/start_sdk2_direct_bridge.sh
```

The authorization flag only acknowledges the commissioning risk. It does not
connect this executor to the map, inflation layer, or `/body_path`.

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
