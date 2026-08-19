# SDK2 Simple Navigation

This is a separate commissioning executor. The existing `go2_sdk2_bridge_node`
is unchanged and is preserved at the `backup/pre-simple-nav-20260819` branch.

The simple node consumes only:

- `/goal_pose` (`geometry_msgs/msg/PoseStamped`)
- `/lio/body_odom` (`nav_msgs/msg/Odometry`)

Its launch file starts only the standalone body-odometry adapter needed to
produce `/lio/body_odom` from Super-LIO `/lio/odom`; it does not start the
terrain mapper, lattice planner, or their RViz process.

For a diagonal goal it chooses one of the two Manhattan routes, selects the
first leg closest to the current heading, rotates in place, translates to the
right-angle waypoint, rotates again, translates to the goal, and finally aligns
to the goal yaw. It does not subscribe to `/body_path`, terrain maps, or planner
diagnostics, and it does not expire a goal after a fixed retention period.

The following controls remain deliberately present: explicit
`~/enable_motion`, rejection of an active `/lowcmd` publisher, and a wall-clock
odometry receive watchdog that sends `StopMove()` if feedback stops arriving.
This is a direct, obstacle-unaware commissioning mode and must not be used in
obstacle-rich environments.

Stop the normal terrain-navigation launch before starting this mode. The start
script refuses to run alongside the old planner or another SDK2 executor.

Start it with:

```bash
cd ~/catkin_ws
export GO2_FASTDDS_PROFILE="$PWD/config/fastdds/jetson_wifi.xml"
source ./shell/ros2_environment.sh
./shell/start_sdk2_simple_nav.sh
```

In another terminal, arm it and use RViz's `/goal_pose` publisher:

```bash
cd ~/catkin_ws
export GO2_FASTDDS_PROFILE="$PWD/config/fastdds/jetson_wifi.xml"
source ./shell/ros2_environment.sh
ros2 service call /go2_sdk2_simple_nav/enable_motion \
  std_srvs/srv/SetBool '{data: true}'
```
