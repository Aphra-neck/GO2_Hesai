# SDK2 path execution mode

The standard go2_sdk2_bridge keeps the planner and the SDK2 motion surface
separate:

1. body_lattice_planner publishes /body_path.
2. The bridge reads /body_path and /lio/body_odom.
3. After one explicit ~/enable_motion authorization, the control timer calls
   SportClient::Move(vx, vy, vyaw) directly at command_rate (20 Hz by
   default).
4. Translation commands have a fixed planar speed of translation_speed
   (0.20 m/s by default). At a new route heading the bridge sends a
   zero-planar, non-zero-yaw Move until the heading is aligned, then resumes
   translation.
5. At the final pose it continues sending Move(0, 0, 0) while remaining
   armed. A path with a new /goal_pose generation resumes execution.

The bridge does not use the former asynchronous SDK mailbox, cross-track
parking gate, bounded-progress parking gate, direction-conflict disarm, or
no-motion watchdog. A same-goal planner refresh is re-anchored to the current
odometry and does not reset the route to an already crossed short connector.

StopMove() is reserved for SDK call failure, missing/stale path or odometry
input, an active /lowcmd publisher, an explicit disable request, and node
shutdown. An empty path and a completed path are normal zero-speed wait states;
they do not revoke the operator authorization.

The planner's obstacle map and footprint/inflation logic are unchanged.
