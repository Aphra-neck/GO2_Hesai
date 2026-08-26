# SDK2 path execution mode

The standard go2_sdk2_bridge keeps the planner and the SDK2 motion surface
separate:

1. body_lattice_planner publishes /body_path.
2. The bridge reads /body_path and /lio/body_odom.
3. After one explicit ~/enable_motion authorization, the control timer calls
   SportClient::Move(vx, vy, vyaw) directly every 5 ms (200 Hz by default),
   matching Unitree's official recurrent velocity example.
4. The bridge does not chase each 0.20 m lattice edge independently. It samples
   eight points over the next 0.35 m path prefix and maximizes a discounted
   local surrogate direction. Intermediate same-position lattice yaw states
   are skipped, so a corner produces one continuous forward arc.
5. Translation sends Move(0.20, 0, 0). At a new local route heading the bridge
   follows Unitree's combined-velocity example with Move(0.20, 0, +/-0.30)
   until aligned, then resumes the fixed straight command.
6. At the final pose it continues sending Move(0, 0, 0) while remaining
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
An accepted goal is retained for 120 seconds by default; fresh map and odometry
watchdogs remain active throughout execution.
