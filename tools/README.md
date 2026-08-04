# Go2 diagnostics

`go2-log` collects bounded, low-bandwidth diagnostics for a Go2 + Hesai +
Super-LIO/navigation session. It records environment and Git identity,
network state, ROS parameters and graph state, process lifecycle, compact topic
rates, odometry position, SDK2 command values, and `/rosout` warning/error
records.

It never subscribes to or copies `/lidar_points`, `/lio/cloud_world`, or full
`TerrainGrid` messages. PCD (including compressed PCD), rosbag/MCAP, packet
captures, archives, core files, binary payloads, and individual files larger
than 40 MiB are rejected by the Git upload command. The source, copied, and
staged forms of a session are all validated. Transfer large artifacts with SCP
into `D:\GO2_Data\maps`, `D:\GO2_Data\bags`, or `D:\GO2_Data\cores` instead.

## Install on the Jetson

From the `GO2_Hesai` workspace root:

```bash
chmod +x tools/go2-log tools/analyze_diagnostics.py
sudo install -m 0755 tools/go2-log /usr/local/bin/go2-log
```

The installed command continues to use `~/go2_logs/sessions` and does not need
write access to the source checkout. Run it from the workspace root as shown
below, or set `GO2_WORKSPACE=~/catkin_ws` if invoking it from another directory.

## Session workflow

Source ROS and the built workspace, then start diagnostics before starting the
robot session:

```bash
cd ~/catkin_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

go2-log start
go2-log status
```

`start` is idempotent: if a healthy collector already owns the active session,
it prints that session and exits successfully without creating another one.
This makes it safe for a launch wrapper to call `go2-log start` before every
SLAM/navigation startup.

Commands use an owner lock containing the process ID, Linux boot ID, and
process start time. A lock left by a killed command or system restart is
reclaimed automatically; a lock belonging to a live `go2-log` process is never
removed.

After stopping SLAM, navigation, the Hesai driver, and the SDK2 bridge:

```bash
go2-log stop
go2-log upload
```

`upload` defaults to the newest unuploaded session. A particular stopped
session can be selected explicitly:

```bash
go2-log upload 20260804T010203Z-unitree-1234
```

The upload checkout defaults to `~/G02_log`, the target branch defaults to
`main`, and Git HTTP(S) operations use `http://192.168.151.143:7890`. Override
the proxy or checkout without changing the script:

```bash
export GO2_LOG_PROXY=http://192.168.151.143:7890
export GO2_LOG_REPO=~/G02_log
```

For an existing checkout, both the fetch and push URLs of `origin` must identify
the same repository as `GO2_LOG_REMOTE`. Common HTTPS and SSH forms of the same
repository are treated as equivalent; a different repository, non-default URL
port, query string, fragment, or embedded HTTP credential is rejected before
any session data is copied. This prevents a stale `~/G02_log` directory from
silently sending logs to another repository.

Git authentication comes from the machine's normal credential helper. Tokens
and credentials must not be placed in this script, environment snapshots, Git
URLs, or collected logs. If the `G02_log` checkout has no repository-local
`user.name` or `user.email`, `go2-log` copies the missing values from the
current `GO2_Hesai` commit author into `G02_log/.git/config`; it never changes
global Git identity.

The command refuses to push while any known lidar, SLAM, planner, IMU bridge,
SDK2 motion bridge, or RL controller process is running. It checks once before
repository preparation and again immediately before `git push`. A failed
clone, copy, commit, push, or remote verification leaves the source session
intact. Retrying the same `go2-log upload SESSION_ID` safely replaces only that
session's partial files or staged changes; unrelated worktree changes and local
commits are rejected rather than hidden or overwritten. Old sessions are
removed only after their pushed commit has been verified against the configured
remote; unuploaded sessions are never deleted even if that temporarily leaves
more than 20 local sessions.

Collection stops with substantial headroom below 100 MiB for concurrently
running ROS log and parameter captures. Every capture has its own ceiling, and
collector-owned text is deterministically compacted to at most 94 MiB when the
session closes or before upload. This keeps a normal collector session within
the 100 MiB Git limit instead of leaving an oversized, unuploadable session.
Unknown or explicitly forbidden artifacts are never truncated to make them fit;
they must be preserved with SCP and removed from the diagnostics session before
upload.

## Analyze after pulling on the local computer

Pull `Aphra-neck/G02_log` into `D:\G02_log`, then run the analyzer from a local
`GO2_Hesai` checkout. PowerShell example:

```powershell
python .\tools\analyze_diagnostics.py `
  D:\G02_log\sessions\20260804T010203Z-unitree-1234
```

It creates `analysis/report.md`, `analysis/topic_rates_summary.csv`,
`analysis/process_health_summary.csv`, and
`analysis/sdk2_command_summary.csv`. Raw JSONL, CSV, YAML, and text inputs remain
unchanged.

## Large artifact transfer

Keep generated maps, bags, and crash dumps outside both Git repositories. For
example, run SCP from Windows PowerShell:

```powershell
scp unitree@192.168.151.213:/home/unitree/catkin_ws/src/Super-LIO/src/super_lio/map/map.pcd `
  D:\GO2_Data\maps\
```
