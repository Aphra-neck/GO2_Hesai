# Go2 diagnostics

`go2-log` collects bounded, low-bandwidth diagnostics for a Go2 + Hesai +
Super-LIO/navigation session. It records environment and Git identity,
network state, ROS parameters and graph state, process lifecycle, compact topic
rates, raw and corrected odometry poses, SDK2 command values, and `/rosout`
warning/error records. `odom_position.csv` retains its legacy filename but now
stores the complete sampled `/lio/odom` pose; `body_odom_pose.csv` stores
`/lio/body_odom`. Both contain the ROS timestamp, parent/child frames,
quaternion, and yaw. The sampler subscribes to both topics concurrently and
buffers a bounded number of messages until it finds an identical ROS timestamp,
so the analyzer can compare the actual relative quaternion with the configured
body yaw correction without mixing robot motion between adjacent frames.

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
SLAM/navigation startup. When navigation reuses a session that SLAM started,
the repeated call records `GO2_BODY_YAW_OFFSET_RAD` in that active session so a
short run still has the exact configured correction available to the analyzer.

Commands use an owner lock containing the process ID, Linux boot ID, and
process start time. A lock left by a killed command or system restart is
reclaimed automatically; a lock belonging to a live `go2-log` process is never
removed.

After stopping SLAM, navigation, the Hesai driver, and the SDK2 bridge:

```bash
go2-log stop
go2-log upload
```

If `upload` rejects an old session because an allowlisted collector-owned text
file contains a trailing NUL suffix, repair that stopped session explicitly and
retry the same upload:

```bash
go2-log repair 20260804T010203Z-unitree-1234
go2-log upload 20260804T010203Z-unitree-1234
```

`repair` is deliberately narrow. It runs only while collection and all known
robot processes are stopped, accepts only one contiguous NUL suffix after a
complete UTF-8 line, and refuses symbolic links, middle NUL bytes, incomplete
lines, oversized files, or malformed prior repair records. Before replacing a
file it durably preserves the byte-identical original under
`~/go2_logs/quarantine/<session-id>/`. A recoverable per-file journal handles a
power loss during repair, while the completed evidence record is written to
the session's `repair_manifest.jsonl` and uploaded with the cleaned text. The
normal upload validator remains strict; `repair` does not create a general
binary-file bypass. Files covered by a completed repair record are never
compacted later; if the remaining session cannot fit below the upload ceiling
without changing audited bytes, preserve it with SCP instead of Git.

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

Git authentication comes from the existing `~/G02_log` checkout. The current
Jetson deployment uses a writable GitHub Deploy Key with an SSH remote on port
443. The current repository-local setup is:

```bash
git -C ~/G02_log remote set-url origin \
  git@github.com:Aphra-neck/G02_log.git
git -C ~/G02_log config core.sshCommand \
  "ssh -i /home/unitree/.ssh/g02_log_deploy -o IdentitiesOnly=yes -o HostName=ssh.github.com -p 443"
```

The matching public key must be registered on `Aphra-neck/G02_log` with
**Allow write access** enabled. Before field use, while all robot processes are
stopped, verify actual push permission without changing the remote:

```bash
git -C ~/G02_log remote -v
GIT_TERMINAL_PROMPT=0 git -C ~/G02_log \
  push --dry-run origin HEAD:refs/heads/main
```

The second command must succeed. Unlike `ls-remote` on a public repository,
this checks write authentication. Tokens, private keys, and other credentials
must not be placed in this script, environment snapshots, Git URLs, or
collected logs. If the `G02_log` checkout has no repository-local
`user.name` or `user.email`, `go2-log` copies the missing values from the
current `GO2_Hesai` commit author into `G02_log/.git/config`; it never changes
global Git identity. `GO2_LOG_PROXY` applies only to HTTP(S) Git operations;
an SSH remote does not use that HTTP proxy.

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
upload. Each `topic_rates.csv` row is also followed by a low-frequency
`fdatasync`; this reduces the delayed-write window after an unexpected reboot
without adding synchronous disk I/O to high-rate sensor topics.

## Analyze after pulling on the local computer

Pull `Aphra-neck/G02_log` into `D:\G02_log`, then run the analyzer from a local
`GO2_Hesai` checkout. PowerShell example (using the project proxy):

```powershell
git -C D:\G02_log `
  -c http.proxy=http://192.168.151.143:7890 `
  -c https.proxy=http://192.168.151.143:7890 `
  pull --ff-only origin main

python .\tools\analyze_diagnostics.py `
  D:\G02_log\sessions\20260804T010203Z-unitree-1234
```

It creates `analysis/report.md`, `analysis/topic_rates_summary.csv`,
`analysis/process_health_summary.csv`, and
`analysis/sdk2_command_summary.csv`, plus `analysis/body_odometry_audit.csv`.
The odometry audit compares only pairs with exactly identical ROS timestamps,
then reports the configured and observed relative rotation, frame mismatches,
invalid quaternion samples, and whether the maximum rotation error stayed
within `0.15 rad`. Raw JSONL, CSV, YAML, and text inputs remain unchanged.

## Large artifact transfer

Keep generated maps, bags, and crash dumps outside both Git repositories. For
example, run SCP from Windows PowerShell:

```powershell
scp unitree@192.168.151.213:/home/unitree/catkin_ws/src/Super-LIO/src/super_lio/map/map.pcd `
  D:\GO2_Data\maps\
```
