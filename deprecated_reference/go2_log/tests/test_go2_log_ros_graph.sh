#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
GO2_LOG="${REPO_ROOT}/tools/go2-log"
FIXTURE_ROOT="$(mktemp -d)"

cleanup() {
  rm -rf -- "${FIXTURE_ROOT}"
}
trap cleanup EXIT

fake_bin="${FIXTURE_ROOT}/bin"
session_dir="${FIXTURE_ROOT}/session"
trace_file="${FIXTURE_ROOT}/ros2-calls.txt"
timeout_trace_file="${FIXTURE_ROOT}/timeout-calls.txt"
mkdir -p -- "${fake_bin}" "${session_dir}"

cat > "${fake_bin}/ros2" <<'SH'
#!/usr/bin/env bash
set -Eeuo pipefail

printf '%s\n' "$*" >> "${GO2_ROS2_TRACE}"
case "$*" in
  'topic list --no-daemon --spin-time 3 -t')
    echo '/lio/cloud_world [sensor_msgs/msg/PointCloud2]'
    ;;
  'node list --no-daemon --spin-time 3')
    printf '%s\n' '/super_lio_node' '/super_lio_failing_node'
    ;;
  'param dump --no-daemon --spin-time 3 /super_lio_node')
    cat <<'YAML'
/super_lio_node:
  ros__parameters:
    lio.output.dense: true
YAML
    ;;
  'param dump --no-daemon --spin-time 3 /super_lio_failing_node')
    echo 'xmlrpc.client.Fault: !rclpy.ok()' >&2
    exit 97
    ;;
  'topic echo --no-daemon --spin-time 1 --once /sdk2_command geometry_msgs/msg/TwistStamped --field twist')
    cat <<'TWIST'
linear:
  x: 0.1
  y: -0.2
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.3
TWIST
    ;;
  'topic echo --no-daemon --spin-time 3 /rosout')
    cat <<'ROSOUT'
stamp:
  sec: 1
  nanosec: 0
level: 30
name: test_logger
msg: test warning
---
ROSOUT
    ;;
  *)
    echo "unexpected fake ros2 invocation: $*" >&2
    exit 97
    ;;
esac
SH

cat > "${fake_bin}/timeout" <<'SH'
#!/usr/bin/env bash
set -Eeuo pipefail
printf '%s\n' "$*" >> "${GO2_TIMEOUT_TRACE}"
exec /usr/bin/timeout "$@"
SH
chmod +x "${fake_bin}/ros2" "${fake_bin}/timeout"

PATH="${fake_bin}:/usr/bin:/bin" \
GO2_ROS2_TRACE="${trace_file}" \
GO2_TIMEOUT_TRACE="${timeout_trace_file}" \
GO2_WORKSPACE="${REPO_ROOT}" \
  bash -c '
    set -Eeuo pipefail
    source "$1"
    stop_requested=0
    capture_ros_graph "$2" stop_requested
    capture_ros_graph "$2" stop_requested
    printf "timestamp,vx,vy,yaw_rate,status\n" > "$2/sdk2_commands.csv"
    : > "$2/rosout_warn_error.txt"
    sample_sdk2_command "$2"
    run_rosout_collector "$2"
  ' _ "${GO2_LOG}" "${session_dir}"

grep -Fqx 'topic list --no-daemon --spin-time 3 -t' "${trace_file}"
grep -Fqx 'node list --no-daemon --spin-time 3' "${trace_file}"
grep -Fqx 'param dump --no-daemon --spin-time 3 /super_lio_node' \
  "${trace_file}"
grep -Fqx 'topic echo --no-daemon --spin-time 1 --once /sdk2_command geometry_msgs/msg/TwistStamped --field twist' \
  "${trace_file}"
grep -Fqx 'topic echo --no-daemon --spin-time 3 /rosout' "${trace_file}"
grep -Fqx '15 ros2 param dump --no-daemon --spin-time 3 /super_lio_node' \
  "${timeout_trace_file}"
test "$(grep -Fxc 'param dump --no-daemon --spin-time 3 /super_lio_node' \
  "${trace_file}")" -eq 1
test "$(grep -Fxc 'param dump --no-daemon --spin-time 3 /super_lio_failing_node' \
  "${trace_file}")" -eq 2
grep -Fq '/super_lio_node:' \
  "${session_dir}/parameters_super_lio_node.yaml"
grep -Fq 'lio.output.dense: true' \
  "${session_dir}/parameters_super_lio_node.yaml"
test ! -e "${session_dir}/parameters_super_lio_failing_node.yaml"
test ! -e "${session_dir}/parameters_super_lio_failing_node.yaml.tmp"
test "$(grep -Fc 'parameter capture for /super_lio_failing_node failed or was empty (exit 97); will retry' \
  "${session_dir}/events.jsonl")" -eq 2
grep -Fq '0.1,-0.2,0.3,ok' "${session_dir}/sdk2_commands.csv"
grep -Fq 'level: 30' "${session_dir}/rosout_warn_error.txt"

echo "PASS: ROS graph diagnostics are independent of the ROS daemon"
