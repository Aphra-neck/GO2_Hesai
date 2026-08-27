#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
FIXTURE_ROOT="$(mktemp -d)"

cleanup() {
  local fixture_pids=()
  mapfile -t fixture_pids < <(pgrep -f -- "${FIXTURE_ROOT}/workspace" || true)
  if (( ${#fixture_pids[@]} > 0 )); then
    kill -TERM -- "${fixture_pids[@]}" 2>/dev/null || true
  fi
  rm -rf -- "${FIXTURE_ROOT}"
}
trap cleanup EXIT

workspace="${FIXTURE_ROOT}/workspace"
fake_bin="${workspace}/bin"
fake_home="${FIXTURE_ROOT}/home"
unitree_lib="${FIXTURE_ROOT}/unitree-lib"
ros2_trace="${FIXTURE_ROOT}/ros2-calls.txt"

mkdir -p -- \
  "${fake_bin}" \
  "${fake_home}" \
  "${unitree_lib}" \
  "${workspace}/shell" \
  "${workspace}/install"

cp -- "${REPO_ROOT}/shell/start_slam.sh" "${workspace}/shell/start_slam.sh"
chmod +x "${workspace}/shell/start_slam.sh"
touch "${unitree_lib}/libddsc.so.0" "${unitree_lib}/libddscxx.so.0"

cat > "${workspace}/shell/ros2_environment.sh" <<'SH'
#!/usr/bin/env bash
export PATH="${FAKE_BIN}:/usr/bin:/bin"
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fake-fastdds.xml
export ROS_LOCALHOST_ONLY=0
SH

cat > "${fake_bin}/ip" <<'SH'
#!/usr/bin/env bash
exit 0
SH

cat > "${fake_bin}/pgrep" <<'SH'
#!/usr/bin/env bash
set -Eeuo pipefail
if [[ "${FAKE_RL_RUNNING:-false}" == true &&
      "$*" == *"utree_go2_rl_controller"* ]]; then
  echo '4242 utree_go2_rl_controller'
  exit 0
fi
exit 1
SH

cat > "${fake_bin}/timeout" <<'SH'
#!/usr/bin/env bash
set -Eeuo pipefail
shift
exec "$@"
SH

cat > "${fake_bin}/ros2" <<'SH'
#!/usr/bin/env bash
set -Eeuo pipefail
printf '%s\n' "$*" >> "${FAKE_ROS2_TRACE}"
case "$*" in
  'pkg prefix rmw_fastrtps_cpp')
    exit 0
    ;;
  'topic info --no-daemon --spin-time 3 /lowcmd')
    case "${FAKE_LOWCMD_MODE:-none}" in
      active)
        echo 'Publisher count: 1'
        ;;
      unknown)
        echo "Unknown topic '/lowcmd'"
        exit 1
        ;;
      *)
        echo 'Publisher count: 0'
        ;;
    esac
    ;;
  'topic echo --once --qos-profile sensor_data /lidar_points sensor_msgs/msg/PointCloud2'|\
  'topic echo --once --qos-profile sensor_data /imu/data sensor_msgs/msg/Imu')
    exit 0
    ;;
  'run hesai_ros_driver hesai_ros_driver_node '*|\
  'run go2_imu_bridge go2_imu_bridge_node '*)
    trap 'exit 0' INT TERM
    while true; do sleep 1; done
    ;;
  'launch super_lio hesai.py '*)
    exit "${FAKE_LAUNCH_STATUS:-0}"
    ;;
  *)
    echo "unexpected fake ros2 invocation: $*" >&2
    exit 97
    ;;
esac
SH

chmod +x \
  "${fake_bin}/ip" \
  "${fake_bin}/pgrep" \
  "${fake_bin}/ros2" \
  "${fake_bin}/timeout"

run_case() {
  local name="$1"
  local launch_status="$2"
  local lowcmd_mode="${3:-none}"
  local rl_running="${4:-false}"
  local dense_output="${5:-false}"

  : > "${ros2_trace}"
  set +e
  CASE_OUTPUT="$(
    /usr/bin/timeout 10 env \
      HOME="${fake_home}" \
      FAKE_BIN="${fake_bin}" \
      FAKE_ROS2_TRACE="${ros2_trace}" \
      FAKE_LAUNCH_STATUS="${launch_status}" \
      FAKE_LOWCMD_MODE="${lowcmd_mode}" \
      FAKE_RL_RUNNING="${rl_running}" \
      GO2_LIO_DENSE_OUTPUT="${dense_output}" \
      GO2_LOG_ENABLED=true \
      GO2_LOG_AUTO_FINALIZE=true \
      UNITREE_SDK_LIBRARY_DIR="${unitree_lib}" \
      PATH="${fake_bin}:/usr/bin:/bin" \
      "${workspace}/shell/start_slam.sh" 2>&1
  )"
  CASE_STATUS=$?
  set -e
  cp -- "${ros2_trace}" "${FIXTURE_ROOT}/${name}.trace"
}

run_case default 0
test "${CASE_STATUS}" -eq 0
grep -Fq ' External diagnostic upload: not part of the runtime pipeline' \
  <<< "${CASE_OUTPUT}"
grep -Fq 'dense_output:=false' "${FIXTURE_ROOT}/default.trace"
test ! -e "${workspace}/tools/go2-log"

run_case dense 0 none false true
test "${CASE_STATUS}" -eq 0
grep -Fq ' LIO dense cloud output: true' <<< "${CASE_OUTPUT}"
grep -Fq 'dense_output:=true' "${FIXTURE_ROOT}/dense.trace"

run_case launch_failure 23
test "${CASE_STATUS}" -eq 23

run_case lowcmd_active 0 active
test "${CASE_STATUS}" -ne 0
[[ "${CASE_OUTPUT}" == *"A /lowcmd publisher is active"* ]]
! grep -Fq 'run hesai_ros_driver' "${FIXTURE_ROOT}/lowcmd_active.trace"

run_case rl_active 0 none true
test "${CASE_STATUS}" -ne 0
[[ "${CASE_OUTPUT}" == *"An RL low-level controller is running"* ]]
! grep -Fq 'run hesai_ros_driver' "${FIXTURE_ROOT}/rl_active.trace"

if pgrep -f -- "${FIXTURE_ROOT}/workspace" >/dev/null 2>&1; then
  echo "SLAM cleanup test leaked a fixture process" >&2
  pgrep -af -- "${FIXTURE_ROOT}/workspace" >&2 || true
  exit 1
fi

echo "PASS: SLAM startup and cleanup are independent of go2-log and external upload state"
