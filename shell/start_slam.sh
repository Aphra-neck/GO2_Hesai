#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
LOG_DIR="${SLAM_LOG_DIR:-${HOME}/slam_logs}"
DIAGNOSTIC_ROOT="${GO2_LOG_ROOT:-${HOME}/go2_logs}"
NETWORK_INTERFACE="${GO2_NETWORK_INTERFACE:-enP8p1s0}"
IMU_RATE="${GO2_IMU_RATE:-200.0}"
LIO_DENSE_OUTPUT="${GO2_LIO_DENSE_OUTPUT:-false}"
AUTO_FINALIZE_DIAGNOSTICS="${GO2_LOG_AUTO_FINALIZE:-true}"
UNITREE_SDK_LIBRARY_DIR="${UNITREE_SDK_LIBRARY_DIR:-/usr/local/lib}"
source "${SCRIPT_DIR}/ros2_environment.sh"

case "${LIO_DENSE_OUTPUT}" in
  true|false) ;;
  *)
    echo "GO2_LIO_DENSE_OUTPUT must be true or false: ${LIO_DENSE_OUTPUT}" >&2
    exit 1
    ;;
esac
export GO2_LIO_DENSE_OUTPUT="${LIO_DENSE_OUTPUT}"

case "${AUTO_FINALIZE_DIAGNOSTICS}" in
  true|false) ;;
  *)
    echo \
      "GO2_LOG_AUTO_FINALIZE must be true or false: ${AUTO_FINALIZE_DIAGNOSTICS}" \
      >&2
    exit 1
    ;;
esac

if ! command -v setsid >/dev/null 2>&1; then
  echo "The setsid command is required to manage ROS 2 child processes." >&2
  exit 1
fi

if ! ip link show dev "${NETWORK_INTERFACE}" >/dev/null 2>&1; then
  echo "Go2 network interface does not exist: ${NETWORK_INTERFACE}" >&2
  echo "Available interfaces:" >&2
  ip -brief address >&2 || true
  exit 1
fi

mkdir -p "${LOG_DIR}"

# Pair Unitree's libddscxx with the matching libddsc before ROS library paths.
if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
  LD_LIBRARY_PATH="${UNITREE_SDK_LIBRARY_DIR}:${LD_LIBRARY_PATH}"
else
  LD_LIBRARY_PATH="${UNITREE_SDK_LIBRARY_DIR}"
fi
export LD_LIBRARY_PATH

if [[ ! -r "${UNITREE_SDK_LIBRARY_DIR}/libddsc.so.0" || \
      ! -r "${UNITREE_SDK_LIBRARY_DIR}/libddscxx.so.0" ]]; then
  echo "Unitree CycloneDDS runtime libraries are missing from ${UNITREE_SDK_LIBRARY_DIR}." >&2
  exit 1
fi

if [[ "${RMW_IMPLEMENTATION}" == "rmw_cyclonedds_cpp" ]]; then
  echo "rmw_cyclonedds_cpp is incompatible with the bundled Unitree SDK2 CycloneDDS." >&2
  echo "Use: RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >&2
  exit 1
fi

checked_pgrep() {
  local pattern="$1"
  local matches='' status=0
  if matches="$(pgrep -af -- "${pattern}" 2>/dev/null)"; then
    status=0
  else
    status=$?
  fi
  case "${status}" in
    0)
      [[ -n "${matches}" ]] || return 2
      printf '%s\n' "${matches}"
      ;;
    1)
      [[ -z "${matches}" ]] || return 2
      ;;
    *)
      return 2
      ;;
  esac
}

lowcmd_publisher_count() {
  command -v ros2 >/dev/null 2>&1 || {
    echo "ROS 2 command is unavailable while checking /lowcmd publishers." >&2
    return 1
  }

  local lowcmd_info='' lowcmd_status=0 publisher_count=''
  if lowcmd_info="$(
      timeout 8 ros2 topic info --no-daemon --spin-time 3 /lowcmd 2>&1
    )"; then
    lowcmd_status=0
  else
    lowcmd_status=$?
  fi
  if (( lowcmd_status != 0 )); then
    if (( lowcmd_status == 1 )) &&
      [[ "${lowcmd_info}" == "Unknown topic '/lowcmd'" ]]; then
      printf '0\n'
      return 0
    fi
    [[ -z "${lowcmd_info}" ]] || printf '%s\n' "${lowcmd_info}" >&2
    return 1
  fi

  publisher_count="$(
    awk -F': *' '$1 == "Publisher count" {print $2; exit}' <<< "${lowcmd_info}"
  )"
  [[ "${publisher_count}" =~ ^[0-9]+$ ]] || {
    [[ -z "${lowcmd_info}" ]] || printf '%s\n' "${lowcmd_info}" >&2
    return 1
  }
  printf '%s\n' "${publisher_count}"
}

if ! ros2 pkg prefix "${RMW_IMPLEMENTATION}" >/dev/null 2>&1; then
  echo "ROS 2 RMW package is not installed: ${RMW_IMPLEMENTATION}" >&2
  echo "Install it with: sudo apt install ros-humble-rmw-fastrtps-cpp" >&2
  exit 1
fi

if ! rl_processes="$(
    checked_pgrep "utree_go2_rl_controller|rl_controller_node"
  )"; then
  echo "The RL controller process query failed; refusing to start this pipeline." >&2
  exit 1
fi
if [[ -n "${rl_processes}" ]]; then
  echo "An RL low-level controller is running. Stop it before starting this pipeline." >&2
  echo "${rl_processes}" >&2
  exit 1
fi

if ! lowcmd_publishers="$(lowcmd_publisher_count)"; then
  echo "The /lowcmd publisher check failed; refusing to start this pipeline." >&2
  exit 1
fi
if (( lowcmd_publishers > 0 )); then
  echo "A /lowcmd publisher is active. Stop it before starting this pipeline." >&2
  echo "Publisher count: ${lowcmd_publishers}" >&2
  exit 1
fi

assert_not_running() {
  local name="$1"
  local executable_path="$2"

  local processes=''
  if ! processes="$(checked_pgrep "${executable_path}")"; then
    echo "${name} process query failed; refusing to start SLAM." >&2
    exit 1
  fi
  if [[ -n "${processes}" ]]; then
    echo "${name} is already running. Stop the existing process before starting SLAM." >&2
    echo "${processes}" >&2
    exit 1
  fi
}

assert_not_running \
  "Hesai LiDAR driver" \
  "${WORKSPACE_DIR}/install/hesai_ros_driver/lib/hesai_ros_driver/hesai_ros_driver_node"
assert_not_running \
  "Go2 IMU bridge" \
  "${WORKSPACE_DIR}/install/go2_imu_bridge/lib/go2_imu_bridge/go2_imu_bridge_node"
assert_not_running \
  "Super-LIO" \
  "${WORKSPACE_DIR}/install/super_lio/lib/super_lio/super_lio_node"

if [[ ! -x "${WORKSPACE_DIR}/tools/go2-log" ]]; then
  echo "Diagnostics command is missing or not executable: ${WORKSPACE_DIR}/tools/go2-log" >&2
  exit 1
fi

declare -a CHILD_SESSION_PIDS=()
declare -a CHILD_WAITER_PIDS=()
LAST_STARTED_PID=""
DIAGNOSTIC_SESSION_OWNED=false
DIAGNOSTIC_SESSION_ID=""
DIAGNOSTIC_START_RESULT_FILE=""

cleanup_children() {
  if (( ${#CHILD_SESSION_PIDS[@]} > 0 )); then
    local pid
    for pid in "${CHILD_SESSION_PIDS[@]}"; do
      if [[ "${pid}" =~ ^[1-9][0-9]*$ ]] && (( pid > 1 )); then
        kill -TERM -- "-${pid}" 2>/dev/null || true
      fi
    done

    local deadline=$((SECONDS + 3))
    while (( SECONDS < deadline )); do
      local running=false
      for pid in "${CHILD_SESSION_PIDS[@]}"; do
        if kill -0 -- "-${pid}" 2>/dev/null; then
          running=true
          break
        fi
      done
      if [[ "${running}" == false ]]; then
        break
      fi
      sleep 0.1
    done

    for pid in "${CHILD_SESSION_PIDS[@]}"; do
      if kill -0 -- "-${pid}" 2>/dev/null; then
        kill -KILL -- "-${pid}" 2>/dev/null || true
      fi
    done
    if (( ${#CHILD_WAITER_PIDS[@]} > 0 )); then
      wait "${CHILD_WAITER_PIDS[@]}" 2>/dev/null || true
    fi
  fi
}

finalize_owned_diagnostics() {
  [[ "${AUTO_FINALIZE_DIAGNOSTICS}" == true ]] || return 0
  [[ "${DIAGNOSTIC_SESSION_OWNED}" == true ]] || return 0

  echo "Finalizing diagnostic session: ${DIAGNOSTIC_SESSION_ID}"
  local blockers=''
  if ! blockers="$(
      checked_pgrep \
        'hesai_ros_driver_node|go2_imu_bridge_node|super_lio_node|terrain_mapper_node|body_lattice_planner_node|body_odom_adapter_node|go2_sdk2_bridge_node|flat_obstacle_map_recorder.py|utree_go2_rl_controller|rl_controller_node'
    )"; then
    echo \
      "Automatic diagnostic finalization skipped because the robot process query failed; active session retained: ${DIAGNOSTIC_SESSION_ID}" \
      >&2
    return 0
  fi
  if [[ -n "${blockers}" ]]; then
    echo "${blockers}" >&2
    echo \
      "Automatic diagnostic finalization skipped because robot processes are still running; active session retained: ${DIAGNOSTIC_SESSION_ID}" \
      >&2
    return 0
  fi

  local lowcmd_publishers=''
  if ! lowcmd_publishers="$(lowcmd_publisher_count)"; then
    echo \
      "Automatic diagnostic finalization skipped because the /lowcmd publisher check failed; active session retained: ${DIAGNOSTIC_SESSION_ID}" \
      >&2
    return 0
  fi
  if (( lowcmd_publishers > 0 )); then
    echo "Publisher count: ${lowcmd_publishers}" >&2
    echo \
      "Automatic diagnostic finalization skipped because an active /lowcmd publisher was detected; active session retained: ${DIAGNOSTIC_SESSION_ID}" \
      >&2
    return 0
  fi

  if ! "${WORKSPACE_DIR}/tools/go2-log" stop "${DIAGNOSTIC_SESSION_ID}"; then
    echo \
      "Diagnostic stop failed; session retained for an explicit retry: ${DIAGNOSTIC_SESSION_ID}" \
      >&2
    return 0
  fi

  if ! "${WORKSPACE_DIR}/tools/go2-log" repair "${DIAGNOSTIC_SESSION_ID}"; then
    echo \
      "Diagnostic repair/validation failed; session retained: ${DIAGNOSTIC_SESSION_ID}" \
      >&2
    return 0
  fi
  if ! "${WORKSPACE_DIR}/tools/go2-log" upload "${DIAGNOSTIC_SESSION_ID}"; then
    echo \
      "Diagnostic upload failed; session retained: ${DIAGNOSTIC_SESSION_ID}" \
      >&2
    return 0
  fi
}

cleanup() {
  local original_status="${1:-0}"
  trap - EXIT INT TERM
  set +e
  [[ -z "${DIAGNOSTIC_START_RESULT_FILE}" ]] ||
    rm -f -- "${DIAGNOSTIC_START_RESULT_FILE}"
  cleanup_children
  finalize_owned_diagnostics
  exit "${original_status}"
}
trap 'cleanup "$?"' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

DIAGNOSTIC_START_RESULT_FILE="$(
  mktemp -- "${LOG_DIR}/.go2-log-start-result.XXXXXX"
)"
if ! "${WORKSPACE_DIR}/tools/go2-log" start \
  --result-file "${DIAGNOSTIC_START_RESULT_FILE}"; then
  rm -f -- "${DIAGNOSTIC_START_RESULT_FILE}"
  DIAGNOSTIC_START_RESULT_FILE=""
  exit 1
fi
start_result_format=''
start_result_ownership=''
start_result_session_id=''
start_result_session_dir=''
while IFS='=' read -r key value; do
  case "${key}" in
    format) start_result_format="${value}" ;;
    ownership) start_result_ownership="${value}" ;;
    session_id) start_result_session_id="${value}" ;;
    session_dir) start_result_session_dir="${value}" ;;
  esac
done < "${DIAGNOSTIC_START_RESULT_FILE}"
rm -f -- "${DIAGNOSTIC_START_RESULT_FILE}"
DIAGNOSTIC_START_RESULT_FILE=""

if [[ "${start_result_format}" != go2-log-start-v1 ||
      ! "${start_result_ownership}" =~ ^(created|existing)$ ||
      -z "${start_result_session_id}" ||
      "${start_result_session_id}" == */* ||
      "${start_result_session_dir}" != \
        "${DIAGNOSTIC_ROOT}/sessions/${start_result_session_id}" ]]; then
  echo "go2-log returned an invalid machine-readable start result." >&2
  exit 1
fi
if [[ "${start_result_ownership}" == created ]]; then
  DIAGNOSTIC_SESSION_OWNED=true
  DIAGNOSTIC_SESSION_ID="${start_result_session_id}"
fi

start_background() {
  local name="$1"
  shift
  local session_pid_file="${LOG_DIR}/.${name}.session-pid.$$"
  local waiter_pid=""
  local session_pid=""
  rm -f -- "${session_pid_file}"
  setsid --fork --wait bash -c '
    session_pid_file="$1"
    shift
    printf "%s\n" "$$" > "${session_pid_file}"
    exec "$@"
  ' bash "${session_pid_file}" "$@" >"${LOG_DIR}/${name}.log" 2>&1 &
  waiter_pid="$!"

  for _ in {1..100}; do
    if [[ -s "${session_pid_file}" ]]; then
      IFS= read -r session_pid < "${session_pid_file}" || true
      break
    fi
    if ! kill -0 "${waiter_pid}" 2>/dev/null; then
      break
    fi
    sleep 0.01
  done
  rm -f -- "${session_pid_file}"

  if [[ ! "${session_pid}" =~ ^[1-9][0-9]*$ ]] || (( session_pid <= 1 )); then
    wait "${waiter_pid}" 2>/dev/null || true
    echo "Failed to capture the process session for ${name}." >&2
    return 1
  fi

  LAST_STARTED_PID="${waiter_pid}"
  CHILD_SESSION_PIDS+=("${session_pid}")
  CHILD_WAITER_PIDS+=("${waiter_pid}")
}

wait_for_message() {
  local topic="$1"
  local message_type="$2"
  local timeout_seconds="$3"
  local producer_pid="$4"
  local probe_status=0

  # Supplying the type lets the subscriber start before DDS discovers the publisher.
  timeout "${timeout_seconds}" \
    ros2 topic echo --once --qos-profile sensor_data \
      "${topic}" "${message_type}" >/dev/null 2>&1 || probe_status=$?
  if (( probe_status == 0 )); then
    return 0
  fi

  if ! kill -0 "${producer_pid}" 2>/dev/null; then
    local exit_status=0
    wait "${producer_pid}" || exit_status=$?
    echo \
      "Producer for ${topic} exited with status ${exit_status}. See ${LOG_DIR}." >&2
    return 1
  fi

  if (( probe_status == 124 )); then
    echo "Timed out waiting for data on ${topic}. See ${LOG_DIR}." >&2
  else
    echo \
      "Topic probe for ${topic} exited with status ${probe_status}. See ${LOG_DIR}." >&2
  fi
  return 1
}

echo "======================================"
echo " Go2 + Hesai XT-16 + Super-LIO (ROS 2)"
echo " ROS domain: ${ROS_DOMAIN_ID} (Unitree SDK domain: 0)"
echo " ROS RMW: ${RMW_IMPLEMENTATION}"
echo " Fast DDS profile: ${FASTRTPS_DEFAULT_PROFILES_FILE}"
echo " ROS localhost only: ${ROS_LOCALHOST_ONLY}"
echo " Unitree DDS libraries: ${UNITREE_SDK_LIBRARY_DIR}"
echo " LIO dense cloud output: ${LIO_DENSE_OUTPUT}"
echo " Diagnostic auto-finalize on exit: ${AUTO_FINALIZE_DIAGNOSTICS}"
echo "======================================"

echo "[1/3] Starting Hesai LiDAR driver..."
start_background hesai \
  ros2 run hesai_ros_driver hesai_ros_driver_node --ros-args \
  -p "config_path:=${WORKSPACE_DIR}/src/HesaiLidar_ROS_2.0/config/config.yaml"
wait_for_message \
  /lidar_points sensor_msgs/msg/PointCloud2 20 "${LAST_STARTED_PID}"
echo "/lidar_points is active."

echo "[2/3] Starting Go2 IMU bridge on ${NETWORK_INTERFACE}..."
start_background go2_imu_bridge \
  ros2 run go2_imu_bridge go2_imu_bridge_node --ros-args \
  -p "net:=${NETWORK_INTERFACE}" \
  -p "publish_rate:=${IMU_RATE}"
wait_for_message /imu/data sensor_msgs/msg/Imu 15 "${LAST_STARTED_PID}"
echo "/imu/data is active."

echo "[3/3] Starting Super-LIO..."
echo "Logs: ${LOG_DIR}"
echo "Diagnostics: ${DIAGNOSTIC_ROOT}/sessions"
echo "Check pose: ros2 topic echo /lio/odom"

ros2 launch super_lio hesai.py \
  rviz:=false \
  dense_output:="${LIO_DENSE_OUTPUT}"
