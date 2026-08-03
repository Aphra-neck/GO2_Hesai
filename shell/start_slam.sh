#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
LOG_DIR="${SLAM_LOG_DIR:-${HOME}/slam_logs}"
NETWORK_INTERFACE="${GO2_NETWORK_INTERFACE:-enP8p1s0}"
IMU_RATE="${GO2_IMU_RATE:-200.0}"
RVIZ="${RVIZ:-false}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
UNITREE_SDK_LIBRARY_DIR="${UNITREE_SDK_LIBRARY_DIR:-/usr/local/lib}"
export ROS_DOMAIN_ID
export RMW_IMPLEMENTATION

# ROS-generated setup scripts may read optional variables that are unset.
set +u
source /opt/ros/humble/setup.bash
set -u

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

if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  echo "Workspace is not built: ${WORKSPACE_DIR}/install/setup.bash is missing." >&2
  echo "Run: colcon build --symlink-install" >&2
  exit 1
fi

set +u
source "${WORKSPACE_DIR}/install/setup.bash"
set -u
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

if ! ros2 pkg prefix "${RMW_IMPLEMENTATION}" >/dev/null 2>&1; then
  echo "ROS 2 RMW package is not installed: ${RMW_IMPLEMENTATION}" >&2
  echo "Install it with: sudo apt install ros-humble-rmw-fastrtps-cpp" >&2
  exit 1
fi

assert_not_running() {
  local name="$1"
  local executable_path="$2"

  if pgrep -f -- "${executable_path}" >/dev/null 2>&1; then
    echo "${name} is already running. Stop the existing process before starting SLAM." >&2
    pgrep -af -- "${executable_path}" >&2 || true
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

declare -a CHILD_PIDS=()
LAST_STARTED_PID=""

cleanup() {
  trap - EXIT INT TERM
  if (( ${#CHILD_PIDS[@]} > 0 )); then
    local pid
    for pid in "${CHILD_PIDS[@]}"; do
      if [[ "${pid}" =~ ^[1-9][0-9]*$ ]] && (( pid > 1 )); then
        kill -TERM -- "-${pid}" 2>/dev/null || true
      fi
    done

    local deadline=$((SECONDS + 3))
    while (( SECONDS < deadline )); do
      local running=false
      for pid in "${CHILD_PIDS[@]}"; do
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

    for pid in "${CHILD_PIDS[@]}"; do
      if kill -0 -- "-${pid}" 2>/dev/null; then
        kill -KILL -- "-${pid}" 2>/dev/null || true
      fi
    done
    wait "${CHILD_PIDS[@]}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

start_background() {
  local name="$1"
  shift
  setsid "$@" >"${LOG_DIR}/${name}.log" 2>&1 &
  LAST_STARTED_PID="$!"
  CHILD_PIDS+=("${LAST_STARTED_PID}")
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
echo " Unitree DDS libraries: ${UNITREE_SDK_LIBRARY_DIR}"
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
echo "Check pose: ros2 topic echo /lio/odom"

ros2 launch super_lio hesai.py "rviz:=${RVIZ}"
