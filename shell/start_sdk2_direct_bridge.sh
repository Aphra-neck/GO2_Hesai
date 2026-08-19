#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
NETWORK_INTERFACE="${GO2_NETWORK_INTERFACE:-enP8p1s0}"
UNITREE_SDK_LIBRARY_DIR="${UNITREE_SDK_LIBRARY_DIR:-/usr/local/lib}"
CONFIG="${GO2_SDK2_DIRECT_BRIDGE_CONFIG:-${WORKSPACE_DIR}/src/utree_go2_sdk2_bridge/config/go2_sdk2_direct_bridge.yaml}"

source "${SCRIPT_DIR}/ros2_environment.sh"

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
  local info='' status=0 count=''
  if info="$(timeout 8 ros2 topic info --no-daemon --spin-time 3 /lowcmd 2>&1)"; then
    status=0
  else
    status=$?
  fi
  if (( status != 0 )); then
    if (( status == 1 )) && [[ "${info}" == "Unknown topic '/lowcmd'" ]]; then
      printf '0\n'
      return 0
    fi
    [[ -z "${info}" ]] || printf '%s\n' "${info}" >&2
    return 1
  fi
  count="$(awk -F': *' '$1 == "Publisher count" {print $2; exit}' <<< "${info}")"
  [[ "${count}" =~ ^[0-9]+$ ]] || return 1
  printf '%s\n' "${count}"
}

if ! ip link show dev "${NETWORK_INTERFACE}" >/dev/null 2>&1; then
  echo "Go2 network interface does not exist: ${NETWORK_INTERFACE}" >&2
  exit 1
fi
if ! ros2 pkg prefix utree_go2_sdk2_bridge >/dev/null 2>&1; then
  echo "ROS 2 package is not installed: utree_go2_sdk2_bridge" >&2
  exit 1
fi
if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
  LD_LIBRARY_PATH="${UNITREE_SDK_LIBRARY_DIR}:${LD_LIBRARY_PATH}"
else
  LD_LIBRARY_PATH="${UNITREE_SDK_LIBRARY_DIR}"
fi
export LD_LIBRARY_PATH
if [[ ! -r "${UNITREE_SDK_LIBRARY_DIR}/libddsc.so.0" ||
      ! -r "${UNITREE_SDK_LIBRARY_DIR}/libddscxx.so.0" ]]; then
  echo "Unitree SDK2 DDS libraries are missing from ${UNITREE_SDK_LIBRARY_DIR}." >&2
  exit 1
fi
if [[ ! -r "${CONFIG}" ]]; then
  echo "Direct bridge config is not readable: ${CONFIG}" >&2
  exit 1
fi

executor_pattern='/utree_go2_sdk2_bridge/(go2_sdk2_bridge_node|go2_sdk2_direct_bridge_node|go2_sdk2_simple_nav_node)'
if ! bridge_processes="$(checked_pgrep "${executor_pattern}")"; then
  echo "The SDK2 bridge process query failed; refusing to start." >&2
  exit 1
fi
if [[ -n "${bridge_processes}" ]]; then
  echo "An SDK2 motion bridge is already running:" >&2
  echo "${bridge_processes}" >&2
  exit 1
fi
if ! lowcmd_publishers="$(lowcmd_publisher_count)"; then
  echo "The /lowcmd publisher check failed; refusing to start the direct bridge." >&2
  exit 1
fi
if (( lowcmd_publishers > 0 )); then
  echo "A /lowcmd publisher is active; refusing to start the direct bridge." >&2
  exit 1
fi
if ! timeout 10 ros2 topic echo --once --qos-profile sensor_data \
  /lio/body_odom nav_msgs/msg/Odometry >/dev/null 2>&1; then
  echo "No body odometry on /lio/body_odom. Keep terrain navigation running first." >&2
  exit 1
fi

echo "======================================"
echo " Go2 SDK2 direct goal bridge"
echo " Terrain mapping/planning remains running but does not control motion"
echo " Motion remains disabled until ~/enable_motion"
echo " Goal input: /goal_pose"
echo " Odom input: /lio/body_odom"
echo " Ignored motion input: /body_path"
echo "======================================"

ros2 launch utree_go2_sdk2_bridge go2_sdk2_direct_bridge.launch.py \
  "config:=${CONFIG}" \
  "network_interface:=${NETWORK_INTERFACE}"
