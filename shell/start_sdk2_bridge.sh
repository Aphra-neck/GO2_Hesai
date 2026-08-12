#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
NETWORK_INTERFACE="${GO2_NETWORK_INTERFACE:-enP8p1s0}"
UNITREE_SDK_LIBRARY_DIR="${UNITREE_SDK_LIBRARY_DIR:-/usr/local/lib}"
BRIDGE_CONFIG="${GO2_SDK2_BRIDGE_CONFIG:-${WORKSPACE_DIR}/src/utree_go2_sdk2_bridge/config/go2_sdk2_bridge.yaml}"
source "${SCRIPT_DIR}/ros2_environment.sh"

yaml_number() {
  local name="$1"
  awk -v name="${name}" '
    $1 == name ":" {
      value = $2
      sub(/[[:space:]]*#.*/, "", value)
      print value
      exit
    }
  ' "${BRIDGE_CONFIG}"
}

velocity_limit() {
  local environment_name="$1" yaml_name="$2" value=''
  if [[ -v "${environment_name}" ]]; then
    value="${!environment_name}"
    [[ -n "${value}" ]] || {
      echo "${environment_name} must not be empty when set." >&2
      return 1
    }
    printf '%s|environment: %s\n' "${value}" "${environment_name}"
    return 0
  fi
  value="$(yaml_number "${yaml_name}")"
  [[ -n "${value}" ]] || {
    echo "Missing ${yaml_name} in bridge config: ${BRIDGE_CONFIG}" >&2
    return 1
  }
  printf '%s|YAML: %s\n' "${value}" "${BRIDGE_CONFIG}"
}

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

if ! ip link show dev "${NETWORK_INTERFACE}" >/dev/null 2>&1; then
  echo "Go2 network interface does not exist: ${NETWORK_INTERFACE}" >&2
  ip -brief address >&2 || true
  exit 1
fi

if ! ros2 pkg prefix utree_go2_sdk2_bridge >/dev/null 2>&1; then
  echo "ROS 2 package is not installed: utree_go2_sdk2_bridge" >&2
  exit 1
fi

if [[ ! -r "${BRIDGE_CONFIG}" ]]; then
  echo "SDK2 bridge config is not readable: ${BRIDGE_CONFIG}" >&2
  exit 1
fi

IFS='|' read -r MAX_VX MAX_VX_SOURCE < <(velocity_limit GO2_MAX_VX max_vx)
IFS='|' read -r MAX_VY MAX_VY_SOURCE < <(velocity_limit GO2_MAX_VY max_vy)
IFS='|' read -r MAX_YAW_RATE MAX_YAW_RATE_SOURCE < <(
  velocity_limit GO2_MAX_YAW_RATE max_yaw_rate
)

if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
  LD_LIBRARY_PATH="${UNITREE_SDK_LIBRARY_DIR}:${LD_LIBRARY_PATH}"
else
  LD_LIBRARY_PATH="${UNITREE_SDK_LIBRARY_DIR}"
fi
export LD_LIBRARY_PATH

if [[ ! -r "${UNITREE_SDK_LIBRARY_DIR}/libddsc.so.0" ||
      ! -r "${UNITREE_SDK_LIBRARY_DIR}/libddscxx.so.0" ]]; then
  echo "Unitree CycloneDDS runtime libraries are missing from ${UNITREE_SDK_LIBRARY_DIR}." >&2
  exit 1
fi

if [[ "${RMW_IMPLEMENTATION}" == "rmw_cyclonedds_cpp" ]]; then
  echo "Use rmw_fastrtps_cpp; Unitree SDK2 already owns its CycloneDDS runtime." >&2
  exit 1
fi

if ! rl_processes="$(
    checked_pgrep "utree_go2_rl_controller|rl_controller_node"
  )"; then
  echo "The RL controller process query failed; refusing to start the SDK2 bridge." >&2
  exit 1
fi
if [[ -n "${rl_processes}" ]]; then
  echo "An RL low-level controller is running. Do not combine it with SportClient control." >&2
  echo "${rl_processes}" >&2
  exit 1
fi

if ! lowcmd_publishers="$(lowcmd_publisher_count)"; then
  echo "The /lowcmd publisher check failed; refusing to start the SDK2 bridge." >&2
  exit 1
fi
if (( lowcmd_publishers > 0 )); then
  echo "A /lowcmd publisher is active. Refusing to start the SDK2 SportClient bridge." >&2
  echo "Publisher count: ${lowcmd_publishers}" >&2
  exit 1
fi

if ! bridge_processes="$(
    checked_pgrep "/utree_go2_sdk2_bridge/go2_sdk2_bridge_node"
  )"; then
  echo "The SDK2 bridge process query failed; refusing to start another bridge." >&2
  exit 1
fi
if [[ -n "${bridge_processes}" ]]; then
  echo "The Go2 SDK2 bridge is already running." >&2
  echo "${bridge_processes}" >&2
  exit 1
fi

if ! timeout 10 ros2 topic echo --once --qos-profile sensor_data \
  /lio/body_odom nav_msgs/msg/Odometry >/dev/null 2>&1; then
  echo "No corrected body odometry on /lio/body_odom. Start flat-obstacle navigation first." >&2
  exit 1
fi

if [[ ! -x "${WORKSPACE_DIR}/tools/go2-log" ]]; then
  echo "Diagnostics command is missing or not executable: ${WORKSPACE_DIR}/tools/go2-log" >&2
  exit 1
fi
"${WORKSPACE_DIR}/tools/go2-log" start

echo "======================================"
echo " Go2 SDK2 path executor (ROS 2)"
echo " Network interface: ${NETWORK_INTERFACE}"
echo " ROS domain: ${ROS_DOMAIN_ID} (Unitree SDK domain: 0)"
echo " ROS RMW: ${RMW_IMPLEMENTATION}"
echo " Fast DDS profile: ${FASTRTPS_DEFAULT_PROFILES_FILE}"
echo " ROS localhost only: ${ROS_LOCALHOST_ONLY}"
echo " Motion: disarmed until one explicit operator authorization"
echo " Velocity limits:"
echo "   vx=${MAX_VX} m/s [${MAX_VX_SOURCE}]"
echo "   vy=${MAX_VY} m/s [${MAX_VY_SOURCE}]"
echo "   yaw=${MAX_YAW_RATE} rad/s [${MAX_YAW_RATE_SOURCE}]"
echo "======================================"
echo "In another Jetson terminal, verify the scene and arm once with:"
echo "cd ${WORKSPACE_DIR}"
echo "source ./shell/ros2_environment.sh"
echo "ros2 service call /go2_sdk2_bridge/enable_motion std_srvs/srv/SetBool '{data: true}'"
echo "Normal goal completion keeps this authorization and waits for the next fresh path."

launch_arguments=(
  "config:=${BRIDGE_CONFIG}"
  "network_interface:=${NETWORK_INTERFACE}"
)
[[ -v GO2_MAX_VX ]] && launch_arguments+=("max_vx:=${GO2_MAX_VX}")
[[ -v GO2_MAX_VY ]] && launch_arguments+=("max_vy:=${GO2_MAX_VY}")
[[ -v GO2_MAX_YAW_RATE ]] && launch_arguments+=("max_yaw_rate:=${GO2_MAX_YAW_RATE}")

ros2 launch utree_go2_sdk2_bridge go2_sdk2_bridge.launch.py \
  "${launch_arguments[@]}"
