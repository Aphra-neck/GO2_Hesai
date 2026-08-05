#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
NETWORK_INTERFACE="${GO2_NETWORK_INTERFACE:-enP8p1s0}"
UNITREE_SDK_LIBRARY_DIR="${UNITREE_SDK_LIBRARY_DIR:-/usr/local/lib}"
source "${SCRIPT_DIR}/ros2_environment.sh"

if ! ip link show dev "${NETWORK_INTERFACE}" >/dev/null 2>&1; then
  echo "Go2 network interface does not exist: ${NETWORK_INTERFACE}" >&2
  ip -brief address >&2 || true
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
  echo "Unitree CycloneDDS runtime libraries are missing from ${UNITREE_SDK_LIBRARY_DIR}." >&2
  exit 1
fi

if [[ "${RMW_IMPLEMENTATION}" == "rmw_cyclonedds_cpp" ]]; then
  echo "Use rmw_fastrtps_cpp; Unitree SDK2 already owns its CycloneDDS runtime." >&2
  exit 1
fi

if pgrep -f -- "utree_go2_rl_controller|rl_controller_node" >/dev/null 2>&1; then
  echo "An RL low-level controller is running. Do not combine it with SportClient control." >&2
  pgrep -af -- "utree_go2_rl_controller|rl_controller_node" >&2 || true
  exit 1
fi

lowcmd_info="$(
  ros2 topic info --no-daemon --spin-time 3 /lowcmd 2>/dev/null || true
)"
if grep -Eq "Publisher count: [1-9][0-9]*" <<<"${lowcmd_info}"; then
  echo "A /lowcmd publisher is active. Refusing to start the SDK2 SportClient bridge." >&2
  printf '%s\n' "${lowcmd_info}" >&2
  exit 1
fi

if pgrep -f -- "/utree_go2_sdk2_bridge/go2_sdk2_bridge_node" >/dev/null 2>&1; then
  echo "The Go2 SDK2 bridge is already running." >&2
  pgrep -af -- "/utree_go2_sdk2_bridge/go2_sdk2_bridge_node" >&2 || true
  exit 1
fi

if ! timeout 10 ros2 topic echo --once --qos-profile sensor_data \
  /lio/body_odom nav_msgs/msg/Odometry >/dev/null 2>&1; then
  echo "No corrected body odometry on /lio/body_odom. Start terrain navigation first." >&2
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
echo " Motion: disabled until explicitly enabled"
echo "======================================"
echo "After verifying /body_path and /lio/body_odom, enable with:"
echo "ros2 service call /go2_sdk2_bridge/enable_motion std_srvs/srv/SetBool '{data: true}'"

ros2 launch utree_go2_sdk2_bridge go2_sdk2_bridge.launch.py \
  "network_interface:=${NETWORK_INTERFACE}"
