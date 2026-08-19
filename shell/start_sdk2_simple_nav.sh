#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
NETWORK_INTERFACE="${GO2_NETWORK_INTERFACE:-enP8p1s0}"
CONFIG="${GO2_SDK2_SIMPLE_NAV_CONFIG:-${WORKSPACE_DIR}/src/utree_go2_sdk2_bridge/config/go2_sdk2_simple_nav.yaml}"

source "${SCRIPT_DIR}/ros2_environment.sh"

export LD_LIBRARY_PATH="/usr/local/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
if [[ ! -r /usr/local/lib/libddsc.so.0 || ! -r /usr/local/lib/libddscxx.so.0 ]]; then
  echo "Unitree SDK2 DDS libraries are missing from /usr/local/lib." >&2
  exit 1
fi

if [[ ! -r "${CONFIG}" ]]; then
  echo "Simple navigation config is not readable: ${CONFIG}" >&2
  exit 1
fi
if pgrep -af -- '/utree_go2_sdk2_bridge/(go2_sdk2_bridge_node|go2_sdk2_simple_nav_node)' >/dev/null 2>&1; then
  echo "An SDK2 motion executor is already running:" >&2
  pgrep -af -- '/utree_go2_sdk2_bridge/(go2_sdk2_bridge_node|go2_sdk2_simple_nav_node)' >&2
  exit 1
fi
if ros2 topic info --no-daemon --spin-time 3 /lowcmd 2>/dev/null |
  awk -F': *' '$1 == "Publisher count" && $2 ~ /^[1-9][0-9]*$/ {found=1} END {exit found ? 0 : 1}'
then
  echo "A /lowcmd publisher is active; refusing to start simple navigation." >&2
  exit 1
fi

echo "======================================"
echo " Go2 SDK2 simple right-angle navigation"
echo " Motion remains disabled until ~/enable_motion"
echo " No terrain map or body-path planner is used"
echo " Goal input: /goal_pose"
echo " Odom input: /lio/body_odom"
echo "======================================"

ros2 launch utree_go2_sdk2_bridge go2_sdk2_simple_nav.launch.py \
  "config:=${CONFIG}" \
  "network_interface:=${NETWORK_INTERFACE}"
