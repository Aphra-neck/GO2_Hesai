#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
PLANNING_RVIZ="${PLANNING_RVIZ:-false}"
BODY_YAW_OFFSET="${GO2_BODY_YAW_OFFSET_RAD:--1.5707963267948966}"
NAVIGATION_CONFIG="${GO2_NAVIGATION_CONFIG:-${WORKSPACE_DIR}/src/utree_dog_navigation/config/terrain_navigation.yaml}"
GO2_BODY_YAW_OFFSET_RAD="${BODY_YAW_OFFSET}"
export GO2_BODY_YAW_OFFSET_RAD

case "${PLANNING_RVIZ}" in
  true|false) ;;
  *)
    echo "PLANNING_RVIZ must be true or false, got: ${PLANNING_RVIZ}" >&2
    exit 1
    ;;
esac

source "${SCRIPT_DIR}/ros2_environment.sh"

if ! ros2 pkg prefix utree_dog_navigation >/dev/null 2>&1; then
  echo "ROS 2 package is not installed: utree_dog_navigation" >&2
  echo "Rebuild the workspace after pulling the planning packages." >&2
  exit 1
fi

if [[ ! -r "${NAVIGATION_CONFIG}" ]]; then
  echo "Navigation configuration is not readable: ${NAVIGATION_CONFIG}" >&2
  exit 1
fi

if pgrep -f -- "/utree_dog_navigation/body_odom_adapter_node" >/dev/null 2>&1 ||
   pgrep -f -- "/utree_dog_navigation/terrain_mapper_node" >/dev/null 2>&1 ||
   pgrep -f -- "/utree_dog_navigation/body_lattice_planner_node" >/dev/null 2>&1; then
  echo "Terrain navigation is already running." >&2
  pgrep -af -- \
    "utree_dog_navigation/(body_odom_adapter_node|terrain_mapper_node|body_lattice_planner_node)" \
    >&2 || true
  exit 1
fi

if [[ "${PLANNING_RVIZ}" == "true" ]] && pgrep -x rviz2 >/dev/null 2>&1; then
  echo "An RViz2 process is already running; planning owns the only local RViz instance." >&2
  pgrep -a -x rviz2 >&2 || true
  exit 1
fi

wait_for_topic() {
  local topic="$1"
  local message_type="$2"
  local timeout_seconds="$3"

  if ! timeout "${timeout_seconds}" \
    ros2 topic echo --once --qos-profile sensor_data \
      "${topic}" "${message_type}" >/dev/null 2>&1; then
    echo "No data received on ${topic}. Start Super-LIO before planning." >&2
    return 1
  fi
}

echo "======================================"
echo " Go2 terrain navigation (ROS 2)"
echo " ROS domain: ${ROS_DOMAIN_ID}"
echo " ROS RMW: ${RMW_IMPLEMENTATION}"
echo " Fast DDS profile: ${FASTRTPS_DEFAULT_PROFILES_FILE}"
echo " ROS localhost only: ${ROS_LOCALHOST_ONLY}"
echo " Planning RViz: ${PLANNING_RVIZ}"
echo " Body yaw offset: ${BODY_YAW_OFFSET} rad"
echo " Navigation config: ${NAVIGATION_CONFIG}"
echo "======================================"

echo "Checking Super-LIO inputs..."
wait_for_topic /lio/odom nav_msgs/msg/Odometry 10
wait_for_topic /lio/cloud_world sensor_msgs/msg/PointCloud2 10

if [[ ! -x "${WORKSPACE_DIR}/tools/go2-log" ]]; then
  echo "Diagnostics command is missing or not executable: ${WORKSPACE_DIR}/tools/go2-log" >&2
  exit 1
fi
"${WORKSPACE_DIR}/tools/go2-log" start

echo "Starting terrain mapper and body lattice planner..."
echo "Set a goal with RViz or publish geometry_msgs/msg/PoseStamped to /goal_pose."
ros2 launch utree_dog_navigation terrain_navigation.launch.py \
  "config:=${NAVIGATION_CONFIG}" \
  "rviz:=${PLANNING_RVIZ}" \
  "body_yaw_offset:=${BODY_YAW_OFFSET}"
