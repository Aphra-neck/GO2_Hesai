#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
PLANNING_RVIZ="${PLANNING_RVIZ:-false}"
PLANNING_MODE="${GO2_PLANNING_MODE:-flat_obstacle}"
ENABLE_LEGACY_TERRAIN="${GO2_ENABLE_LEGACY_TERRAIN:-false}"
ALLOW_CUSTOM_RVIZ_CONFIG="${GO2_ALLOW_CUSTOM_NAVIGATION_RVIZ_CONFIG:-false}"
FLAT_GROUND_CONFIRMED="${GO2_FLAT_GROUND_CONFIRMED:-false}"
BODY_YAW_OFFSET="${GO2_BODY_YAW_OFFSET_RAD:--1.5707963267948966}"
LIDAR_OFFSET_X="${GO2_LIDAR_OFFSET_X:-0.171}"
LIDAR_OFFSET_Y="${GO2_LIDAR_OFFSET_Y:-0.0}"
LIDAR_OFFSET_Z="${GO2_LIDAR_OFFSET_Z:-0.0908}"
NAVIGATION_CONFIG="${GO2_NAVIGATION_CONFIG:-${WORKSPACE_DIR}/src/utree_dog_navigation/config/terrain_navigation.yaml}"
FLAT_OBSTACLE_RVIZ_CONFIG="${WORKSPACE_DIR}/src/utree_dog_navigation/rviz/flat_obstacle_navigation.rviz"
LEGACY_TERRAIN_RVIZ_CONFIG="${WORKSPACE_DIR}/src/utree_dog_navigation/rviz/hesai_navigation.rviz"
if [[ -n "${GO2_NAVIGATION_RVIZ_CONFIG:-}" ]]; then
  NAVIGATION_RVIZ_CONFIG="${GO2_NAVIGATION_RVIZ_CONFIG}"
elif [[ "${PLANNING_MODE}" == "flat_obstacle" ]]; then
  NAVIGATION_RVIZ_CONFIG="${FLAT_OBSTACLE_RVIZ_CONFIG}"
else
  NAVIGATION_RVIZ_CONFIG="${LEGACY_TERRAIN_RVIZ_CONFIG}"
fi
VERIFIED_FLAT_START="${GO2_VERIFIED_FLAT_START:-false}"
MAP_CAPTURE="${GO2_MAP_CAPTURE:-false}"
MAP_CAPTURE_DIR="${GO2_MAP_CAPTURE_DIR:-${HOME}/go2_map_exports}"
MAP_CAPTURE_MAX_SNAPSHOTS="${GO2_MAP_CAPTURE_MAX_SNAPSHOTS:-120}"
MAP_CAPTURE_MAX_MB="${GO2_MAP_CAPTURE_MAX_MB:-100}"
if SOURCE_GIT_SHA="$(git -C "${WORKSPACE_DIR}" rev-parse --verify HEAD 2>/dev/null)"; then
  :
else
  SOURCE_GIT_SHA="unknown"
fi
GO2_BODY_YAW_OFFSET_RAD="${BODY_YAW_OFFSET}"
export GO2_BODY_YAW_OFFSET_RAD

case "${PLANNING_RVIZ}" in
  true|false) ;;
  *)
    echo "PLANNING_RVIZ must be true or false, got: ${PLANNING_RVIZ}" >&2
    exit 1
    ;;
esac

case "${PLANNING_MODE}" in
  terrain|flat_obstacle) ;;
  *)
    echo "GO2_PLANNING_MODE must be terrain or flat_obstacle, got: ${PLANNING_MODE}" >&2
    exit 1
    ;;
esac

case "${ENABLE_LEGACY_TERRAIN}" in
  true|false) ;;
  *)
    echo \
      "GO2_ENABLE_LEGACY_TERRAIN must be true or false, got: ${ENABLE_LEGACY_TERRAIN}" \
      >&2
    exit 1
    ;;
esac

case "${ALLOW_CUSTOM_RVIZ_CONFIG}" in
  true|false) ;;
  *)
    echo \
      "GO2_ALLOW_CUSTOM_NAVIGATION_RVIZ_CONFIG must be true or false, got: ${ALLOW_CUSTOM_RVIZ_CONFIG}" \
      >&2
    exit 1
    ;;
esac

if [[ "${PLANNING_MODE}" == "terrain" && "${ENABLE_LEGACY_TERRAIN}" != "true" ]]; then
  echo "Legacy terrain mode requires GO2_ENABLE_LEGACY_TERRAIN=true." >&2
  echo "Normal 2D operation uses GO2_PLANNING_MODE=flat_obstacle." >&2
  exit 1
fi

if [[ "${PLANNING_MODE}" != "terrain" && "${ENABLE_LEGACY_TERRAIN}" == "true" ]]; then
  echo "GO2_ENABLE_LEGACY_TERRAIN=true requires GO2_PLANNING_MODE=terrain." >&2
  exit 1
fi

case "${FLAT_GROUND_CONFIRMED}" in
  true|false) ;;
  *)
    echo "GO2_FLAT_GROUND_CONFIRMED must be true or false, got: ${FLAT_GROUND_CONFIRMED}" >&2
    exit 1
    ;;
esac

if [[ "${PLANNING_MODE}" == "flat_obstacle" && "${FLAT_GROUND_CONFIRMED}" != "true" ]]; then
  echo "Flat-obstacle navigation requires the robot to be standing and stationary before startup." >&2
  echo "Stand the robot, keep it still, then set GO2_FLAT_GROUND_CONFIRMED=true." >&2
  exit 1
fi

case "${VERIFIED_FLAT_START}" in
  true|false) ;;
  *)
    echo "GO2_VERIFIED_FLAT_START must be true or false, got: ${VERIFIED_FLAT_START}" >&2
    exit 1
    ;;
esac

case "${MAP_CAPTURE}" in
  true|false) ;;
  *)
    echo "GO2_MAP_CAPTURE must be true or false, got: ${MAP_CAPTURE}" >&2
    exit 1
    ;;
esac

if [[ "${MAP_CAPTURE}" == "true" && "${PLANNING_MODE}" != "flat_obstacle" ]]; then
  echo "GO2_MAP_CAPTURE is available only in flat_obstacle mode." >&2
  exit 1
fi

if [[ ! "${MAP_CAPTURE_MAX_SNAPSHOTS}" =~ ^[1-9][0-9]*$ ]] ||
   [[ ! "${MAP_CAPTURE_MAX_MB}" =~ ^[1-9][0-9]*$ ]]; then
  echo "GO2_MAP_CAPTURE_MAX_SNAPSHOTS and GO2_MAP_CAPTURE_MAX_MB must be positive integers." >&2
  exit 1
fi

if [[ "${MAP_CAPTURE}" == "true" ]]; then
  MAP_CAPTURE_DIR="$(realpath -m -- "${MAP_CAPTURE_DIR}")"
  case "${MAP_CAPTURE_DIR}/" in
    "${WORKSPACE_DIR}/"*)
      echo "GO2_MAP_CAPTURE_DIR must be outside the Git workspace: ${MAP_CAPTURE_DIR}" >&2
      exit 1
      ;;
  esac
fi

for rviz_config in \
  "${FLAT_OBSTACLE_RVIZ_CONFIG}" \
  "${LEGACY_TERRAIN_RVIZ_CONFIG}" \
  "${NAVIGATION_RVIZ_CONFIG}"
do
  if [[ ! -f "${rviz_config}" || ! -r "${rviz_config}" ]]; then
    echo "Navigation RViz configuration is not readable: ${rviz_config}" >&2
    exit 1
  fi
done

if [[ "${PLANNING_MODE}" == "flat_obstacle" ]]; then
  EXPECTED_RVIZ_CONFIG="${FLAT_OBSTACLE_RVIZ_CONFIG}"
  INCOMPATIBLE_RVIZ_CONFIG="${LEGACY_TERRAIN_RVIZ_CONFIG}"
  INCOMPATIBLE_RVIZ_MESSAGE="flat_obstacle mode cannot use legacy terrain RViz config"
else
  EXPECTED_RVIZ_CONFIG="${LEGACY_TERRAIN_RVIZ_CONFIG}"
  INCOMPATIBLE_RVIZ_CONFIG="${FLAT_OBSTACLE_RVIZ_CONFIG}"
  INCOMPATIBLE_RVIZ_MESSAGE="terrain mode cannot use flat-obstacle RViz config"
fi

if cmp -s -- "${NAVIGATION_RVIZ_CONFIG}" "${EXPECTED_RVIZ_CONFIG}"; then
  :
elif cmp -s -- "${NAVIGATION_RVIZ_CONFIG}" "${INCOMPATIBLE_RVIZ_CONFIG}"; then
  echo "${INCOMPATIBLE_RVIZ_MESSAGE}: ${NAVIGATION_RVIZ_CONFIG}" >&2
  exit 1
elif [[ "${ALLOW_CUSTOM_RVIZ_CONFIG}" != "true" ]]; then
  echo \
    "Custom navigation RViz config requires GO2_ALLOW_CUSTOM_NAVIGATION_RVIZ_CONFIG=true: ${NAVIGATION_RVIZ_CONFIG}" \
    >&2
  exit 1
fi

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
   pgrep -f -- "/utree_dog_navigation/body_lattice_planner_node" >/dev/null 2>&1 ||
   pgrep -f -- "/utree_dog_navigation/flat_obstacle_map_recorder.py" >/dev/null 2>&1; then
  echo "Terrain navigation is already running." >&2
  pgrep -af -- \
    "utree_dog_navigation/(body_odom_adapter_node|terrain_mapper_node|body_lattice_planner_node|flat_obstacle_map_recorder.py)" \
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
echo " Go2 navigation (ROS 2)"
echo " ROS domain: ${ROS_DOMAIN_ID}"
echo " ROS RMW: ${RMW_IMPLEMENTATION}"
echo " Fast DDS profile: ${FASTRTPS_DEFAULT_PROFILES_FILE}"
echo " ROS localhost only: ${ROS_LOCALHOST_ONLY}"
echo " Planning RViz: ${PLANNING_RVIZ}"
echo " Planning mode: ${PLANNING_MODE}"
echo " Legacy terrain enabled: ${ENABLE_LEGACY_TERRAIN}"
echo " Custom RViz config authorized: ${ALLOW_CUSTOM_RVIZ_CONFIG}"
echo " Flat ground confirmed: ${FLAT_GROUND_CONFIRMED}"
echo " Verified flat start: ${VERIFIED_FLAT_START}"
echo " Body yaw offset: ${BODY_YAW_OFFSET} rad"
echo " XT-16 offset: (${LIDAR_OFFSET_X}, ${LIDAR_OFFSET_Y}, ${LIDAR_OFFSET_Z}) m"
echo " Navigation config: ${NAVIGATION_CONFIG}"
echo " RViz config: ${NAVIGATION_RVIZ_CONFIG}"
echo " Diagnostic filtered 3D obstacle map capture: ${MAP_CAPTURE}"
if [[ "${MAP_CAPTURE}" == "true" ]]; then
  echo " Filtered map output root: ${MAP_CAPTURE_DIR}"
  echo " Filtered map limits: ${MAP_CAPTURE_MAX_SNAPSHOTS} snapshots, ${MAP_CAPTURE_MAX_MB} MB"
fi
if [[ "${PLANNING_MODE}" == "flat_obstacle" ]]; then
  echo " Robot posture: STANDING and stationary required; never start this mode while prone."
fi
echo "======================================"

echo "Checking Super-LIO inputs..."
wait_for_topic /lio/odom nav_msgs/msg/Odometry 10
wait_for_topic /lio/cloud_world sensor_msgs/msg/PointCloud2 10

if [[ ! -x "${WORKSPACE_DIR}/tools/go2-log" ]]; then
  echo "Diagnostics command is missing or not executable: ${WORKSPACE_DIR}/tools/go2-log" >&2
  exit 1
fi
"${WORKSPACE_DIR}/tools/go2-log" start

echo "Starting ${PLANNING_MODE} mapper and body lattice planner..."
echo "Set a goal with RViz or publish geometry_msgs/msg/PoseStamped to /goal_pose."
ros2 launch utree_dog_navigation terrain_navigation.launch.py \
  "config:=${NAVIGATION_CONFIG}" \
  "rviz:=${PLANNING_RVIZ}" \
  "rviz_config:=${NAVIGATION_RVIZ_CONFIG}" \
  "body_yaw_offset:=${BODY_YAW_OFFSET}" \
  "lidar_offset_x:=${LIDAR_OFFSET_X}" \
  "lidar_offset_y:=${LIDAR_OFFSET_Y}" \
  "lidar_offset_z:=${LIDAR_OFFSET_Z}" \
  "planning_mode:=${PLANNING_MODE}" \
  "enable_legacy_terrain:=${ENABLE_LEGACY_TERRAIN}" \
  "allow_custom_rviz_config:=${ALLOW_CUSTOM_RVIZ_CONFIG}" \
  "flat_ground_confirmed:=${FLAT_GROUND_CONFIRMED}" \
  "verified_flat_start:=${VERIFIED_FLAT_START}" \
  "record_3d_maps:=${MAP_CAPTURE}" \
  "record_3d_maps_output:=${MAP_CAPTURE_DIR}" \
  "record_3d_maps_max_snapshots:=${MAP_CAPTURE_MAX_SNAPSHOTS}" \
  "record_3d_maps_max_megabytes:=${MAP_CAPTURE_MAX_MB}" \
  "record_3d_maps_source_git_sha:=${SOURCE_GIT_SHA}"
