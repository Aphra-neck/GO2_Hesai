#!/usr/bin/env bash

# Source this file to prepare one Jetson terminal for the project ROS 2 graph.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "Source this file instead of executing it:" >&2
  echo "  source ./shell/ros2_environment.sh" >&2
  exit 2
fi

_go2_prepare_jetson_ros2() {
  local script_dir workspace_dir profile nounset_was_set
  script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
  workspace_dir="$(cd -- "${script_dir}/.." && pwd)"
  profile="${GO2_FASTDDS_PROFILE:-${workspace_dir}/config/fastdds/jetson_wifi.xml}"

  if [[ ! -r /opt/ros/humble/setup.bash ]]; then
    echo "ROS 2 Humble setup is missing: /opt/ros/humble/setup.bash" >&2
    return 1
  fi

  if [[ ! -r "${workspace_dir}/install/setup.bash" ]]; then
    echo "Workspace is not built: ${workspace_dir}/install/setup.bash is missing." >&2
    echo "Run: colcon build --symlink-install" >&2
    return 1
  fi

  if [[ ! -r "${profile}" ]]; then
    echo "Fast DDS profile is not readable: ${profile}" >&2
    return 1
  fi

  unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT CYCLONEDDS_URI \
    FASTDDS_DEFAULT_PROFILES_FILE
  export FASTRTPS_DEFAULT_PROFILES_FILE="${profile}"
  export ROS_DOMAIN_ID=30
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ROS_LOCALHOST_ONLY=0

  # ROS-generated setup scripts may read optional variables that are unset.
  nounset_was_set=false
  case "$-" in
    *u*) nounset_was_set=true ;;
  esac
  set +u
  if ! source /opt/ros/humble/setup.bash; then
    if [[ "${nounset_was_set}" == true ]]; then
      set -u
    fi
    echo "Failed to source /opt/ros/humble/setup.bash" >&2
    return 1
  fi
  if ! source "${workspace_dir}/install/setup.bash"; then
    if [[ "${nounset_was_set}" == true ]]; then
      set -u
    fi
    echo "Failed to source ${workspace_dir}/install/setup.bash" >&2
    return 1
  fi
  if [[ "${nounset_was_set}" == true ]]; then
    set -u
  fi

  if ! command -v ros2 >/dev/null 2>&1; then
    echo "ros2 is unavailable after sourcing the Humble environment." >&2
    return 1
  fi

  # Drop daemon state created with an old domain, RMW, or discovery configuration.
  ros2 daemon stop >/dev/null 2>&1 || true
}

if ! _go2_prepare_jetson_ros2; then
  unset -f _go2_prepare_jetson_ros2
  return 1
fi
unset -f _go2_prepare_jetson_ros2
