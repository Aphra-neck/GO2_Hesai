#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
SOURCE_DIR="${SCRIPT_DIR}/sdk2_motion_stall_probe"
BUILD_PARENT="${GO2_STALL_PROBE_BUILD_PARENT:-/tmp}"
BUILD_DIR=''
capture_dir=''
NETWORK_INTERFACE="${GO2_NETWORK_INTERFACE:-enP8p1s0}"
UNITREE_SDK_LIBRARY_DIR="${UNITREE_SDK_LIBRARY_DIR:-/usr/local/lib}"
LOG_ROOT="${GO2_LOG_ROOT:-${HOME}/go2_logs}"
ACTIVE_SESSION_FILE="${LOG_ROOT}/active_session"
CAPTURE_MAX_BYTES=$((24 * 1024 * 1024))
CAPTURE_METADATA_HEADROOM_BYTES=$((4 * 1024 * 1024))
SESSION_FINAL_BYTES=$((94 * 1024 * 1024))

cleanup() {
  if [[ -n "${capture_dir}" ]]; then
    rmdir -- "${capture_dir}" 2>/dev/null || true
  fi
  if [[ -n "${BUILD_DIR}" && -d "${BUILD_DIR}" && ! -L "${BUILD_DIR}" ]]; then
    rm -rf -- "${BUILD_DIR}"
  fi
}
trap cleanup EXIT

source "${WORKSPACE_DIR}/shell/ros2_environment.sh"

if ! ip link show dev "${NETWORK_INTERFACE}" >/dev/null 2>&1; then
  echo "Go2 network interface does not exist: ${NETWORK_INTERFACE}" >&2
  exit 1
fi
if [[ ! -r "${UNITREE_SDK_LIBRARY_DIR}/libddsc.so.0" ||
      ! -r "${UNITREE_SDK_LIBRARY_DIR}/libddscxx.so.0" ]]; then
  echo "Unitree SDK2 runtime libraries are missing from ${UNITREE_SDK_LIBRARY_DIR}." >&2
  exit 1
fi
if [[ "${RMW_IMPLEMENTATION}" == "rmw_cyclonedds_cpp" ]]; then
  echo "Use rmw_fastrtps_cpp; the read-only SDK2 reader owns its CycloneDDS runtime." >&2
  exit 1
fi
probe_arguments=()
while (( $# > 0 )); do
  case "$1" in
    --duration|--discovery-timeout)
      if (( $# < 2 )); then
        echo "Missing value for stall-probe argument: $1" >&2
        exit 1
      fi
      probe_arguments+=("$1" "$2")
      shift 2
      ;;
    --duration=*|--discovery-timeout=*)
      probe_arguments+=("$1")
      shift
      ;;
    *)
      echo "Unsupported stall-probe argument: $1" >&2
      exit 1
      ;;
  esac
done
if [[ ! -r "${ACTIVE_SESSION_FILE}" || -L "${ACTIVE_SESSION_FILE}" ]]; then
  echo "No active diagnostic session; start SLAM/planning before running the stall probe." >&2
  exit 1
fi
IFS= read -r active_session_entry < "${ACTIVE_SESSION_FILE}"
if [[ -z "${active_session_entry}" || ! -d "${active_session_entry}" ||
      -L "${active_session_entry}" ]]; then
  echo "Active diagnostic session is an invalid or symbolic-link session directory: ${active_session_entry}" >&2
  exit 1
fi
sessions_root="$(realpath -- "${LOG_ROOT}/sessions")"
active_session="$(realpath -- "${active_session_entry}")"
if [[ "$(dirname -- "${active_session}")" != "${sessions_root}" ]]; then
  echo "Active diagnostic session is not a direct child of ${sessions_root}: ${active_session}" >&2
  exit 1
fi
if [[ ! -d "${active_session}" || -e "${active_session}/.uploaded" ||
      -L "${active_session}/.uploaded" ||
      -e "${active_session}/ended_at.txt" ||
      -L "${active_session}/ended_at.txt" ]]; then
  echo "Active diagnostic session is invalid: ${active_session}" >&2
  exit 1
fi
collector_pid=''
if [[ -f "${active_session}/collector.pid" &&
      ! -L "${active_session}/collector.pid" ]]; then
  IFS= read -r collector_pid < "${active_session}/collector.pid"
fi
if [[ ! "${collector_pid}" =~ ^[1-9][0-9]*$ ]] ||
   ! kill -0 "${collector_pid}" 2>/dev/null ||
   [[ ! -r "/proc/${collector_pid}/cmdline" ]]; then
  echo "Active diagnostic collector is not running for ${active_session}." >&2
  exit 1
fi
collector_command="$(tr '\0' ' ' < "/proc/${collector_pid}/cmdline")"
if [[ "${collector_command}" != *"go2-log"* ||
      "${collector_command}" != *"_collect"* ||
      "${collector_command}" != *"${active_session}"* ]]; then
  echo "Active diagnostic collector identity does not match ${active_session}." >&2
  exit 1
fi
session_size="$(du -sb -- "${active_session}" | awk '{print $1}')"
if [[ ! "${session_size}" =~ ^[0-9]+$ ]] ||
   (( session_size + CAPTURE_MAX_BYTES + CAPTURE_METADATA_HEADROOM_BYTES >
      SESSION_FINAL_BYTES )); then
  echo "Active diagnostic session lacks reserved capacity for a bounded stall probe." >&2
  exit 1
fi
capture_dir="${active_session}/sdk2_motion_stall_$(date -u +%Y%m%dT%H%M%SZ)_$$"
if ! mkdir -m 0700 -- "${capture_dir}"; then
  echo "Could not reserve stall-probe capture directory: ${capture_dir}" >&2
  exit 1
fi
if [[ ! -f "${ACTIVE_SESSION_FILE}" || -L "${ACTIVE_SESSION_FILE}" ]]; then
  rmdir -- "${capture_dir}" 2>/dev/null || true
  echo "Active diagnostic session changed while reserving the capture directory." >&2
  exit 1
fi
IFS= read -r current_active_entry < "${ACTIVE_SESSION_FILE}"
if [[ -z "${current_active_entry}" || ! -d "${current_active_entry}" ||
      -L "${current_active_entry}" ||
      "$(realpath -- "${current_active_entry}")" != "${active_session}" ||
      -e "${active_session}/.uploaded" ||
      -L "${active_session}/.uploaded" ||
      -e "${active_session}/ended_at.txt" ||
      -L "${active_session}/ended_at.txt" ]]; then
  rmdir -- "${capture_dir}" 2>/dev/null || true
  echo "Active diagnostic session changed while reserving the capture directory." >&2
  exit 1
fi

if [[ ! -d "${BUILD_PARENT}" || -L "${BUILD_PARENT}" ]]; then
  echo "Stall-probe build parent is invalid: ${BUILD_PARENT}" >&2
  exit 1
fi
BUILD_PARENT="$(realpath -- "${BUILD_PARENT}")"
BUILD_DIR="$(mktemp -d -- "${BUILD_PARENT}/go2-sdk2-motion-stall-build.XXXXXX")"
if [[ "$(stat -c '%u' -- "${BUILD_DIR}")" != "$(id -u)" ||
      "$(stat -c '%a' -- "${BUILD_DIR}")" != '700' ]]; then
  echo "Stall-probe build directory ownership or permissions are unsafe: ${BUILD_DIR}" >&2
  exit 1
fi

cmake -S "${SOURCE_DIR}" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE=Release
cmake --build "${BUILD_DIR}" --parallel 2

post_build_error=''
current_active_entry=''
if [[ ! -f "${ACTIVE_SESSION_FILE}" || -L "${ACTIVE_SESSION_FILE}" ]]; then
  post_build_error='the active-session marker is missing or symbolic-linked'
else
  IFS= read -r current_active_entry < "${ACTIVE_SESSION_FILE}"
fi
if [[ -z "${post_build_error}" ]] &&
   [[ -z "${current_active_entry}" || ! -d "${current_active_entry}" ||
      -L "${current_active_entry}" ||
      "$(realpath -- "${current_active_entry}")" != "${active_session}" ||
      -e "${active_session}/.uploaded" ||
      -L "${active_session}/.uploaded" ||
      -e "${active_session}/ended_at.txt" ||
      -L "${active_session}/ended_at.txt" ]]; then
  post_build_error='the active session ended, changed, or became invalid'
fi
post_build_collector_pid=''
if [[ -z "${post_build_error}" &&
      -f "${active_session}/collector.pid" &&
      ! -L "${active_session}/collector.pid" ]]; then
  IFS= read -r post_build_collector_pid < "${active_session}/collector.pid"
fi
if [[ -z "${post_build_error}" ]] &&
   { [[ "${post_build_collector_pid}" != "${collector_pid}" ]] ||
     ! kill -0 "${post_build_collector_pid}" 2>/dev/null ||
     [[ ! -r "/proc/${post_build_collector_pid}/cmdline" ]]; }; then
  post_build_error='the diagnostic collector stopped or changed identity'
fi
if [[ -z "${post_build_error}" ]]; then
  post_build_collector_command="$(
    tr '\0' ' ' < "/proc/${post_build_collector_pid}/cmdline"
  )"
  if [[ "${post_build_collector_command}" != *"go2-log"* ||
        "${post_build_collector_command}" != *"_collect"* ||
        "${post_build_collector_command}" != *"${active_session}"* ]]; then
    post_build_error='the diagnostic collector command no longer matches'
  fi
fi
if [[ -z "${post_build_error}" ]]; then
  post_build_session_size="$(du -sb -- "${active_session}" | awk '{print $1}')"
  if [[ ! "${post_build_session_size}" =~ ^[0-9]+$ ]] ||
     (( post_build_session_size + CAPTURE_MAX_BYTES +
        CAPTURE_METADATA_HEADROOM_BYTES > SESSION_FINAL_BYTES )); then
    post_build_error='the active session no longer has reserved capacity'
  fi
fi
if [[ -z "${post_build_error}" ]] &&
   [[ ! -d "${capture_dir}" || -L "${capture_dir}" ]]; then
  post_build_error='the reserved capture directory changed'
fi
if [[ -n "${post_build_error}" ]]; then
  echo "Active diagnostic session changed during the probe build: ${post_build_error}." >&2
  exit 1
fi

reader="${BUILD_DIR}/go2_sdk2_motion_stall_reader"
if [[ ! -x "${reader}" ]]; then
  echo "Probe build did not produce an executable reader: ${reader}" >&2
  exit 1
fi

if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
  export LD_LIBRARY_PATH="${UNITREE_SDK_LIBRARY_DIR}:${LD_LIBRARY_PATH}"
else
  export LD_LIBRARY_PATH="${UNITREE_SDK_LIBRARY_DIR}"
fi

python3 "${SCRIPT_DIR}/sdk2_motion_stall_probe.py" \
  --reader "${reader}" \
  --interface "${NETWORK_INTERFACE}" \
  --output-dir "${capture_dir}" \
  --active-session-file "${ACTIVE_SESSION_FILE}" \
  --expected-session "${active_session}" \
  --collector-pid "${collector_pid}" \
  "${probe_arguments[@]}"
