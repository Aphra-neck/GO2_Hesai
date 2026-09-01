#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
FIXTURE_ROOT="$(mktemp -d)"

cleanup() {
  local fixture_pids=()
  mapfile -t fixture_pids < <(pgrep -f -- "${FIXTURE_ROOT}/workspace" || true)
  if (( ${#fixture_pids[@]} > 0 )); then
    kill -TERM -- "${fixture_pids[@]}" 2>/dev/null || true
    sleep 0.1
    kill -KILL -- "${fixture_pids[@]}" 2>/dev/null || true
  fi
  rm -rf -- "${FIXTURE_ROOT}"
}
trap cleanup EXIT

workspace="${FIXTURE_ROOT}/workspace"
fake_bin="${workspace}/bin"
fake_home="${FIXTURE_ROOT}/home"
log_root="${fake_home}/go2_logs"
trace_file="${FIXTURE_ROOT}/go2-log-calls.txt"
lowcmd_calls_file="${FIXTURE_ROOT}/lowcmd-calls.txt"
pgrep_calls_file="${FIXTURE_ROOT}/pgrep-calls.txt"
unitree_lib="${FIXTURE_ROOT}/unitree-lib"
mkdir -p -- \
  "${fake_bin}" \
  "${fake_home}" \
  "${unitree_lib}" \
  "${workspace}/shell" \
  "${workspace}/tools" \
  "${workspace}/install"

cp -- "${REPO_ROOT}/shell/start_slam.sh" "${workspace}/shell/start_slam.sh"
chmod +x "${workspace}/shell/start_slam.sh"
: > "${unitree_lib}/libddsc.so.0"
: > "${unitree_lib}/libddscxx.so.0"

cat > "${workspace}/shell/ros2_environment.sh" <<'SH'
#!/usr/bin/env bash
export PATH="${FAKE_BIN}:/usr/bin:/bin"
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fake-fastdds.xml
export ROS_LOCALHOST_ONLY=0
SH

cat > "${fake_bin}/ip" <<'SH'
#!/usr/bin/env bash
exit 0
SH

cat > "${fake_bin}/pgrep" <<'SH'
#!/usr/bin/env bash
set -Eeuo pipefail
call_count=0
if [[ -s "${FAKE_PGREP_CALLS_FILE}" ]]; then
  call_count="$(< "${FAKE_PGREP_CALLS_FILE}")"
fi
printf '%s\n' "$((call_count + 1))" > "${FAKE_PGREP_CALLS_FILE}"
if [[ "${FAKE_PGREP_START_ERROR_MODE:-none}" == rl &&
      "$*" == *"utree_go2_rl_controller"* ]]; then
  exit 2
fi
if [[ "${FAKE_PGREP_START_ERROR_MODE:-none}" == component &&
      "$*" == *"install/hesai_ros_driver"* ]]; then
  exit 2
fi
if [[ "${FAKE_PGREP_FINAL_ERROR:-false}" == true &&
      "$*" == *"terrain_mapper_node"* ]]; then
  exit 2
fi
if [[ "${FAKE_OTHER_ROBOT_RUNNING:-false}" == true &&
      "$*" == *"terrain_mapper_node"* ]]; then
  echo "4242 terrain_mapper_node"
  exit 0
fi
exit 1
SH

cat > "${fake_bin}/timeout" <<'SH'
#!/usr/bin/env bash
set -Eeuo pipefail
shift
exec "$@"
SH

cat > "${fake_bin}/ros2" <<'SH'
#!/usr/bin/env bash
set -Eeuo pipefail
case "$*" in
  'pkg prefix rmw_fastrtps_cpp')
    exit 0
    ;;
  'topic info --no-daemon --spin-time 3 /lowcmd')
    call_count=0
    if [[ -s "${FAKE_LOWCMD_CALLS_FILE}" ]]; then
      call_count="$(< "${FAKE_LOWCMD_CALLS_FILE}")"
    fi
    call_count=$((call_count + 1))
    printf '%s\n' "${call_count}" > "${FAKE_LOWCMD_CALLS_FILE}"
    if (( call_count == 1 )); then
      case "${FAKE_LOWCMD_START_MODE:-none}" in
        active)
          echo 'Publisher count: 1'
          exit 0
          ;;
        unknown)
          echo "Unknown topic '/lowcmd'"
          exit 1
          ;;
        malformed)
          echo 'Publisher count: unavailable'
          exit 0
          ;;
        error)
          echo 'DDS graph unavailable' >&2
          exit 2
          ;;
        mixed_unknown)
          echo "Unknown topic '/lowcmd'"
          echo 'DDS graph timed out after reporting an unknown topic' >&2
          exit 124
          ;;
        timeout)
          exit 124
          ;;
        missing)
          exit 127
          ;;
      esac
    fi
    if (( call_count > 1 )); then
      case "${FAKE_LOWCMD_AFTER_START:-false}" in
        true|active)
          echo 'Publisher count: 1'
          exit 0
          ;;
        malformed)
          echo 'Publisher count: unavailable'
          exit 0
          ;;
        error)
          echo 'DDS graph unavailable' >&2
          exit 2
          ;;
        mixed_unknown)
          echo "Unknown topic '/lowcmd'"
          echo 'DDS graph timed out after reporting an unknown topic' >&2
          exit 124
          ;;
        timeout)
          exit 124
          ;;
      esac
    fi
    echo 'Publisher count: 0'
    exit 0
    ;;
  'topic echo --once --qos-profile sensor_data /lidar_points sensor_msgs/msg/PointCloud2'|\
  'topic echo --once --qos-profile sensor_data /imu/data sensor_msgs/msg/Imu')
    exit 0
    ;;
  'run hesai_ros_driver hesai_ros_driver_node '*|\
  'run go2_imu_bridge go2_imu_bridge_node '*)
    trap 'exit 0' INT TERM
    while true; do sleep 1; done
    ;;
  'launch super_lio hesai.py '*)
    if [[ "${FAKE_LAUNCH_WAIT:-false}" == true ]]; then
      printf '%s\n' "$$" > "${FAKE_LAUNCH_PID_FILE}"
      printf '%s\n' "${PPID}" > "${FAKE_WRAPPER_PID_FILE}"
      trap 'exit 130' INT
      trap 'exit 143' TERM
      while true; do sleep 1; done
    fi
    exit "${FAKE_LAUNCH_STATUS:-0}"
    ;;
  *)
    echo "unexpected fake ros2 invocation: $*" >&2
    exit 97
    ;;
esac
SH

cat > "${workspace}/tools/go2-log" <<'SH'
#!/usr/bin/env bash
set -Eeuo pipefail
command_name="${1:-}"
session_id="${2:-}"
result_file=""
if [[ "${command_name}" == start && "${2:-}" == --result-file ]]; then
  result_file="${3:-}"
  printf 'start\n' >> "${FAKE_GO2_LOG_TRACE}"
else
  printf '%s\n' "$*" >> "${FAKE_GO2_LOG_TRACE}"
fi
mkdir -p -- "${GO2_LOG_ROOT}/sessions"
case "${command_name}" in
  start)
    if [[ "${FAKE_CONCURRENT_START:-false}" == true &&
          ! -r "${GO2_LOG_ROOT}/active_session" ]]; then
      session_id="concurrent-session"
      mkdir -p -- "${GO2_LOG_ROOT}/sessions/${session_id}"
      printf '%s\n' "${GO2_LOG_ROOT}/sessions/${session_id}" \
        > "${GO2_LOG_ROOT}/active_session"
    fi
    if [[ ! -r "${GO2_LOG_ROOT}/active_session" ]]; then
      session_id="owned-session"
      mkdir -p -- "${GO2_LOG_ROOT}/sessions/${session_id}"
      printf '%s\n' "${GO2_LOG_ROOT}/sessions/${session_id}" \
        > "${GO2_LOG_ROOT}/active_session"
      ownership="created"
    else
      active_session="$(< "${GO2_LOG_ROOT}/active_session")"
      session_id="$(basename -- "${active_session}")"
      ownership="existing"
    fi
    if [[ -n "${result_file}" ]]; then
      result_temp="${result_file}.tmp.$$"
      {
        echo 'format=go2-log-start-v1'
        echo "ownership=${ownership}"
        echo "session_id=${session_id}"
        echo "session_dir=${GO2_LOG_ROOT}/sessions/${session_id}"
      } > "${result_temp}"
      mv -f -- "${result_temp}" "${result_file}"
    fi
    ;;
  stop)
    if [[ -n "${session_id}" ]]; then
      active_session="$(< "${GO2_LOG_ROOT}/active_session")"
      if [[ "$(basename -- "${active_session}")" != "${session_id}" ]]; then
        echo \
          "go2-log: refusing to stop unexpected active diagnostic session" \
          >&2
        exit 1
      fi
    fi
    rm -f -- "${GO2_LOG_ROOT}/active_session"
    ;;
  repair)
    [[ -n "${session_id}" ]]
    ;;
  upload)
    [[ -n "${session_id}" ]]
    if [[ "${FAKE_UPLOAD_STATUS:-0}" != 0 ]]; then
      exit "${FAKE_UPLOAD_STATUS}"
    fi
    : > "${GO2_LOG_ROOT}/sessions/${session_id}/.uploaded"
    ;;
  *)
    exit 98
    ;;
esac
SH

chmod +x \
  "${fake_bin}/ip" \
  "${fake_bin}/pgrep" \
  "${fake_bin}/ros2" \
  "${fake_bin}/timeout" \
  "${workspace}/tools/go2-log"

run_case() {
  local name="$1"
  local preexisting="$2"
  local other_robot_running="$3"
  local launch_status="$4"
  local upload_status="$5"
  local lowcmd_after_start="${6:-false}"
  local pgrep_final_error="${7:-false}"
  local concurrent_start="${8:-false}"
  local lowcmd_start_mode="${9:-none}"
  local pgrep_start_error_mode="${10:-none}"
  local diagnostics_enabled="${11:-true}"
  local diagnostics_auto_finalize="${12:-true}"

  rm -rf -- "${log_root}"
  mkdir -p -- "${log_root}/sessions"
  : > "${trace_file}"
  : > "${lowcmd_calls_file}"
  : > "${pgrep_calls_file}"
  if [[ "${preexisting}" == true ]]; then
    mkdir -p -- "${log_root}/sessions/existing-session"
    printf '%s\n' "${log_root}/sessions/existing-session" \
      > "${log_root}/active_session"
  fi

  set +e
  CASE_OUTPUT="$(
    /usr/bin/timeout 10 env \
    HOME="${fake_home}" \
    FAKE_BIN="${fake_bin}" \
    FAKE_GO2_LOG_TRACE="${trace_file}" \
    FAKE_LOWCMD_CALLS_FILE="${lowcmd_calls_file}" \
    FAKE_LOWCMD_AFTER_START="${lowcmd_after_start}" \
    FAKE_LOWCMD_START_MODE="${lowcmd_start_mode}" \
    FAKE_PGREP_CALLS_FILE="${pgrep_calls_file}" \
    FAKE_PGREP_FINAL_ERROR="${pgrep_final_error}" \
    FAKE_PGREP_START_ERROR_MODE="${pgrep_start_error_mode}" \
    FAKE_CONCURRENT_START="${concurrent_start}" \
    FAKE_OTHER_ROBOT_RUNNING="${other_robot_running}" \
    FAKE_LAUNCH_STATUS="${launch_status}" \
    FAKE_UPLOAD_STATUS="${upload_status}" \
    GO2_LOG_ENABLED="${diagnostics_enabled}" \
    GO2_LOG_AUTO_FINALIZE="${diagnostics_auto_finalize}" \
    GO2_LOG_ROOT="${log_root}" \
    UNITREE_SDK_LIBRARY_DIR="${unitree_lib}" \
    PATH="${fake_bin}:/usr/bin:/bin" \
      "${workspace}/shell/start_slam.sh" 2>&1
  )"
  CASE_STATUS=$?
  set -e
  cp -- "${trace_file}" "${FIXTURE_ROOT}/${name}.trace"
}

mv -- "${workspace}/tools/go2-log" "${workspace}/tools/go2-log.disabled"
run_case diagnostics_disabled false false 0 0 false false false none none false false
test "${CASE_STATUS}" -eq 0
test ! -s "${FIXTURE_ROOT}/diagnostics_disabled.trace"
test ! -e "${log_root}/active_session"
[[ "${CASE_OUTPUT}" == *"GO2 diagnostic logging: false"* ]]
[[ "${CASE_OUTPUT}" == *"Unuploaded diagnostic sessions are ignored by this startup"* ]]
mv -- "${workspace}/tools/go2-log.disabled" "${workspace}/tools/go2-log"

run_case owned false false 0 0
test "${CASE_STATUS}" -eq 0
cat > "${FIXTURE_ROOT}/owned.expected" <<'EOF'
start
stop owned-session
repair owned-session
upload owned-session
EOF
cmp "${FIXTURE_ROOT}/owned.expected" "${FIXTURE_ROOT}/owned.trace"
test -f "${log_root}/sessions/owned-session/.uploaded"

run_case busy false true 0 0
test "${CASE_STATUS}" -eq 0
printf 'start\n' > "${FIXTURE_ROOT}/busy.expected"
cmp "${FIXTURE_ROOT}/busy.expected" "${FIXTURE_ROOT}/busy.trace"
[[ "${CASE_OUTPUT}" == *"Automatic diagnostic finalization skipped"* ]]
test -r "${log_root}/active_session"
test ! -e "${log_root}/sessions/owned-session/.uploaded"

run_case lowcmd false false 0 0 true
test "${CASE_STATUS}" -eq 0
printf 'start\n' > "${FIXTURE_ROOT}/lowcmd.expected"
cmp "${FIXTURE_ROOT}/lowcmd.expected" "${FIXTURE_ROOT}/lowcmd.trace"
[[ "${CASE_OUTPUT}" == *"active /lowcmd publisher"* ]]
test -r "${log_root}/active_session"
test ! -e "${log_root}/sessions/owned-session/.uploaded"

run_case lowcmd_query_failure false false 0 0 error
test "${CASE_STATUS}" -eq 0
printf 'start\n' > "${FIXTURE_ROOT}/lowcmd_query_failure.expected"
cmp "${FIXTURE_ROOT}/lowcmd_query_failure.expected" \
  "${FIXTURE_ROOT}/lowcmd_query_failure.trace"
[[ "${CASE_OUTPUT}" == *"/lowcmd publisher check failed"* ]]
test -r "${log_root}/active_session"
test ! -e "${log_root}/sessions/owned-session/.uploaded"

run_case lowcmd_mixed_unknown false false 0 0 mixed_unknown
test "${CASE_STATUS}" -eq 0
printf 'start\n' > "${FIXTURE_ROOT}/lowcmd_mixed_unknown.expected"
cmp "${FIXTURE_ROOT}/lowcmd_mixed_unknown.expected" \
  "${FIXTURE_ROOT}/lowcmd_mixed_unknown.trace"
[[ "${CASE_OUTPUT}" == *"/lowcmd publisher check failed"* ]]
test -r "${log_root}/active_session"
test ! -e "${log_root}/sessions/owned-session/.uploaded"

run_case pgrep_failure false false 0 0 false true
test "${CASE_STATUS}" -eq 0
printf 'start\n' > "${FIXTURE_ROOT}/pgrep_failure.expected"
cmp "${FIXTURE_ROOT}/pgrep_failure.expected" \
  "${FIXTURE_ROOT}/pgrep_failure.trace"
[[ "${CASE_OUTPUT}" == *"robot process query failed"* ]]
test -r "${log_root}/active_session"
test ! -e "${log_root}/sessions/owned-session/.uploaded"

run_case concurrent_start false false 0 0 false false true
test "${CASE_STATUS}" -eq 0
printf 'start\n' > "${FIXTURE_ROOT}/concurrent_start.expected"
cmp "${FIXTURE_ROOT}/concurrent_start.expected" \
  "${FIXTURE_ROOT}/concurrent_start.trace"
test "$(< "${log_root}/active_session")" = \
  "${log_root}/sessions/concurrent-session"
test ! -e "${log_root}/sessions/concurrent-session/.uploaded"

run_case unknown_start false false 0 0 false false false unknown
test "${CASE_STATUS}" -eq 0
cmp "${FIXTURE_ROOT}/owned.expected" "${FIXTURE_ROOT}/unknown_start.trace"

for lowcmd_failure_mode in active malformed error mixed_unknown timeout missing; do
  run_case \
    "lowcmd_start_${lowcmd_failure_mode}" \
    false false 0 0 false false false "${lowcmd_failure_mode}"
  test "${CASE_STATUS}" -ne 0
  test ! -s "${FIXTURE_ROOT}/lowcmd_start_${lowcmd_failure_mode}.trace"
  [[ "${CASE_OUTPUT}" == *"/lowcmd publisher check failed"* ||
     "${CASE_OUTPUT}" == *"/lowcmd publisher is active"* ]]
done

run_case pgrep_start_rl false false 0 0 false false false none rl
test "${CASE_STATUS}" -ne 0
test ! -s "${FIXTURE_ROOT}/pgrep_start_rl.trace"
[[ "${CASE_OUTPUT}" == *"RL controller process query failed"* ]]

run_case \
  pgrep_start_component false false 0 0 false false false none component
test "${CASE_STATUS}" -ne 0
test ! -s "${FIXTURE_ROOT}/pgrep_start_component.trace"
[[ "${CASE_OUTPUT}" == *"Hesai LiDAR driver process query failed"* ]]

run_case reused true false 0 0
test "${CASE_STATUS}" -eq 0
printf 'start\n' > "${FIXTURE_ROOT}/reused.expected"
cmp "${FIXTURE_ROOT}/reused.expected" "${FIXTURE_ROOT}/reused.trace"
test -r "${log_root}/active_session"

run_case upload_failure false false 23 91
test "${CASE_STATUS}" -eq 23
cat > "${FIXTURE_ROOT}/upload_failure.expected" <<'EOF'
start
stop owned-session
repair owned-session
upload owned-session
EOF
cmp "${FIXTURE_ROOT}/upload_failure.expected" \
  "${FIXTURE_ROOT}/upload_failure.trace"
[[ "${CASE_OUTPUT}" == *"Diagnostic upload failed; session retained"* ]]
test ! -e "${log_root}/sessions/owned-session/.uploaded"

run_interrupt_case() {
  local name="$1"
  local replace_active="$2"
  local interrupt_output="${FIXTURE_ROOT}/${name}.output"
  local launch_pid_file="${FIXTURE_ROOT}/${name}.launch.pid"
  local wrapper_pid_file="${FIXTURE_ROOT}/${name}.wrapper.pid"
  local signal_helper_pid=""
  local signal_helper_status=0

  rm -rf -- "${log_root}"
  mkdir -p -- "${log_root}/sessions"
  rm -f -- "${launch_pid_file}" "${wrapper_pid_file}"
  : > "${trace_file}"
  : > "${lowcmd_calls_file}"
  (
    for _ in {1..100}; do
      if [[ -s "${launch_pid_file}" && -s "${wrapper_pid_file}" ]]; then
        if [[ "${replace_active}" == true ]]; then
          mkdir -p -- "${log_root}/sessions/replacement-session"
          printf '%s\n' "${log_root}/sessions/replacement-session" \
            > "${log_root}/active_session"
        fi
        kill -INT -- \
          "$(< "${wrapper_pid_file}")" \
          "$(< "${launch_pid_file}")"
        exit 0
      fi
      sleep 0.02
    done
    exit 1
  ) &
  signal_helper_pid="$!"

  set +e
  /usr/bin/timeout 5 env \
    HOME="${fake_home}" \
    FAKE_BIN="${fake_bin}" \
    FAKE_GO2_LOG_TRACE="${trace_file}" \
    FAKE_LOWCMD_CALLS_FILE="${lowcmd_calls_file}" \
    FAKE_LOWCMD_AFTER_START=false \
    FAKE_OTHER_ROBOT_RUNNING=false \
    FAKE_LAUNCH_WAIT=true \
    FAKE_LAUNCH_PID_FILE="${launch_pid_file}" \
    FAKE_WRAPPER_PID_FILE="${wrapper_pid_file}" \
    FAKE_UPLOAD_STATUS=0 \
    GO2_LOG_ENABLED=true \
    GO2_LOG_AUTO_FINALIZE=true \
    GO2_LOG_ROOT="${log_root}" \
    UNITREE_SDK_LIBRARY_DIR="${unitree_lib}" \
    PATH="${fake_bin}:/usr/bin:/bin" \
      "${workspace}/shell/start_slam.sh" \
      > "${interrupt_output}" 2>&1
  INTERRUPT_STATUS=$?
  wait "${signal_helper_pid}"
  signal_helper_status=$?
  set -e

  test "${signal_helper_status}" -eq 0
  test "${INTERRUPT_STATUS}" -eq 130
  grep -Fq '[3/3] Starting Super-LIO...' "${interrupt_output}"
  cp -- "${trace_file}" "${FIXTURE_ROOT}/${name}.trace"
  INTERRUPT_OUTPUT="$(< "${interrupt_output}")"
}

run_interrupt_case interrupt false
cmp "${FIXTURE_ROOT}/owned.expected" "${FIXTURE_ROOT}/interrupt.trace"
test -f "${log_root}/sessions/owned-session/.uploaded"

run_interrupt_case replaced true
cat > "${FIXTURE_ROOT}/replaced.expected" <<'EOF'
start
stop owned-session
EOF
cmp "${FIXTURE_ROOT}/replaced.expected" "${FIXTURE_ROOT}/replaced.trace"
[[ "${INTERRUPT_OUTPUT}" == *"Diagnostic stop failed"* ]]
test "$(< "${log_root}/active_session")" = \
  "${log_root}/sessions/replacement-session"
test ! -e "${log_root}/sessions/owned-session/.uploaded"

if pgrep -f -- "${FIXTURE_ROOT}/workspace" >/dev/null 2>&1; then
  echo "SLAM cleanup test leaked a fixture process" >&2
  pgrep -af -- "${FIXTURE_ROOT}/workspace" >&2 || true
  exit 1
fi

echo "PASS: SLAM ignores diagnostics by default and finalizes only explicitly enabled owned sessions"
