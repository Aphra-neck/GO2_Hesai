#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
FIXTURE_ROOT="$(mktemp -d)"
collector_pid=''

cleanup() {
  if [[ "${collector_pid}" =~ ^[1-9][0-9]*$ ]]; then
    kill "${collector_pid}" 2>/dev/null || true
    wait "${collector_pid}" 2>/dev/null || true
  fi
  rm -rf -- "${FIXTURE_ROOT}"
}
trap cleanup EXIT

workspace="${FIXTURE_ROOT}/workspace"
fake_bin="${FIXTURE_ROOT}/bin"
log_root="${FIXTURE_ROOT}/go2_logs"
unitree_lib="${FIXTURE_ROOT}/unitree-lib"
build_parent="${FIXTURE_ROOT}/build-parent"
build_record="${FIXTURE_ROOT}/build-path.txt"
python_record="${FIXTURE_ROOT}/python-called.txt"
python_args_record="${FIXTURE_ROOT}/python-args.txt"
session_dir="${log_root}/sessions/session-real"
session_link="${log_root}/sessions/session-link"

mkdir -p -- \
  "${workspace}/tools" \
  "${workspace}/shell" \
  "${fake_bin}" \
  "${build_parent}" \
  "${unitree_lib}" \
  "${session_dir}"
cp -- "${SCRIPT_DIR}/run_sdk2_motion_stall_probe.sh" "${workspace}/tools/"

cat > "${workspace}/shell/ros2_environment.sh" <<'SH'
#!/usr/bin/env bash
export PATH="${FAKE_BIN}:${PATH}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
SH

cat > "${fake_bin}/ip" <<'SH'
#!/usr/bin/env bash
exit 0
SH
chmod +x "${fake_bin}/ip"
touch "${unitree_lib}/libddsc.so.0" "${unitree_lib}/libddscxx.so.0"

cat > "${fake_bin}/go2-log" <<'SH'
#!/usr/bin/env bash
while :; do
  sleep 1
done
SH
chmod +x "${fake_bin}/go2-log"

cat > "${fake_bin}/cmake" <<'SH'
#!/usr/bin/env bash
if [[ "$1" == "-S" ]]; then
  while (( $# > 0 )); do
    if [[ "$1" == "-B" ]]; then
      printf '%s\n' "$2" > "${BUILD_RECORD:?}"
      mkdir -p -- "$2"
      exit 0
    fi
    shift
  done
elif [[ "$1" == "--build" ]]; then
  reader="$2/go2_sdk2_motion_stall_reader"
  printf '%s\n' '#!/usr/bin/env bash' 'exit 0' > "${reader}"
  chmod +x "${reader}"
  if [[ "${INVALIDATE_DURING_BUILD:-false}" == true ]]; then
    touch "${SESSION_DIR:?}/ended_at.txt"
  fi
  exit 0
fi
exit 1
SH
chmod +x "${fake_bin}/cmake"

cat > "${fake_bin}/python3" <<'SH'
#!/usr/bin/env bash
if [[ -n "${PYTHON_RECORD:-}" ]]; then
  printf 'called\n' > "${PYTHON_RECORD}"
fi
if [[ -n "${PYTHON_ARGS_RECORD:-}" ]]; then
  printf '%s\n' "$@" > "${PYTHON_ARGS_RECORD}"
fi
output_dir=''
active_session_file=''
expected_session=''
collector_pid=''
while (( $# > 0 )); do
  case "$1" in
    --output-dir)
      output_dir="$2"
      shift 2
      ;;
    --active-session-file)
      active_session_file="$2"
      shift 2
      ;;
    --expected-session)
      expected_session="$2"
      shift 2
      ;;
    --collector-pid)
      collector_pid="$2"
      shift 2
      ;;
    *)
      shift
      ;;
  esac
done
[[ -n "${output_dir}" && -d "${output_dir}" && ! -L "${output_dir}" ]]
[[ "${active_session_file}" == "${GO2_LOG_ROOT:?}/active_session" ]]
[[ "${expected_session}" == "${EXPECTED_SESSION:?}" ]]
[[ "${collector_pid}" == "${EXPECTED_COLLECTOR_PID:?}" ]]
printf 'reserved_capture=%s\n' "${output_dir}"
SH
chmod +x "${fake_bin}/python3"

set +e
reserved_output="$(
  env \
    FAKE_BIN="${fake_bin}" \
    GO2_LOG_ROOT="${log_root}" \
    UNITREE_SDK_LIBRARY_DIR="${unitree_lib}" \
    "${workspace}/tools/run_sdk2_motion_stall_probe.sh" \
      --read /tmp/not-the-built-reader 2>&1
)"
reserved_status=$?
set -e

test "${reserved_status}" -ne 0
[[ "${reserved_output}" == *"Unsupported stall-probe argument"* ]]

ln -s -- "${session_dir}" "${session_link}"
printf '%s\n' "${session_link}" > "${log_root}/active_session"

if [[ -L "${session_link}" ]]; then
  set +e
  output="$(
    env \
      FAKE_BIN="${fake_bin}" \
      GO2_LOG_ROOT="${log_root}" \
      UNITREE_SDK_LIBRARY_DIR="${unitree_lib}" \
      "${workspace}/tools/run_sdk2_motion_stall_probe.sh" --duration 5 2>&1
  )"
  status=$?
  set -e

  test "${status}" -ne 0
  [[ "${output}" == *"symbolic-link session directory"* ]]
fi

rm -- "${log_root}/active_session"
printf '%s\n' "${session_dir}" > "${log_root}/active_session"
"${fake_bin}/go2-log" _collect "${session_dir}" &
collector_pid=$!
printf '%s\n' "${collector_pid}" > "${session_dir}/collector.pid"

set +e
invalidated_output="$(
  env \
    FAKE_BIN="${fake_bin}" \
    GO2_LOG_ROOT="${log_root}" \
    GO2_STALL_PROBE_BUILD_PARENT="${build_parent}" \
    BUILD_RECORD="${build_record}" \
    INVALIDATE_DURING_BUILD=true \
    SESSION_DIR="${session_dir}" \
    PYTHON_RECORD="${python_record}" \
    UNITREE_SDK_LIBRARY_DIR="${unitree_lib}" \
    "${workspace}/tools/run_sdk2_motion_stall_probe.sh" --duration 5 2>&1
)"
invalidated_status=$?
set -e
test "${invalidated_status}" -ne 0
[[ "${invalidated_output}" == *"changed during the probe build"* ]]
[[ ! -e "${python_record}" ]]
test -z "$(find "${session_dir}" -mindepth 1 -maxdepth 1 \
  -type d -name 'sdk2_motion_stall_*' -print -quit)"
rm -- "${session_dir}/ended_at.txt"

success_output="$(
  env \
    FAKE_BIN="${fake_bin}" \
    GO2_LOG_ROOT="${log_root}" \
    GO2_STALL_PROBE_BUILD_DIR="${FIXTURE_ROOT}/unsafe-fixed-build" \
    GO2_STALL_PROBE_BUILD_PARENT="${build_parent}" \
    BUILD_RECORD="${build_record}" \
    PYTHON_RECORD="${python_record}" \
    PYTHON_ARGS_RECORD="${python_args_record}" \
    EXPECTED_SESSION="${session_dir}" \
    EXPECTED_COLLECTOR_PID="${collector_pid}" \
    UNITREE_SDK_LIBRARY_DIR="${unitree_lib}" \
    "${workspace}/tools/run_sdk2_motion_stall_probe.sh" --duration 5
)"
[[ "${success_output}" == *"reserved_capture=${session_dir}/sdk2_motion_stall_"* ]]
grep -Fxq -- '--active-session-file' "${python_args_record}"
grep -Fxq -- "${log_root}/active_session" "${python_args_record}"
grep -Fxq -- '--expected-session' "${python_args_record}"
grep -Fxq -- "${session_dir}" "${python_args_record}"
grep -Fxq -- '--collector-pid' "${python_args_record}"
grep -Fxq -- "${collector_pid}" "${python_args_record}"
reserved_build="$(< "${build_record}")"
[[ "${reserved_build}" == "${build_parent}/go2-sdk2-motion-stall-build."* ]]
[[ ! -e "${reserved_build}" ]]

truncate -s $((70 * 1024 * 1024)) "${session_dir}/capacity-fixture.bin"
set +e
capacity_output="$(
  env \
    FAKE_BIN="${fake_bin}" \
    GO2_LOG_ROOT="${log_root}" \
    GO2_STALL_PROBE_BUILD_PARENT="${build_parent}" \
    UNITREE_SDK_LIBRARY_DIR="${unitree_lib}" \
    "${workspace}/tools/run_sdk2_motion_stall_probe.sh" --duration 5 2>&1
)"
capacity_status=$?
set -e
test "${capacity_status}" -ne 0
[[ "${capacity_output}" == *"lacks reserved capacity"* ]]

echo "PASS: motion-stall probe enforces read-only arguments, session ownership, and capacity"
