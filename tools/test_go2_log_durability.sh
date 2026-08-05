#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
GO2_LOG="${REPO_ROOT}/tools/go2-log"
FIXTURE_ROOT="$(mktemp -d)"
COLLECTOR_PID=''
ROSOUT_PID=''

cleanup() {
  if [[ "${COLLECTOR_PID}" =~ ^[1-9][0-9]*$ ]]; then
    kill -TERM "${COLLECTOR_PID}" 2>/dev/null || true
    wait "${COLLECTOR_PID}" 2>/dev/null || true
  fi
  if [[ "${ROSOUT_PID}" =~ ^[1-9][0-9]*$ ]]; then
    kill -TERM "${ROSOUT_PID}" 2>/dev/null || true
    wait "${ROSOUT_PID}" 2>/dev/null || true
  fi
  rm -rf -- "${FIXTURE_ROOT}"
}
trap cleanup EXIT

fake_bin="${FIXTURE_ROOT}/bin"
session_id="20260101T001000Z-test-host-7"
session_dir="${FIXTURE_ROOT}/runtime/sessions/${session_id}"
sync_trace="${FIXTURE_ROOT}/sync-trace.tsv"
collector_pid_file="${FIXTURE_ROOT}/collector.pid"
mkdir -p -- "${fake_bin}" "${session_dir}"

cat > "${fake_bin}/ros2" <<'SH'
#!/usr/bin/env bash
if [[ "${1:-}" == topic && "${2:-}" == hz ]]; then
  echo "average rate: 10.000"
elif [[ "${1:-}" == topic && "${2:-}" == list ]]; then
  exit 0
elif [[ "${1:-}" == node && "${2:-}" == list ]]; then
  exit 0
fi
SH

cat > "${fake_bin}/setsid" <<'SH'
#!/usr/bin/env bash
exit 0
SH

cat > "${fake_bin}/sync" <<'SH'
#!/usr/bin/env bash
set -Eeuo pipefail
path="${!#}"
line_count="$(wc -l < "${path}")"
printf '%s\t%s\t%s\t%s\n' "${1:-}" "${2:-}" "${path}" "${line_count}" \
  >> "${GO2_SYNC_TRACE}"
call_count="$(wc -l < "${GO2_SYNC_TRACE}")"
if [[ "${GO2_SYNC_FAIL_AT:-}" == "${call_count}" ]]; then
  exit 1
fi
if (( call_count == 6 )); then
  for _ in {1..100}; do
    [[ -s "${GO2_COLLECTOR_PID_FILE}" ]] && break
    sleep 0.01
  done
  kill -TERM "$(< "${GO2_COLLECTOR_PID_FILE}")"
fi
SH
chmod +x "${fake_bin}/ros2" "${fake_bin}/setsid" "${fake_bin}/sync"

printf 'timestamp,topic,average_hz,status\n' > "${session_dir}/topic_rates.csv"
printf 'timestamp,component,running,pids\n' > "${session_dir}/process_health.csv"
printf 'timestamp,topic,ros_stamp_ns,frame_id,child_frame_id,x,y,z,qx,qy,qz,qw,yaw,status\n' \
  > "${session_dir}/odom_position.csv"
cp -- "${session_dir}/odom_position.csv" "${session_dir}/body_odom_pose.csv"
printf 'timestamp,vx,vy,yaw_rate,status\n' > "${session_dir}/sdk2_commands.csv"
: > "${session_dir}/events.jsonl"
: > "${session_dir}/rosout_warn_error.txt"

PATH="${fake_bin}:/usr/bin:/bin" \
GO2_LOG_ROOT="${FIXTURE_ROOT}/runtime" \
GO2_WORKSPACE="${REPO_ROOT}" \
GO2_SYNC_TRACE="${sync_trace}" \
GO2_COLLECTOR_PID_FILE="${collector_pid_file}" \
  "${GO2_LOG}" _collect "${session_dir}" \
  > "${FIXTURE_ROOT}/collector.log" 2>&1 &
COLLECTOR_PID="$!"
printf '%s\n' "${COLLECTOR_PID}" > "${collector_pid_file}"
wait "${COLLECTOR_PID}"
COLLECTOR_PID=''

test "$(wc -l < "${sync_trace}")" -eq 6
test "$(wc -l < "${session_dir}/topic_rates.csv")" -eq 7

expected_line_count=2
while IFS=$'\t' read -r option separator path line_count; do
  [[ "${option}" == --data ]]
  [[ "${separator}" == -- ]]
  [[ "${path}" == "${session_dir}/topic_rates.csv" ]]
  [[ "${line_count}" == "${expected_line_count}" ]]
  expected_line_count=$((expected_line_count + 1))
done < "${sync_trace}"
test "${expected_line_count}" -eq 8

failure_id="20260101T001100Z-test-host-8"
failure_dir="${FIXTURE_ROOT}/runtime/sessions/${failure_id}"
failure_trace="${FIXTURE_ROOT}/sync-failure-trace.tsv"
failure_log="${FIXTURE_ROOT}/collector-failure.log"
mkdir -p -- "${failure_dir}"
printf 'timestamp,topic,average_hz,status\n' > "${failure_dir}/topic_rates.csv"
printf 'timestamp,component,running,pids\n' > "${failure_dir}/process_health.csv"
printf 'timestamp,topic,ros_stamp_ns,frame_id,child_frame_id,x,y,z,qx,qy,qz,qw,yaw,status\n' \
  > "${failure_dir}/odom_position.csv"
cp -- "${failure_dir}/odom_position.csv" "${failure_dir}/body_odom_pose.csv"
printf 'timestamp,vx,vy,yaw_rate,status\n' > "${failure_dir}/sdk2_commands.csv"
: > "${failure_dir}/events.jsonl"
: > "${failure_dir}/rosout_warn_error.txt"

set +e
PATH="${fake_bin}:/usr/bin:/bin" \
GO2_LOG_ROOT="${FIXTURE_ROOT}/runtime" \
GO2_WORKSPACE="${REPO_ROOT}" \
GO2_SYNC_TRACE="${failure_trace}" \
GO2_SYNC_FAIL_AT=3 \
GO2_COLLECTOR_PID_FILE="${collector_pid_file}" \
  "${GO2_LOG}" _collect "${failure_dir}" > "${failure_log}" 2>&1
failure_status=$?
set -e
if (( failure_status == 0 )); then
  echo "FAIL: collector ignored a sync --data failure" >&2
  exit 1
fi
test "$(wc -l < "${failure_trace}")" -eq 3
grep -Fq "failed to fdatasync ${failure_dir}/topic_rates.csv" "${failure_log}"

stale_id="20260101T001200Z-test-host-10"
stale_dir="${FIXTURE_ROOT}/runtime/sessions/${stale_id}"
mkdir -p -- "${stale_dir}"
bash -c "exec -a 'go2-log _rosout ${stale_dir}' sleep 60" &
ROSOUT_PID="$!"
for _ in {1..100}; do
  [[ -r "/proc/${ROSOUT_PID}/stat" ]] && break
  sleep 0.01
done
rosout_start_ticks="$(awk '{print $22}' "/proc/${ROSOUT_PID}/stat")"
rosout_boot_id="$(< /proc/sys/kernel/random/boot_id)"
printf '%s\n' "${stale_dir}" > "${FIXTURE_ROOT}/runtime/active_session"
printf '99999999\n' > "${stale_dir}/collector.pid"
{
  echo "pid=${ROSOUT_PID}"
  echo "boot_id=${rosout_boot_id}"
  echo "start_ticks=${rosout_start_ticks}"
  echo "pgid=unknown"
  echo "session_dir=${stale_dir}"
  echo "collector_pid=99999999"
  echo "captured_at=2026-01-01T00:12:00Z"
} > "${stale_dir}/rosout.identity"

set +e
stale_output="$(
  GO2_LOG_ROOT="${FIXTURE_ROOT}/runtime" \
  GO2_WORKSPACE="${REPO_ROOT}" \
    "${GO2_LOG}" stop 2>&1
)"
stale_status=$?
set -e
if (( stale_status == 0 )) ||
  [[ "${stale_output}" != *"removed stale active-session state"* ]]; then
  printf 'FAIL: expected stale stop to terminate the verified rosout writer:\n%s\n' \
    "${stale_output}" >&2
  exit 1
fi
wait "${ROSOUT_PID}" 2>/dev/null || true
ROSOUT_PID=''
test ! -e "${FIXTURE_ROOT}/runtime/active_session"

echo "PASS: durable topic rates and stale rosout cleanup are enforced"
