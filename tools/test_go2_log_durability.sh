#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
GO2_LOG="${REPO_ROOT}/tools/go2-log"
FIXTURE_ROOT="$(mktemp -d)"
COLLECTOR_PID=''
ROSOUT_PID=''
HEALTH_PID=''
TIMING_PID=''
ZOMBIE_SUPERVISOR_PID=''
ZOMBIE_WRITER_PGID=''

cleanup() {
  if [[ "${COLLECTOR_PID}" =~ ^[1-9][0-9]*$ ]]; then
    kill -TERM "${COLLECTOR_PID}" 2>/dev/null || true
    wait "${COLLECTOR_PID}" 2>/dev/null || true
  fi
  if [[ "${ROSOUT_PID}" =~ ^[1-9][0-9]*$ ]]; then
    kill -TERM "${ROSOUT_PID}" 2>/dev/null || true
    wait "${ROSOUT_PID}" 2>/dev/null || true
  fi
  if [[ "${HEALTH_PID}" =~ ^[1-9][0-9]*$ ]]; then
    kill -TERM "${HEALTH_PID}" 2>/dev/null || true
    wait "${HEALTH_PID}" 2>/dev/null || true
  fi
  if [[ "${TIMING_PID}" =~ ^[1-9][0-9]*$ ]]; then
    kill -TERM "${TIMING_PID}" 2>/dev/null || true
    wait "${TIMING_PID}" 2>/dev/null || true
  fi
  if [[ "${ZOMBIE_WRITER_PGID}" =~ ^[1-9][0-9]*$ ]]; then
    kill -KILL -- "-${ZOMBIE_WRITER_PGID}" 2>/dev/null || true
  fi
  if [[ "${ZOMBIE_SUPERVISOR_PID}" =~ ^[1-9][0-9]*$ ]]; then
    kill -TERM "${ZOMBIE_SUPERVISOR_PID}" 2>/dev/null || true
    wait "${ZOMBIE_SUPERVISOR_PID}" 2>/dev/null || true
  fi
  rm -rf -- "${FIXTURE_ROOT}"
}
trap cleanup EXIT

initialize_session_files() {
  local directory="$1"
  printf 'timestamp,topic,average_hz,status\n' > "${directory}/topic_rates.csv"
  printf '%s\n' \
    'timestamp,component,running,pids,states,cpu_percent_interval,rss_kib,threads,metrics_status' \
    > "${directory}/process_health.csv"
  printf '%s\n' \
    'window_end,window_duration_sec,topic,status,message_count,first_receive_ns,last_receive_ns,first_header_ns,last_header_ns,latest_header_age_ms,header_span_ms,first_local_sequence,last_local_sequence,unique_header_count,duplicate_header_count,nonmonotonic_header_count,invalid_header_count,min_header_gap_ms,max_header_gap_ms,max_receive_gap_ms' \
    > "${directory}/topic_timing.csv"
  printf '%s\n' \
    'timestamp,monotonic_ns,load1,load5,load15,mem_available_kib,swap_free_kib,thermal_max_millic,thermal_zone,status' \
    > "${directory}/system_health.csv"
  printf '%s\n' \
    'timestamp,monotonic_ns,interface,operstate,carrier,rx_bytes,rx_packets,rx_errors,rx_dropped,tx_bytes,tx_packets,tx_errors,tx_dropped,status' \
    > "${directory}/network_health.csv"
  printf '%s\n' \
    'timestamp,status,file_size_bytes,mtime_epoch,last_frame,frame_delta,last_points,last_packets,last_start_time,last_end_time,tail_warning_lines,tail_error_lines' \
    > "${directory}/hesai_summary.csv"
  printf 'timestamp,topic,ros_stamp_ns,frame_id,child_frame_id,x,y,z,qx,qy,qz,qw,yaw,status\n' \
    > "${directory}/odom_position.csv"
  cp -- "${directory}/odom_position.csv" "${directory}/body_odom_pose.csv"
  printf 'timestamp,vx,vy,yaw_rate,status\n' > "${directory}/sdk2_commands.csv"
  : > "${directory}/events.jsonl"
  : > "${directory}/rosout_warn_error.txt"
}

fake_bin="${FIXTURE_ROOT}/bin"
session_id="20260101T001000Z-test-host-7"
session_dir="${FIXTURE_ROOT}/runtime/sessions/${session_id}"
sync_trace="${FIXTURE_ROOT}/sync-trace.tsv"
collector_pid_file="${FIXTURE_ROOT}/collector.pid"
slam_log_dir="${FIXTURE_ROOT}/slam-logs"
mkdir -p -- "${fake_bin}" "${session_dir}" "${slam_log_dir}"

cat > "${fake_bin}/ros2" <<'SH'
#!/usr/bin/env bash
if [[ "${1:-}" == topic && "${2:-}" == hz ]]; then
  echo "average rate: 10.000"
elif [[ "${1:-}" == topic && "${2:-}" == list ]]; then
  if [[ -n "${GO2_GRAPH_TRACE:-}" ]]; then
    printf 'topic-list\n' >> "${GO2_GRAPH_TRACE}"
  fi
  exit 0
elif [[ "${1:-}" == node && "${2:-}" == list ]]; then
  if [[ -n "${GO2_GRAPH_TRACE:-}" ]]; then
    printf 'node-list\n' >> "${GO2_GRAPH_TRACE}"
  fi
  exit 0
fi
SH

cat > "${fake_bin}/setsid" <<'SH'
#!/usr/bin/env bash
if [[ "${*}" == *" _timing "* ]]; then
  exec /usr/bin/setsid "$@"
fi
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

initialize_session_files "${session_dir}"
printf '%s\n' \
  'raw frame:58 points:64000 packet:500 start time:1785898580.092123 end time:1785898580.191396' \
  > "${slam_log_dir}/hesai.log"
bash -c "exec -a hesai_ros_driver_node sleep 60" &
HEALTH_PID="$!"

PATH="${fake_bin}:/usr/bin:/bin" \
GO2_LOG_ROOT="${FIXTURE_ROOT}/runtime" \
GO2_WORKSPACE="${REPO_ROOT}" \
GO2_SYNC_TRACE="${sync_trace}" \
GO2_COLLECTOR_PID_FILE="${collector_pid_file}" \
GO2_TOPIC_TIMING_COMMAND=/bin/true \
SLAM_LOG_DIR="${slam_log_dir}" \
  "${GO2_LOG}" _collect "${session_dir}" \
  > "${FIXTURE_ROOT}/collector.log" 2>&1 &
COLLECTOR_PID="$!"
printf '%s\n' "${COLLECTOR_PID}" > "${collector_pid_file}"
wait "${COLLECTOR_PID}"
COLLECTOR_PID=''

test "$(wc -l < "${sync_trace}")" -eq 6
test "$(wc -l < "${session_dir}/topic_rates.csv")" -eq 7
test "$(wc -l < "${session_dir}/topic_timing.csv")" -eq 1
test "$(wc -l < "${session_dir}/system_health.csv")" -eq 2
test "$(wc -l < "${session_dir}/hesai_summary.csv")" -eq 2

awk -F, -v expected_pid="${HEALTH_PID}" '
  $2 == "hesai_lidar" {
    found = 1
    if ($3 != 1 || index($4, expected_pid) == 0 || index($5, expected_pid ":") != 1 ||
        $7 !~ /^[0-9]+$/ || $7 <= 0 || $8 !~ /^[0-9]+$/ || $8 <= 0 ||
        $9 != "baseline") {
      exit 1
    }
  }
  END {if (!found) exit 1}
' "${session_dir}/process_health.csv"

awk -F, '
  NR == 2 {
    if (NF != 10 || $2 !~ /^[0-9]+$/ || $3 == "" || $6 !~ /^[0-9]+$/ ||
        ($10 != "ok" && $10 != "partial")) {
      exit 1
    }
    found = 1
  }
  END {if (!found) exit 1}
' "${session_dir}/system_health.csv"

awk -F, '
  NR == 2 {
    if (NF != 12 || $2 != "ok" || $3 !~ /^[0-9]+$/ || $5 != 58 ||
        $7 != 64000 || $8 != 500 || $9 != "1785898580.092123" ||
        $10 != "1785898580.191396") {
      exit 1
    }
    found = 1
  }
  END {if (!found) exit 1}
' "${session_dir}/hesai_summary.csv"

head -n 1 "${session_dir}/network_health.csv" | grep -Fqx \
  'timestamp,monotonic_ns,interface,operstate,carrier,rx_bytes,rx_packets,rx_errors,rx_dropped,tx_bytes,tx_packets,tx_errors,tx_dropped,status'

expected_line_count=2
while IFS=$'\t' read -r option separator path line_count; do
  [[ "${option}" == --data ]]
  [[ "${separator}" == -- ]]
  [[ "${path}" == "${session_dir}/topic_rates.csv" ]]
  [[ "${line_count}" == "${expected_line_count}" ]]
  expected_line_count=$((expected_line_count + 1))
done < "${sync_trace}"
test "${expected_line_count}" -eq 8

resource_dir="${FIXTURE_ROOT}/resource-sampling"
mkdir -p -- "${resource_dir}"
initialize_session_files "${resource_dir}"
GO2_WORKSPACE="${REPO_ROOT}" bash -c '
  set -Eeuo pipefail
  source "$1"
  declare -A previous_states=()
  declare -A previous_cpu_samples=()
  sample_process_health \
    "$2" "2026-01-01T00:10:00Z" previous_states previous_cpu_samples
  sleep 0.2
  sample_process_health \
    "$2" "2026-01-01T00:10:01Z" previous_states previous_cpu_samples
' _ "${GO2_LOG}" "${resource_dir}"

awk -F, -v expected_pid="${HEALTH_PID}" '
  $2 == "hesai_lidar" {
    count++
    if (count == 1 && $9 != "baseline") exit 1
    if (count == 2 &&
        ($3 != 1 || index($4, expected_pid) == 0 || $6 !~ /^[0-9]+([.][0-9]+)?$/ ||
         $7 !~ /^[0-9]+$/ || $7 <= 0 || $8 !~ /^[0-9]+$/ || $8 <= 0 ||
         $9 != "ok")) {
      exit 1
    }
  }
  END {if (count != 2) exit 1}
' "${resource_dir}/process_health.csv"

mixed_metrics_status="$(
  GO2_WORKSPACE="${REPO_ROOT}" bash -c '
    set -Eeuo pipefail
    source "$1"
    classify_process_metrics_status 1 0 1 2 1
  ' _ "${GO2_LOG}"
)"
test "${mixed_metrics_status}" = partial

network_partial_dir="${FIXTURE_ROOT}/network-partial"
mkdir -p -- "${network_partial_dir}"
initialize_session_files "${network_partial_dir}"
GO2_WORKSPACE="${REPO_ROOT}" bash -c '
  set -Eeuo pipefail
  source "$1"
  read_first_line() {
    local path="$1" value=""
    case "${path}" in
      */operstate|*/carrier) return 1 ;;
    esac
    [[ -r "${path}" ]] || return 1
    IFS= read -r value < "${path}" || [[ -n "${value}" ]]
    printf "%s\n" "${value}"
  }
  sample_network_health "$2" "2026-01-01T00:10:02Z"
' _ "${GO2_LOG}" "${network_partial_dir}"
awk -F, '
  NR > 1 {count++; if ($14 != "partial") exit 1}
  END {if (count == 0) exit 1}
' "${network_partial_dir}/network_health.csv"

bounded_tail_bin="${FIXTURE_ROOT}/bounded-tail-bin"
bounded_tail_dir="${FIXTURE_ROOT}/bounded-tail-session"
bounded_tail_trace="${FIXTURE_ROOT}/bounded-tail-trace.txt"
mkdir -p -- "${bounded_tail_bin}" "${bounded_tail_dir}"
initialize_session_files "${bounded_tail_dir}"
cat > "${bounded_tail_bin}/tail" <<'SH'
#!/usr/bin/env bash
if [[ "${1:-}" == -c ]]; then
  printf '%s\n' "$*" >> "${GO2_TAIL_TRACE}"
fi
exec /usr/bin/tail "$@"
SH
chmod +x "${bounded_tail_bin}/tail"
PATH="${bounded_tail_bin}:/usr/bin:/bin" \
GO2_WORKSPACE="${REPO_ROOT}" \
GO2_TAIL_TRACE="${bounded_tail_trace}" \
SLAM_LOG_DIR="${slam_log_dir}" \
  bash -c '
    set -Eeuo pipefail
    source "$1"
    previous_frame=""
    sample_hesai_summary "$2" "2026-01-01T00:10:03Z" previous_frame
  ' _ "${GO2_LOG}" "${bounded_tail_dir}"
grep -Fq -- '-c 262144' "${bounded_tail_trace}"
awk -F, 'NR == 2 {found = 1; if ($2 != "ok" || $5 != 58) exit 1}
  END {if (!found) exit 1}' "${bounded_tail_dir}/hesai_summary.csv"

stop_id="20260101T001050Z-test-host-75"
stop_dir="${FIXTURE_ROOT}/runtime/sessions/${stop_id}"
stop_target_file="${FIXTURE_ROOT}/stop-target.pid"
stop_graph_trace="${FIXTURE_ROOT}/stop-graph-trace.txt"
mkdir -p -- "${stop_dir}"
initialize_session_files "${stop_dir}"
cat > "${fake_bin}/stop-timing" <<'SH'
#!/usr/bin/env bash
for _ in {1..100}; do
  if [[ -s "${GO2_STOP_TARGET_FILE}" ]]; then
    kill -TERM "$(< "${GO2_STOP_TARGET_FILE}")"
    exit 0
  fi
  sleep 0.01
done
exit 1
SH
chmod +x "${fake_bin}/stop-timing"
PATH="${fake_bin}:/usr/bin:/bin" \
GO2_LOG_ROOT="${FIXTURE_ROOT}/runtime" \
GO2_WORKSPACE="${REPO_ROOT}" \
GO2_TOPIC_TIMING_COMMAND="${fake_bin}/stop-timing" \
GO2_STOP_TARGET_FILE="${stop_target_file}" \
GO2_GRAPH_TRACE="${stop_graph_trace}" \
SLAM_LOG_DIR="${slam_log_dir}" \
  "${GO2_LOG}" _collect "${stop_dir}" \
  > "${FIXTURE_ROOT}/stop-collector.log" 2>&1 &
COLLECTOR_PID="$!"
printf '%s\n' "${COLLECTOR_PID}" > "${stop_target_file}"
wait "${COLLECTOR_PID}"
COLLECTOR_PID=''
test ! -s "${stop_graph_trace}"

failure_id="20260101T001100Z-test-host-8"
failure_dir="${FIXTURE_ROOT}/runtime/sessions/${failure_id}"
failure_trace="${FIXTURE_ROOT}/sync-failure-trace.tsv"
failure_log="${FIXTURE_ROOT}/collector-failure.log"
mkdir -p -- "${failure_dir}"
initialize_session_files "${failure_dir}"

set +e
PATH="${fake_bin}:/usr/bin:/bin" \
GO2_LOG_ROOT="${FIXTURE_ROOT}/runtime" \
GO2_WORKSPACE="${REPO_ROOT}" \
GO2_SYNC_TRACE="${failure_trace}" \
GO2_SYNC_FAIL_AT=3 \
GO2_COLLECTOR_PID_FILE="${collector_pid_file}" \
GO2_TOPIC_TIMING_COMMAND=/bin/true \
SLAM_LOG_DIR="${FIXTURE_ROOT}/missing-slam-logs" \
  "${GO2_LOG}" _collect "${failure_dir}" > "${failure_log}" 2>&1
failure_status=$?
set -e
if (( failure_status == 0 )); then
  echo "FAIL: collector ignored a sync --data failure" >&2
  exit 1
fi
test "$(wc -l < "${failure_trace}")" -eq 3
grep -Fq "failed to fdatasync ${failure_dir}/topic_rates.csv" "${failure_log}"
awk -F, 'NR == 2 {found = 1; if ($2 != "missing" || NF != 12) exit 1}
  END {if (!found) exit 1}' "${failure_dir}/hesai_summary.csv"

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

stale_timing_id="20260101T001300Z-test-host-11"
stale_timing_dir="${FIXTURE_ROOT}/runtime/sessions/${stale_timing_id}"
stale_timing_pid_file="${FIXTURE_ROOT}/stale-timing.pid"
mkdir -p -- "${stale_timing_dir}"
initialize_session_files "${stale_timing_dir}"
cat > "${fake_bin}/stale-timing" <<'SH'
#!/usr/bin/env bash
set -Eeuo pipefail
rates_csv=''
while (( $# > 0 )); do
  case "$1" in
    --rates-csv)
      rates_csv="$2"
      shift 2
      ;;
    *)
      shift
      ;;
  esac
done
[[ -n "${rates_csv}" ]]
printf '%s\n' "$$" > "${GO2_STALE_TIMING_PID_FILE}"
while :; do
  printf '2026-01-01T00:13:00Z,/lio/odom,10.0,ok\n' >> "${rates_csv}"
  sleep 0.05
done
SH
chmod +x "${fake_bin}/stale-timing"
printf '%s\n' "${stale_timing_dir}" > "${FIXTURE_ROOT}/runtime/active_session"
PATH="${fake_bin}:/usr/bin:/bin" \
GO2_LOG_ROOT="${FIXTURE_ROOT}/runtime" \
GO2_WORKSPACE="${REPO_ROOT}" \
GO2_TOPIC_TIMING_COMMAND="${fake_bin}/stale-timing" \
GO2_STALE_TIMING_PID_FILE="${stale_timing_pid_file}" \
SLAM_LOG_DIR="${FIXTURE_ROOT}/missing-slam-logs" \
  "${GO2_LOG}" _collect "${stale_timing_dir}" \
  > "${FIXTURE_ROOT}/stale-timing-collector.log" 2>&1 &
COLLECTOR_PID="$!"
printf '%s\n' "${COLLECTOR_PID}" > "${stale_timing_dir}/collector.pid"
for _ in {1..100}; do
  [[ -s "${stale_timing_pid_file}" ]] && break
  sleep 0.02
done
test -s "${stale_timing_pid_file}"
TIMING_PID="$(< "${stale_timing_pid_file}")"
kill -KILL "${COLLECTOR_PID}"
wait "${COLLECTOR_PID}" 2>/dev/null || true
COLLECTOR_PID=''

sessions_before_stale_start="$(
  find "${FIXTURE_ROOT}/runtime/sessions" -mindepth 1 -maxdepth 1 -type d |
    wc -l
)"
active_before_stale_start="$(< "${FIXTURE_ROOT}/runtime/active_session")"
set +e
stale_start_output="$(
  GO2_LOG_ROOT="${FIXTURE_ROOT}/runtime" \
  GO2_WORKSPACE="${REPO_ROOT}" \
    "${GO2_LOG}" start 2>&1
)"
stale_start_status=$?
set -e
if (( stale_start_status == 0 )) ||
  [[ "${stale_start_output}" != *"run './tools/go2-log stop'"* ]]; then
  printf 'FAIL: expected start to reject the stale active session:\n%s\n' \
    "${stale_start_output}" >&2
  exit 1
fi
test "$(< "${FIXTURE_ROOT}/runtime/active_session")" = \
  "${active_before_stale_start}"
test "$(
  find "${FIXTURE_ROOT}/runtime/sessions" -mindepth 1 -maxdepth 1 -type d |
    wc -l
)" -eq "${sessions_before_stale_start}"
kill -0 "${TIMING_PID}"

set +e
stale_timing_output="$(
  GO2_LOG_ROOT="${FIXTURE_ROOT}/runtime" \
  GO2_WORKSPACE="${REPO_ROOT}" \
    "${GO2_LOG}" stop 2>&1
)"
stale_timing_status=$?
set -e
if (( stale_timing_status == 0 )) ||
  [[ "${stale_timing_output}" != *"removed stale active-session state"* ]]; then
  printf 'FAIL: expected stale stop to clean the verified timing writer:\n%s\n' \
    "${stale_timing_output}" >&2
  exit 1
fi
timing_state=''
if [[ -r "/proc/${TIMING_PID}/stat" ]]; then
  timing_stat="$(< "/proc/${TIMING_PID}/stat")"
  timing_state="${timing_stat##*) }"
  timing_state="${timing_state%% *}"
fi
if kill -0 "${TIMING_PID}" 2>/dev/null && [[ "${timing_state}" != Z ]]; then
  echo "FAIL: stale timing writer ${TIMING_PID} survived stale-session cleanup" >&2
  exit 1
fi
TIMING_PID=''
timing_lines_after_stop="$(wc -l < "${stale_timing_dir}/topic_rates.csv")"
sleep 0.2
test "${timing_lines_after_stop}" -eq \
  "$(wc -l < "${stale_timing_dir}/topic_rates.csv")"
test ! -e "${FIXTURE_ROOT}/runtime/active_session"

zombie_timing_id="20260101T001350Z-test-host-115"
zombie_timing_dir="${FIXTURE_ROOT}/runtime/sessions/${zombie_timing_id}"
zombie_timing_pid_file="${FIXTURE_ROOT}/zombie-timing.pids"
zombie_reap_request="${FIXTURE_ROOT}/zombie-timing.reap"
zombie_reaped_marker="${FIXTURE_ROOT}/zombie-timing.reaped"
mkdir -p -- "${zombie_timing_dir}"
initialize_session_files "${zombie_timing_dir}"
python3 - "${zombie_timing_pid_file}" \
  "${zombie_timing_dir}/topic_rates.csv" \
  "${zombie_reap_request}" "${zombie_reaped_marker}" <<'PY' &
import os
from pathlib import Path
import signal
import sys
import time

pid_path = Path(sys.argv[1])
rates_path = Path(sys.argv[2])
reap_request = Path(sys.argv[3])
reaped_marker = Path(sys.argv[4])
leader_pid = os.fork()
if leader_pid == 0:
    os.setsid()
    ready_read, ready_write = os.pipe()
    helper_pid = os.fork()
    if helper_pid == 0:
        os.close(ready_read)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        os.write(ready_write, b"ready")
        os.close(ready_write)
        while True:
            with rates_path.open("a", encoding="ascii") as stream:
                stream.write("2026-01-01T00:13:50Z,/lio/odom,10.0,ok\n")
            time.sleep(0.05)
    os.close(ready_write)
    os.read(ready_read, 5)
    os.close(ready_read)
    pid_path.write_text(
        f"{os.getpid()} {helper_pid} {os.getpgrp()}\n", encoding="ascii"
    )
    os._exit(0)

# Initially do not wait: the leader remains a zombie while its same-PGID helper
# ignores TERM. The test can then request reaping to cover a vanished leader.
while not reap_request.exists():
    time.sleep(0.05)
os.waitpid(leader_pid, 0)
reaped_marker.write_text("reaped\n", encoding="ascii")
while True:
    time.sleep(1)
PY
ZOMBIE_SUPERVISOR_PID="$!"
for _ in {1..100}; do
  [[ -s "${zombie_timing_pid_file}" ]] && break
  sleep 0.02
done
test -s "${zombie_timing_pid_file}"
read -r zombie_leader_pid zombie_helper_pid ZOMBIE_WRITER_PGID \
  < "${zombie_timing_pid_file}"
test "${zombie_leader_pid}" = "${ZOMBIE_WRITER_PGID}"
for _ in {1..100}; do
  zombie_state="$(awk '{print $3}' "/proc/${zombie_leader_pid}/stat" 2>/dev/null || true)"
  [[ "${zombie_state}" == Z ]] && break
  sleep 0.02
done
test "${zombie_state}" = Z
zombie_start_ticks="$(awk '{print $22}' "/proc/${zombie_leader_pid}/stat")"
zombie_boot_id="$(< /proc/sys/kernel/random/boot_id)"
printf '%s\n' "${zombie_timing_dir}" > "${FIXTURE_ROOT}/runtime/active_session"
printf '99999999\n' > "${zombie_timing_dir}/collector.pid"
{
  echo "pid=${zombie_leader_pid}"
  echo "boot_id=${zombie_boot_id}"
  echo "start_ticks=${zombie_start_ticks}"
  echo "pgid=${ZOMBIE_WRITER_PGID}"
  echo "session_dir=${zombie_timing_dir}"
  echo "writer=timing"
  echo "collector_pid=99999999"
  echo "captured_at=2026-01-01T00:13:50Z"
} > "${zombie_timing_dir}/timing.identity"

set +e
zombie_stop_output="$(
  GO2_LOG_ROOT="${FIXTURE_ROOT}/runtime" \
  GO2_WORKSPACE="${REPO_ROOT}" \
    "${GO2_LOG}" stop 2>&1
)"
zombie_stop_status=$?
set -e
zombie_lines_after_stop="$(wc -l < "${zombie_timing_dir}/topic_rates.csv")"
sleep 0.2
zombie_lines_later="$(wc -l < "${zombie_timing_dir}/topic_rates.csv")"
if [[ -e "${FIXTURE_ROOT}/runtime/active_session" ]]; then
  if (( zombie_stop_status == 0 )) ||
    [[ "${zombie_stop_output}" != *"active state was preserved"* ]]; then
    printf 'FAIL: expected zombie-PGID cleanup failure to preserve active state:\n%s\n' \
      "${zombie_stop_output}" >&2
    exit 1
  fi
elif kill -0 "${zombie_helper_pid}" 2>/dev/null &&
  (( zombie_lines_later > zombie_lines_after_stop )); then
    printf 'FAIL: zombie timing leader hid a live same-PGID writer:\n%s\n' \
      "${zombie_stop_output}" >&2
  exit 1
fi

touch "${zombie_reap_request}"
for _ in {1..100}; do
  [[ -s "${zombie_reaped_marker}" && ! -e "/proc/${zombie_leader_pid}" ]] && break
  sleep 0.02
done
test -s "${zombie_reaped_marker}"
test ! -e "/proc/${zombie_leader_pid}"
set +e
reaped_stop_output="$(
  GO2_LOG_ROOT="${FIXTURE_ROOT}/runtime" \
  GO2_WORKSPACE="${REPO_ROOT}" \
    "${GO2_LOG}" stop 2>&1
)"
reaped_stop_status=$?
set -e
if (( reaped_stop_status == 0 )) ||
  [[ "${reaped_stop_output}" != *"active state was preserved"* ]] ||
  [[ ! -e "${FIXTURE_ROOT}/runtime/active_session" ]]; then
  printf 'FAIL: vanished leader with live recorded PGID was not fail-closed:\n%s\n' \
    "${reaped_stop_output}" >&2
  exit 1
fi
kill -0 "${zombie_helper_pid}"

kill -KILL -- "-${ZOMBIE_WRITER_PGID}" 2>/dev/null || true
ZOMBIE_WRITER_PGID=''
kill -TERM "${ZOMBIE_SUPERVISOR_PID}" 2>/dev/null || true
wait "${ZOMBIE_SUPERVISOR_PID}" 2>/dev/null || true
ZOMBIE_SUPERVISOR_PID=''
rm -f -- "${FIXTURE_ROOT}/runtime/active_session"

untracked_timing_id="20260101T001400Z-test-host-12"
untracked_timing_dir="${FIXTURE_ROOT}/runtime/sessions/${untracked_timing_id}"
mkdir -p -- "${untracked_timing_dir}"
initialize_session_files "${untracked_timing_dir}"
bash -c "exec -a 'go2-log _timing ${untracked_timing_dir}' sleep 60" &
TIMING_PID="$!"
printf '%s\n' "${untracked_timing_dir}" > "${FIXTURE_ROOT}/runtime/active_session"
printf '99999999\n' > "${untracked_timing_dir}/collector.pid"

set +e
untracked_stop_output="$(
  GO2_LOG_ROOT="${FIXTURE_ROOT}/runtime" \
  GO2_WORKSPACE="${REPO_ROOT}" \
    "${GO2_LOG}" stop 2>&1
)"
untracked_stop_status=$?
set -e
if (( untracked_stop_status == 0 )) ||
  [[ "${untracked_stop_output}" != *"active state was preserved"* ]]; then
  printf 'FAIL: expected untracked timing writer to preserve active state:\n%s\n' \
    "${untracked_stop_output}" >&2
  exit 1
fi
test -e "${FIXTURE_ROOT}/runtime/active_session"
kill -0 "${TIMING_PID}"

# Upload has its own writer preflight even if active-session state is missing.
rm -f -- "${FIXTURE_ROOT}/runtime/active_session"
sessions_before_untracked_start="$(
  find "${FIXTURE_ROOT}/runtime/sessions" -mindepth 1 -maxdepth 1 -type d |
    wc -l
)"
set +e
untracked_start_output="$(
  GO2_LOG_ROOT="${FIXTURE_ROOT}/runtime" \
  GO2_WORKSPACE="${REPO_ROOT}" \
    "${GO2_LOG}" start 2>&1
)"
untracked_start_status=$?
set -e
if (( untracked_start_status == 0 )) ||
  [[ "${untracked_start_output}" != *"live diagnostic writer exists without active-session state"* ]]; then
  printf 'FAIL: expected start to reject an orphan diagnostic writer:\n%s\n' \
    "${untracked_start_output}" >&2
  exit 1
fi
test "$(
  find "${FIXTURE_ROOT}/runtime/sessions" -mindepth 1 -maxdepth 1 -type d |
    wc -l
)" -eq "${sessions_before_untracked_start}"
set +e
untracked_upload_output="$(
  GO2_LOG_ROOT="${FIXTURE_ROOT}/runtime" \
  GO2_WORKSPACE="${REPO_ROOT}" \
    "${GO2_LOG}" upload "${untracked_timing_id}" 2>&1
)"
untracked_upload_status=$?
set -e
if (( untracked_upload_status == 0 )) ||
  [[ "${untracked_upload_output}" != *"stop all diagnostic writers before upload"* ]]; then
  printf 'FAIL: expected upload to reject an untracked timing writer:\n%s\n' \
    "${untracked_upload_output}" >&2
  exit 1
fi
kill -TERM "${TIMING_PID}"
wait "${TIMING_PID}" 2>/dev/null || true
TIMING_PID=''

echo "PASS: durable rates, bounded health summaries, and stale writer cleanup are enforced"
