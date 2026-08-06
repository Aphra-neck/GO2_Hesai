#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
GO2_LOG="${REPO_ROOT}/tools/go2-log"
FIXTURE_ROOT="$(mktemp -d)"
COLLECTOR_PID=''
STREAM_PID=''

cleanup() {
  if [[ "${STREAM_PID}" =~ ^[1-9][0-9]*$ ]]; then
    kill -TERM "${STREAM_PID}" 2>/dev/null || true
    wait "${STREAM_PID}" 2>/dev/null || true
  fi
  if [[ "${COLLECTOR_PID}" =~ ^[1-9][0-9]*$ ]]; then
    kill -TERM "${COLLECTOR_PID}" 2>/dev/null || true
    wait "${COLLECTOR_PID}" 2>/dev/null || true
  fi
  rm -rf -- "${FIXTURE_ROOT}"
}
trap cleanup EXIT

runtime="${FIXTURE_ROOT}/runtime"
workspace="${FIXTURE_ROOT}/workspace"
session_id="20260806T000000Z-test-host-42"
session_dir="${runtime}/sessions/${session_id}"
mkdir -p -- \
  "${session_dir}" \
  "${workspace}/bin" \
  "${workspace}/tools" \
  "${workspace}/shell"

cat > "${workspace}/shell/ros2_environment.sh" <<'SH'
#!/usr/bin/env bash
export PATH="${GO2_WORKSPACE}/bin:${PATH}"
SH

cat > "${workspace}/bin/ros2" <<'SH'
#!/usr/bin/env bash
set -Eeuo pipefail

if [[ "$*" == "topic list --no-daemon --spin-time 3" ]]; then
  [[ "${FAKE_GRAPH_FAILURE:-0}" == 0 ]] || exit 1
  echo '/lowcmd'
  exit 0
fi

if [[ "$*" == "topic info --no-daemon --spin-time 3 -v /lowcmd" ]]; then
  publishers="${FAKE_LOWCMD_PUBLISHERS:-0}"
  if [[ -n "${FAKE_MOTION_MARKER:-}" && -e "${FAKE_MOTION_MARKER}" ]]; then
    publishers=1
  fi
  printf 'Type: unitree_go/msg/LowCmd\nPublisher count: %s\n' \
    "${publishers}"
  exit 0
fi

if [[ "$1 $2" == "param get" ]]; then
  if [[ "${FAKE_PARAM_FAILURE:-}" == "$3 $4" ]]; then
    exit 1
  fi
  case "$3 $4" in
    '/terrain_mapper min_observed_frames')
      echo 'Integer value is: 6'
      ;;
    '/terrain_mapper max_slope')
      echo 'Double value is: 0.58'
      ;;
    '/terrain_mapper max_roughness')
      echo 'Double value is: 0.07'
      ;;
    '/body_lattice_planner min_traversability')
      echo 'Double value is: 0.21'
      ;;
    '/body_lattice_planner max_slope')
      echo 'Double value is: 0.61'
      ;;
    '/body_lattice_planner max_step_height')
      echo 'Double value is: 0.19'
      ;;
    '/body_lattice_planner snap_radius')
      echo 'Double value is: 0.45'
      ;;
    *)
      exit 1
      ;;
  esac
  exit 0
fi

echo "unexpected fake ros2 invocation: $*" >&2
exit 1
SH
chmod +x "${workspace}/bin/ros2"

cat > "${workspace}/bin/pgrep" <<'SH'
#!/usr/bin/env bash
set -Eeuo pipefail

if [[ "${FAKE_PGREP_FAILURE:-0}" == 1 ]]; then
  exit 2
fi
if [[ "${FAKE_MOTION_PROCESS:-0}" == 1 ]]; then
  echo '4242 /opt/go2/go2_sdk2_bridge_node'
  exit 0
fi
exit 1
SH
chmod +x "${workspace}/bin/pgrep"

cat > "${workspace}/tools/inspect_planner_inputs.py" <<'PY'
#!/usr/bin/env python3
import importlib.util
import json
import os
import sys
from pathlib import Path

if __name__ == "__main__":
    if "--malformed-success" in sys.argv:
        print("not planner inspection JSON")
        raise SystemExit(0)
    if "--capture-error" in sys.argv:
        print("fake planner capture failed", file=sys.stderr)
        raise SystemExit(1)
    if "--stream-probe" in sys.argv:
        import time

        gate = Path(os.environ["FAKE_STREAM_GATE"])
        print("FAKE_PLANNER_READY", file=sys.stderr, flush=True)
        deadline = time.monotonic() + 5.0
        while not gate.exists() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not gate.exists():
            print("stream probe gate timed out", file=sys.stderr)
            raise SystemExit(1)
    diagnostic_failure = "--diagnostic-failure" in sys.argv
    print(json.dumps({
        "diagnosis": (
            "start_has_no_valid_cell_in_snap_square"
            if diagnostic_failure
            else "start_ready_waiting_for_goal"
        ),
        "map": {},
        "start": {},
        "goal": None,
        "arguments": sys.argv[1:],
    }))
    if "--motion-after-capture" in sys.argv:
        open(os.environ["FAKE_MOTION_MARKER"], "w", encoding="utf-8").close()
    if "--switch-session" in sys.argv:
        Path(os.environ["FAKE_ACTIVE_FILE"]).write_text(
            os.environ["FAKE_SWITCH_SESSION"] + "\n",
            encoding="utf-8",
        )
    raise SystemExit(2 if diagnostic_failure else 0)

real_path = os.environ["GO2_REAL_INSPECTOR"]
spec = importlib.util.spec_from_file_location("real_inspector", real_path)
module = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = module
spec.loader.exec_module(module)
_write_session_record_atomic = module._write_session_record_atomic
PY

: > "${session_dir}/planner_input_inspections.jsonl"
printf '%s\n' "${session_dir}" > "${runtime}/active_session"
bash -c "exec -a 'go2-log _collect ${session_dir}' sleep 60" &
COLLECTOR_PID="$!"
printf '%s\n' "${COLLECTOR_PID}" > "${session_dir}/collector.pid"

common_environment=(
  GO2_LOG_ROOT="${runtime}"
  GO2_WORKSPACE="${workspace}"
  GO2_REAL_INSPECTOR="${REPO_ROOT}/tools/inspect_planner_inputs.py"
)

stream_output="${runtime}/planner-check-stream.txt"
stream_gate="${runtime}/planner-check-stream.release"
env "${common_environment[@]}" FAKE_STREAM_GATE="${stream_gate}" \
  "${GO2_LOG}" planner-check --stream-probe > "${stream_output}" 2>&1 &
STREAM_PID="$!"
for _ in $(seq 1 100); do
  grep -Fq 'FAKE_PLANNER_READY' "${stream_output}" && break
  kill -0 "${STREAM_PID}" 2>/dev/null || break
  sleep 0.02
done
grep -Fq 'FAKE_PLANNER_READY' "${stream_output}"
kill -0 "${STREAM_PID}" 2>/dev/null
: > "${stream_gate}"
wait "${STREAM_PID}"
STREAM_PID=''
: > "${session_dir}/planner_input_inspections.jsonl"

before_graph_failure="$(sha256sum "${session_dir}/planner_input_inspections.jsonl")"
set +e
pgrep_output="$(
  env "${common_environment[@]}" FAKE_PGREP_FAILURE=1 \
    "${GO2_LOG}" planner-check --no-goal 2>&1
)"
pgrep_status=$?
set -e
test "${pgrep_status}" -ne 0
[[ "${pgrep_output}" == *"motion process query failed"* ]]
test "${before_graph_failure}" = \
  "$(sha256sum "${session_dir}/planner_input_inspections.jsonl")"

set +e
process_output="$(
  env "${common_environment[@]}" FAKE_MOTION_PROCESS=1 \
    "${GO2_LOG}" planner-check --no-goal 2>&1
)"
process_status=$?
set -e
test "${process_status}" -ne 0
[[ "${process_output}" == *"motion command process is running"* ]]
test "${before_graph_failure}" = \
  "$(sha256sum "${session_dir}/planner_input_inspections.jsonl")"

set +e
graph_output="$(
  env "${common_environment[@]}" FAKE_GRAPH_FAILURE=1 \
    "${GO2_LOG}" planner-check --no-goal 2>&1
)"
graph_status=$?
set -e
test "${graph_status}" -ne 0
[[ "${graph_output}" == *"ROS graph query failed"* ]]
test "${before_graph_failure}" = \
  "$(sha256sum "${session_dir}/planner_input_inspections.jsonl")"

switched_session="${runtime}/sessions/20260806T000100Z-test-host-43"
mkdir -p -- "${switched_session}"
set +e
switch_output="$(
  env "${common_environment[@]}" \
    FAKE_ACTIVE_FILE="${runtime}/active_session" \
    FAKE_SWITCH_SESSION="${switched_session}" \
    "${GO2_LOG}" planner-check --switch-session 2>&1
)"
switch_status=$?
set -e
test "${switch_status}" -ne 0
[[ "${switch_output}" == *"active diagnostic session changed"* ]]
test "${before_graph_failure}" = \
  "$(sha256sum "${session_dir}/planner_input_inspections.jsonl")"
test ! -e "${switched_session}/planner_input_inspections.jsonl"
printf '%s\n' "${session_dir}" > "${runtime}/active_session"

set +e
diagnostic_output="$(
  env "${common_environment[@]}" \
    "${GO2_LOG}" planner-check --diagnostic-failure 2>&1
)"
diagnostic_status=$?
set -e
test "${diagnostic_status}" -eq 2
[[ "${diagnostic_output}" == *"start_has_no_valid_cell_in_snap_square"* ]]

set +e
capture_output="$(
  env "${common_environment[@]}" \
    "${GO2_LOG}" planner-check --capture-error 2>&1
)"
capture_status=$?
set -e
test "${capture_status}" -eq 1
[[ "${capture_output}" == *"capture error recorded"* ]]

python3 - "${session_dir}/planner_input_inspections.jsonl" <<'PY'
import json
from pathlib import Path
import sys

records = [json.loads(line) for line in Path(sys.argv[1]).read_text().splitlines()]
assert len(records) == 2
assert records[0]["exit_code"] == 2
assert records[0]["status"] == "diagnostic_failure"
assert records[0]["inspection"]["diagnosis"] == (
    "start_has_no_valid_cell_in_snap_square"
)
arguments = records[0]["inspection"]["arguments"]
assert arguments[:3] == [
    "--json", "--record-start-on-goal-timeout", "--diagnostic-failure"
]
assert arguments[3:] == [
    "--min-observed-frames", "6",
    "--mapper-max-slope", "0.58",
    "--max-roughness", "0.07",
    "--min-traversability", "0.21",
    "--max-slope", "0.61",
    "--max-step-height", "0.19",
    "--snap-radius", "0.45",
]
assert records[1]["exit_code"] == 1
assert records[1]["status"] == "capture_error"
assert records[1]["error"]["type"] == "inspector_capture_error"
assert "fake planner capture failed" in records[1]["error"]["message"]
PY

set +e
malformed_output="$(
  env "${common_environment[@]}" \
    "${GO2_LOG}" planner-check --malformed-success 2>&1
)"
malformed_status=$?
set -e
test "${malformed_status}" -eq 1
[[ "${malformed_output}" == *"capture error recorded"* ]]

motion_marker="${runtime}/motion-after-capture"
set +e
postflight_output="$(
  env "${common_environment[@]}" FAKE_MOTION_MARKER="${motion_marker}" \
    "${GO2_LOG}" planner-check --motion-after-capture 2>&1
)"
postflight_status=$?
set -e
test "${postflight_status}" -eq 1
[[ "${postflight_output}" == *"post-capture motion safety preflight failed"* ]]
rm -f -- "${motion_marker}"

set +e
parameter_output="$(
  env "${common_environment[@]}" \
    FAKE_PARAM_FAILURE='/terrain_mapper max_roughness' \
    "${GO2_LOG}" planner-check --no-goal 2>&1
)"
parameter_status=$?
set -e
test "${parameter_status}" -eq 1
[[ "${parameter_output}" == \
  *"could not read /terrain_mapper max_roughness"* ]]

python3 - "${session_dir}/planner_input_inspections.jsonl" <<'PY'
import json
from pathlib import Path
import sys

records = [json.loads(line) for line in Path(sys.argv[1]).read_text().splitlines()]
assert len(records) == 5
assert records[-1]["status"] == "capture_error"
assert records[-1]["exit_code"] == 1
assert records[-1]["error"]["type"] == "inspector_capture_error"
assert "could not read /terrain_mapper max_roughness" in (
    records[-1]["error"]["message"]
)
PY

before_motion_check="$(sha256sum "${session_dir}/planner_input_inspections.jsonl")"
set +e
motion_output="$(
  env "${common_environment[@]}" FAKE_LOWCMD_PUBLISHERS=1 \
    "${GO2_LOG}" planner-check --no-goal 2>&1
)"
motion_status=$?
set -e
test "${motion_status}" -ne 0
[[ "${motion_output}" == *"/lowcmd has 1 publisher(s)"* ]]
test "${before_motion_check}" = \
  "$(sha256sum "${session_dir}/planner_input_inspections.jsonl")"

before_stale="$(sha256sum "${session_dir}/planner_input_inspections.jsonl")"
kill -TERM "${COLLECTOR_PID}"
wait "${COLLECTOR_PID}" 2>/dev/null || true
COLLECTOR_PID=''
set +e
stale_output="$(
  env "${common_environment[@]}" "${GO2_LOG}" planner-check --no-goal 2>&1
)"
stale_status=$?
set -e
test "${stale_status}" -ne 0
[[ "${stale_output}" == *"collector is not healthy"* ]]
test "${before_stale}" = "$(sha256sum "${session_dir}/planner_input_inspections.jsonl")"
test -f "${runtime}/active_session"

echo "PASS: planner checks are recorded only in a healthy active session"
