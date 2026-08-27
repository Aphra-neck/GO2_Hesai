#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
GO2_LOG="${REPO_ROOT}/tools/go2-log"
FIXTURE_ROOT="$(mktemp -d)"
trap 'rm -rf -- "${FIXTURE_ROOT}"' EXIT

LOG_ROOT="${FIXTURE_ROOT}/runtime"
LOG_REPO="${FIXTURE_ROOT}/log-repo"
REMOTE_REPO="${FIXTURE_ROOT}/remote.git"
SEED_REPO="${FIXTURE_ROOT}/seed"
REMOTE_URL="file://${REMOTE_REPO}"
FAKE_BIN="${FIXTURE_ROOT}/bin"

mkdir -p -- "${FAKE_BIN}"
cat > "${FAKE_BIN}/ros2" <<'SH'
#!/usr/bin/env bash
if [[ "${1:-}" == topic && "${2:-}" == info && "${!#}" == /lowcmd ]]; then
  printf 'Type: unitree_go/msg/LowCmd\nPublisher count: 0\nSubscription count: 0\n'
  exit 0
fi
exit 1
SH
chmod +x "${FAKE_BIN}/ros2"

run_go2_log() {
  PATH="${FAKE_BIN}:${PATH}" \
  GO2_LOG_ROOT="${LOG_ROOT}" \
  GO2_LOG_REPO="${LOG_REPO}" \
  GO2_LOG_REMOTE="${REMOTE_URL}" \
  GO2_LOG_BRANCH=main \
  GO2_LOG_PROXY= \
    "${GO2_LOG}" "$@"
}

git init --bare -q --initial-branch=main "${REMOTE_REPO}"
git init -q --initial-branch=main "${SEED_REPO}"
git -C "${SEED_REPO}" config user.name "GO2 Log Test"
git -C "${SEED_REPO}" config user.email "go2-log-test@example.invalid"
printf '# Fixture log repository\n' > "${SEED_REPO}/README.md"
git -C "${SEED_REPO}" add README.md
git -C "${SEED_REPO}" commit -q -m "Initialize fixture"
git -C "${SEED_REPO}" remote add origin "${REMOTE_URL}"
git -C "${SEED_REPO}" push -q origin main

session_id="20260101T000000Z-test-host-1"
session_dir="${LOG_ROOT}/sessions/${session_id}"
mkdir -p -- "${session_dir}"
expected="${FIXTURE_ROOT}/expected-topic-rates.csv"
printf 'timestamp,topic,average_hz,status\n2026-01-01T00:00:00Z,/imu/data,160.0,ok\n' \
  > "${expected}"
cp -- "${expected}" "${session_dir}/topic_rates.csv"
printf '\0\0\0' >> "${session_dir}/topic_rates.csv"
original_sha="$(sha256sum "${session_dir}/topic_rates.csv" | awk '{print $1}')"

set +e
upload_output="$(run_go2_log upload "${session_id}" 2>&1)"
upload_status=$?
set -e
if (( upload_status == 0 )) ||
  [[ "${upload_output}" != *"NUL-containing binary file: topic_rates.csv"* ]]; then
  printf 'FAIL: expected the original upload to reject trailing NUL bytes:\n%s\n' \
    "${upload_output}" >&2
  exit 1
fi

run_go2_log repair "${session_id}"
cmp -- "${expected}" "${session_dir}/topic_rates.csv"
test -s "${session_dir}/repair_manifest.jsonl"
grep -Fq '"trailing_nul_bytes":3' "${session_dir}/repair_manifest.jsonl"
grep -Fq "\"original_sha256\":\"${original_sha}\"" \
  "${session_dir}/repair_manifest.jsonl"

mapfile -t quarantined < <(
  find "${LOG_ROOT}/quarantine/${session_id}" -maxdepth 1 -type f -name 'topic_rates.csv.*.corrupt'
)
if (( ${#quarantined[@]} != 1 )); then
  printf 'FAIL: expected one quarantined original, found %s\n' "${#quarantined[@]}" >&2
  exit 1
fi
test "$(sha256sum "${quarantined[0]}" | awk '{print $1}')" = "${original_sha}"
grep -Fq '"state":"completed"' \
  "${LOG_ROOT}/quarantine/${session_id}/topic_rates.csv.${original_sha}.repair.json"

second_repair_output="$(run_go2_log repair "${session_id}")"
[[ "${second_repair_output}" == *"No repairable trailing NUL suffix found"* ]]
test "$(wc -l < "${session_dir}/repair_manifest.jsonl")" -eq 1

printf '\0\0\0' >> "${session_dir}/topic_rates.csv"
run_go2_log repair "${session_id}"
cmp -- "${expected}" "${session_dir}/topic_rates.csv"
test "$(wc -l < "${session_dir}/repair_manifest.jsonl")" -eq 1

head -c $((95 * 1024 * 1024)) < /dev/zero | tr '\0' x \
  > "${session_dir}/network.txt"
printf '\n' >> "${session_dir}/network.txt"
run_go2_log upload "${session_id}"
test -s "${session_dir}/.uploaded"
cmp -- "${expected}" "${session_dir}/topic_rates.csv"
grep -Fq 'capture truncated to preserve the session size limit' \
  "${session_dir}/network.txt"
git --git-dir="${REMOTE_REPO}" cat-file -e \
  "refs/heads/main:sessions/${session_id}/topic_rates.csv"
git --git-dir="${REMOTE_REPO}" cat-file -e \
  "refs/heads/main:sessions/${session_id}/repair_manifest.jsonl"

invalid_id="20260101T000100Z-test-host-2"
invalid_dir="${LOG_ROOT}/sessions/${invalid_id}"
mkdir -p -- "${invalid_dir}"
printf 'timestamp,topic,average_hz,status\nvalid\0corrupt\n' \
  > "${invalid_dir}/topic_rates.csv"
invalid_sha="$(sha256sum "${invalid_dir}/topic_rates.csv" | awk '{print $1}')"

set +e
repair_output="$(run_go2_log repair "${invalid_id}" 2>&1)"
repair_status=$?
set -e
if (( repair_status == 0 )) ||
  [[ "${repair_output}" != *"NUL bytes are not a trailing suffix"* ]]; then
  printf 'FAIL: expected repair to reject a middle NUL byte:\n%s\n' \
    "${repair_output}" >&2
  exit 1
fi
test "$(sha256sum "${invalid_dir}/topic_rates.csv" | awk '{print $1}')" = "${invalid_sha}"

orphan_manifest_id="20260101T000150Z-test-host-9"
orphan_manifest_dir="${LOG_ROOT}/sessions/${orphan_manifest_id}"
mkdir -p -- "${orphan_manifest_dir}"
cp -- "${expected}" "${orphan_manifest_dir}/topic_rates.csv"
python3 - \
  "${orphan_manifest_dir}/repair_manifest.jsonl" \
  "${orphan_manifest_id}" \
  "${original_sha}" \
  "$(sha256sum "${expected}" | awk '{print $1}')" \
  "$(($(stat -c '%s' "${expected}") + 3))" \
  "$(stat -c '%s' "${expected}")" <<'PY'
import json
from pathlib import Path
import sys

path, session_id, original_sha, repaired_sha, original_size, repaired_size = sys.argv[1:]
record = {
    "file": "topic_rates.csv",
    "original_sha256": original_sha,
    "original_size": int(original_size),
    "quarantine_file": (
        f"quarantine/{session_id}/topic_rates.csv.{original_sha}.corrupt"
    ),
    "repaired_at": "2026-01-01T00:01:50Z",
    "repaired_sha256": repaired_sha,
    "repaired_size": int(repaired_size),
    "trailing_nul_bytes": int(original_size) - int(repaired_size),
}
Path(path).write_text(
    json.dumps(record, sort_keys=True, separators=(",", ":")) + "\n",
    encoding="utf-8",
)
PY
set +e
orphan_manifest_output="$(run_go2_log upload "${orphan_manifest_id}" 2>&1)"
orphan_manifest_status=$?
set -e
if (( orphan_manifest_status == 0 )) ||
  [[ "${orphan_manifest_output}" != *"repair evidence is incomplete or inconsistent"* ]]; then
  printf 'FAIL: expected upload to reject a manifest without quarantine evidence:\n%s\n' \
    "${orphan_manifest_output}" >&2
  exit 1
fi

symlink_session_id="20260101T000200Z-test-host-3"
symlink_target="${FIXTURE_ROOT}/outside-session"
mkdir -p -- "${symlink_target}"
printf 'safe\n\0' > "${symlink_target}/topic_rates.csv"
ln -s -- "${symlink_target}" "${LOG_ROOT}/sessions/${symlink_session_id}"
set +e
symlink_session_output="$(run_go2_log repair "${symlink_session_id}" 2>&1)"
symlink_session_status=$?
set -e
if (( symlink_session_status == 0 )) ||
  [[ "${symlink_session_output}" != *"symbolic-link session directory"* ]]; then
  printf 'FAIL: expected a symbolic-link session to be rejected:\n%s\n' \
    "${symlink_session_output}" >&2
  exit 1
fi
test "$(sha256sum "${symlink_target}/topic_rates.csv" | awk '{print $1}')" = \
  "$(printf 'safe\n\0' | sha256sum | awk '{print $1}')"

symlink_file_id="20260101T000300Z-test-host-4"
symlink_file_dir="${LOG_ROOT}/sessions/${symlink_file_id}"
symlink_file_target="${FIXTURE_ROOT}/outside-topic-rates.csv"
mkdir -p -- "${symlink_file_dir}"
printf 'safe\n\0' > "${symlink_file_target}"
symlink_file_sha="$(sha256sum "${symlink_file_target}" | awk '{print $1}')"
ln -s -- "${symlink_file_target}" "${symlink_file_dir}/topic_rates.csv"
set +e
symlink_file_output="$(run_go2_log repair "${symlink_file_id}" 2>&1)"
symlink_file_status=$?
set -e
if (( symlink_file_status == 0 )) ||
  [[ "${symlink_file_output}" != *"cannot open repairable session file"* ]]; then
  printf 'FAIL: expected a symbolic-link session file to be rejected:\n%s\n' \
    "${symlink_file_output}" >&2
  exit 1
fi
test "$(sha256sum "${symlink_file_target}" | awk '{print $1}')" = "${symlink_file_sha}"

bad_manifest_id="20260101T000400Z-test-host-5"
bad_manifest_dir="${LOG_ROOT}/sessions/${bad_manifest_id}"
mkdir -p -- "${bad_manifest_dir}"
printf 'safe\n\0' > "${bad_manifest_dir}/topic_rates.csv"
printf 'not-json\n' > "${bad_manifest_dir}/repair_manifest.jsonl"
bad_manifest_sha="$(sha256sum "${bad_manifest_dir}/topic_rates.csv" | awk '{print $1}')"
set +e
bad_manifest_output="$(run_go2_log repair "${bad_manifest_id}" 2>&1)"
bad_manifest_status=$?
set -e
if (( bad_manifest_status == 0 )) ||
  [[ "${bad_manifest_output}" != *"invalid repair manifest JSON"* ]]; then
  printf 'FAIL: expected invalid existing manifest JSON to be rejected:\n%s\n' \
    "${bad_manifest_output}" >&2
  exit 1
fi
test "$(sha256sum "${bad_manifest_dir}/topic_rates.csv" | awk '{print $1}')" = \
  "${bad_manifest_sha}"

recovery_id="20260101T000500Z-test-host-6"
recovery_dir="${LOG_ROOT}/sessions/${recovery_id}"
recovery_quarantine="${LOG_ROOT}/quarantine/${recovery_id}"
recovery_clean="${FIXTURE_ROOT}/recovery-clean.csv"
recovery_original="${FIXTURE_ROOT}/recovery-original.csv"
mkdir -p -- "${recovery_dir}" "${recovery_quarantine}"
printf 'timestamp,topic,average_hz,status\n2026-01-01T00:00:00Z,/imu/data,,no_data\n' \
  > "${recovery_clean}"
cp -- "${recovery_clean}" "${recovery_original}"
printf '\0\0\0\0' >> "${recovery_original}"
cp -- "${recovery_clean}" "${recovery_dir}/topic_rates.csv"
recovery_original_sha="$(sha256sum "${recovery_original}" | awk '{print $1}')"
recovery_repaired_sha="$(sha256sum "${recovery_clean}" | awk '{print $1}')"
recovery_evidence="topic_rates.csv.${recovery_original_sha}.corrupt"
recovery_journal="topic_rates.csv.${recovery_original_sha}.repair.json"
cp -- "${recovery_original}" "${recovery_quarantine}/${recovery_evidence}"
python3 - \
  "${recovery_quarantine}/${recovery_journal}" \
  "${recovery_id}" \
  "${recovery_original_sha}" \
  "${recovery_repaired_sha}" \
  "$(stat -c '%s' "${recovery_original}")" \
  "$(stat -c '%s' "${recovery_clean}")" <<'PY'
import json
from pathlib import Path
import sys

path, session_id, original_sha, repaired_sha, original_size, repaired_size = sys.argv[1:]
record = {
    "file": "topic_rates.csv",
    "original_sha256": original_sha,
    "original_size": int(original_size),
    "quarantine_file": (
        f"quarantine/{session_id}/topic_rates.csv.{original_sha}.corrupt"
    ),
    "repaired_at": "2026-01-01T00:05:00Z",
    "repaired_sha256": repaired_sha,
    "repaired_size": int(repaired_size),
    "state": "prepared",
    "trailing_nul_bytes": int(original_size) - int(repaired_size),
}
Path(path).write_text(
    json.dumps(record, sort_keys=True, separators=(",", ":")) + "\n",
    encoding="utf-8",
)
PY

run_go2_log repair "${recovery_id}"
cmp -- "${recovery_clean}" "${recovery_dir}/topic_rates.csv"
grep -Fq "\"original_sha256\":\"${recovery_original_sha}\"" \
  "${recovery_dir}/repair_manifest.jsonl"
grep -Fq '"state":"completed"' \
  "${recovery_quarantine}/${recovery_journal}"

echo "PASS: trailing-NUL repair is durable, resumable, and rejects unsafe inputs"
