#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
FIXTURE_ROOT="$(mktemp -d)"

cleanup() {
  rm -rf -- "${FIXTURE_ROOT}"
}
trap cleanup EXIT

workspace="${FIXTURE_ROOT}/workspace"
fake_bin="${workspace}/bin"
unitree_lib="${FIXTURE_ROOT}/unitree-lib"
trace_file="${FIXTURE_ROOT}/ros2-calls.txt"
mkdir -p -- \
  "${fake_bin}" \
  "${unitree_lib}" \
  "${workspace}/shell" \
  "${workspace}/src/utree_go2_sdk2_bridge/config"

cp -- "${REPO_ROOT}/shell/start_sdk2_bridge.sh" \
  "${workspace}/shell/start_sdk2_bridge.sh"
chmod +x "${workspace}/shell/start_sdk2_bridge.sh"

cat > "${workspace}/shell/ros2_environment.sh" <<'SH'
#!/usr/bin/env bash
export PATH="${FAKE_BIN}:${PATH}"
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fake-fastdds.xml
export ROS_LOCALHOST_ONLY=0
SH

cat > "${workspace}/src/utree_go2_sdk2_bridge/config/go2_sdk2_bridge.yaml" <<'YAML'
go2_sdk2_bridge:
  ros__parameters:
    command_rate: 200.0
    translation_speed: 0.40
    rotation_speed: 0.60
    max_vx: 0.6
    max_vy: 0.35
    max_yaw_rate: 0.8
YAML

cat > "${fake_bin}/ros2" <<'SH'
#!/usr/bin/env bash
set -Eeuo pipefail
printf '%s\n' "$*" >> "${FAKE_ROS2_TRACE}"
case "$*" in
  'pkg prefix utree_go2_sdk2_bridge')
    printf '%s\n' "${FAKE_WORKSPACE}/install/utree_go2_sdk2_bridge"
    ;;
  'topic info --no-daemon --spin-time 3 /lowcmd')
    printf 'Publisher count: 0\n'
    ;;
  'topic echo --once --qos-profile sensor_data /lio/body_odom nav_msgs/msg/Odometry')
    ;;
  'launch utree_go2_sdk2_bridge go2_sdk2_bridge.launch.py '*)
    ;;
  *)
    echo "unexpected fake ros2 invocation: $*" >&2
    exit 97
    ;;
esac
SH

cat > "${fake_bin}/timeout" <<'SH'
#!/usr/bin/env bash
set -Eeuo pipefail
shift
exec "$@"
SH

cat > "${fake_bin}/ip" <<'SH'
#!/usr/bin/env bash
exit 0
SH

cat > "${fake_bin}/pgrep" <<'SH'
#!/usr/bin/env bash
exit 1
SH

chmod +x \
  "${fake_bin}/ros2" \
  "${fake_bin}/timeout" \
  "${fake_bin}/ip" \
  "${fake_bin}/pgrep"
touch "${unitree_lib}/libddsc.so.0" "${unitree_lib}/libddscxx.so.0"

common_environment=(
  FAKE_BIN="${fake_bin}"
  FAKE_ROS2_TRACE="${trace_file}"
  FAKE_WORKSPACE="${workspace}"
  PATH="${fake_bin}:/usr/bin:/bin"
  UNITREE_SDK_LIBRARY_DIR="${unitree_lib}"
)

: > "${trace_file}"
default_output="$(
  env -u GO2_MAX_VX -u GO2_MAX_VY -u GO2_MAX_YAW_RATE \
    "${common_environment[@]}" "${workspace}/shell/start_sdk2_bridge.sh"
)"
grep -Fq ' vx=0.6 m/s [YAML:' <<< "${default_output}"
grep -Fq ' vy=0.35 m/s [YAML:' <<< "${default_output}"
grep -Fq ' yaw=0.8 rad/s [YAML:' <<< "${default_output}"
grep -Fq ' Control: direct SportClient::Move refresh at 200.0 Hz' <<< "${default_output}"
grep -Fq ' Fixed translation speed: 0.40 m/s [YAML:' <<< "${default_output}"
grep -Fq ' Arc turn: Move(0.40,0,+/-0.60) [YAML:' <<< "${default_output}"
grep -Fq ' External diagnostic upload: not part of the runtime pipeline' \
  <<< "${default_output}"
[[ "${default_output}" != *'Motion-response watchdog'* ]]
grep -Fq 'config:='"${workspace}"'/src/utree_go2_sdk2_bridge/config/go2_sdk2_bridge.yaml' \
  "${trace_file}"
test "$(grep -c 'max_vx:=' "${trace_file}" || true)" -eq 0
test "$(grep -c 'max_vy:=' "${trace_file}" || true)" -eq 0
test "$(grep -c 'max_yaw_rate:=' "${trace_file}" || true)" -eq 0

: > "${trace_file}"
override_output="$(
  env -u GO2_MAX_VX -u GO2_MAX_YAW_RATE \
    "${common_environment[@]}" GO2_MAX_VY=0.7 \
    "${workspace}/shell/start_sdk2_bridge.sh"
)"
grep -Fq ' vx=0.6 m/s [YAML:' <<< "${override_output}"
grep -Fq ' vy=0.7 m/s [environment: GO2_MAX_VY]' <<< "${override_output}"
grep -Fq ' yaw=0.8 rad/s [YAML:' <<< "${override_output}"
grep -Fq 'max_vy:=0.7' "${trace_file}"
test "$(grep -c 'max_vx:=' "${trace_file}" || true)" -eq 0
test "$(grep -c 'max_yaw_rate:=' "${trace_file}" || true)" -eq 0

echo "PASS: SDK2 startup is independent of external diagnostics, uses YAML defaults, and forwards only explicit velocity overrides"
