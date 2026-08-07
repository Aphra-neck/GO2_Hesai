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
trace_file="${FIXTURE_ROOT}/ros2-calls.txt"
mkdir -p -- \
  "${fake_bin}" \
  "${workspace}/shell" \
  "${workspace}/tools" \
  "${workspace}/src/utree_dog_navigation/config"

cp -- "${REPO_ROOT}/shell/start_navigation.sh" \
  "${workspace}/shell/start_navigation.sh"
chmod +x "${workspace}/shell/start_navigation.sh"

cat > "${workspace}/shell/ros2_environment.sh" <<'SH'
#!/usr/bin/env bash
export PATH="${FAKE_BIN}:${PATH}"
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fake-fastdds.xml
export ROS_LOCALHOST_ONLY=0
SH

cat > "${fake_bin}/ros2" <<'SH'
#!/usr/bin/env bash
set -Eeuo pipefail
printf '%s\n' "$*" >> "${FAKE_ROS2_TRACE}"
case "$*" in
  'pkg prefix utree_dog_navigation'|\
  'topic echo --once --qos-profile sensor_data /lio/odom nav_msgs/msg/Odometry'|\
  'topic echo --once --qos-profile sensor_data /lio/cloud_world sensor_msgs/msg/PointCloud2'|\
  'launch utree_dog_navigation terrain_navigation.launch.py '*)
    exit 0
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

cat > "${fake_bin}/pgrep" <<'SH'
#!/usr/bin/env bash
exit 1
SH

cat > "${workspace}/tools/go2-log" <<'SH'
#!/usr/bin/env bash
set -Eeuo pipefail
[[ "$*" == start ]]
SH

chmod +x \
  "${fake_bin}/ros2" \
  "${fake_bin}/timeout" \
  "${fake_bin}/pgrep" \
  "${workspace}/tools/go2-log"

cat > "${workspace}/src/utree_dog_navigation/config/terrain_navigation.yaml" <<'YAML'
body_lattice_planner:
  ros__parameters: {}
YAML

common_environment=(
  FAKE_BIN="${fake_bin}"
  FAKE_ROS2_TRACE="${trace_file}"
  PATH="${fake_bin}:/usr/bin:/bin"
  PLANNING_RVIZ=false
)

: > "${trace_file}"
default_output="$(
  env -u GO2_VERIFIED_FLAT_START "${common_environment[@]}" \
    "${workspace}/shell/start_navigation.sh"
)"
grep -Fq ' Verified flat start: false' <<< "${default_output}"
grep -Fq 'verified_flat_start:=false' "${trace_file}"

: > "${trace_file}"
enabled_output="$(
  env "${common_environment[@]}" GO2_VERIFIED_FLAT_START=true \
    "${workspace}/shell/start_navigation.sh"
)"
grep -Fq ' Verified flat start: true' <<< "${enabled_output}"
grep -Fq 'verified_flat_start:=true' "${trace_file}"

for invalid in TRUE False 1 yes; do
  : > "${trace_file}"
  set +e
  invalid_output="$(
    env "${common_environment[@]}" GO2_VERIFIED_FLAT_START="${invalid}" \
      "${workspace}/shell/start_navigation.sh" 2>&1
  )"
  invalid_status=$?
  set -e
  test "${invalid_status}" -ne 0
  [[ "${invalid_output}" == \
    *"GO2_VERIFIED_FLAT_START must be true or false, got: ${invalid}"* ]]
  test ! -s "${trace_file}"
done

echo "PASS: navigation startup strictly validates and forwards verified-flat-start"
