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
ros2_trace="${FIXTURE_ROOT}/ros2-calls.txt"
pgrep_trace="${FIXTURE_ROOT}/pgrep-calls.txt"
mkdir -p -- \
  "${fake_bin}" \
  "${unitree_lib}" \
  "${workspace}/shell" \
  "${workspace}/src/utree_go2_sdk2_bridge/config"

cp -- "${REPO_ROOT}/shell/start_sdk2_direct_bridge.sh" \
  "${workspace}/shell/start_sdk2_direct_bridge.sh"
chmod +x "${workspace}/shell/start_sdk2_direct_bridge.sh"

cat > "${workspace}/shell/ros2_environment.sh" <<'SH'
#!/usr/bin/env bash
export PATH="${FAKE_BIN}:${PATH}"
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fake-fastdds.xml
export ROS_LOCALHOST_ONLY=0
SH

cp -- \
  "${REPO_ROOT}/src/utree_go2_sdk2_bridge/config/go2_sdk2_direct_bridge.yaml" \
  "${workspace}/src/utree_go2_sdk2_bridge/config/go2_sdk2_direct_bridge.yaml"

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
  'launch utree_go2_sdk2_bridge go2_sdk2_direct_bridge.launch.py '* )
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
printf '%s\n' "$*" >> "${FAKE_PGREP_TRACE}"
if [[ "${FAKE_OLD_BRIDGE_RUNNING:-false}" == true &&
      "$*" == *"go2_sdk2_bridge_node"* ]]; then
  echo '4242 /opt/go2/go2_sdk2_bridge_node'
  exit 0
fi
exit 1
SH

chmod +x "${fake_bin}/ros2" "${fake_bin}/timeout" "${fake_bin}/ip" "${fake_bin}/pgrep"
touch "${unitree_lib}/libddsc.so.0" "${unitree_lib}/libddscxx.so.0"

common_environment=(
  FAKE_BIN="${fake_bin}"
  FAKE_ROS2_TRACE="${ros2_trace}"
  FAKE_PGREP_TRACE="${pgrep_trace}"
  FAKE_WORKSPACE="${workspace}"
  PATH="${fake_bin}:/usr/bin:/bin"
  UNITREE_SDK_LIBRARY_DIR="${unitree_lib}"
)

set +e
default_output="$(
  env "${common_environment[@]}" \
    "${workspace}/shell/start_sdk2_direct_bridge.sh" 2>&1
)"
default_status=$?
set -e
test "${default_status}" -ne 0
[[ "${default_output}" == *"Obstacle-unaware direct bridge is disabled by default"* ]]
[[ "${default_output}" == *"Use ./shell/start_sdk2_bridge.sh to follow the planner-produced, obstacle-checked /body_path"* ]]

env "${common_environment[@]}" \
  GO2_ALLOW_OBSTACLE_UNAWARE_DIRECT_BRIDGE=true \
  "${workspace}/shell/start_sdk2_direct_bridge.sh"
grep -Fq \
  'launch utree_go2_sdk2_bridge go2_sdk2_direct_bridge.launch.py' \
  "${ros2_trace}"
grep -Fq 'go2_sdk2_direct_bridge_node' "${pgrep_trace}"
! grep -Eq 'terrain_mapper_node|body_lattice_planner_node|body_odom_adapter_node' \
  "${pgrep_trace}"

set +e
old_bridge_output="$(
  env "${common_environment[@]}" \
    GO2_ALLOW_OBSTACLE_UNAWARE_DIRECT_BRIDGE=true \
    FAKE_OLD_BRIDGE_RUNNING=true \
    "${workspace}/shell/start_sdk2_direct_bridge.sh" 2>&1
)"
old_bridge_status=$?
set -e
test "${old_bridge_status}" -ne 0
[[ "${old_bridge_output}" == *"An SDK2 motion bridge is already running"* ]]

echo "PASS: obstacle-unaware direct SDK2 bridge requires explicit commissioning authorization"
