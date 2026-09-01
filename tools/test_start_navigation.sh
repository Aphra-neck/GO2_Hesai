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
  "${workspace}/src/utree_dog_navigation/config" \
  "${workspace}/src/utree_dog_navigation/rviz/renamed"

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

chmod +x \
  "${fake_bin}/ros2" \
  "${fake_bin}/timeout" \
  "${fake_bin}/pgrep"

cat > "${workspace}/src/utree_dog_navigation/config/terrain_navigation.yaml" <<'YAML'
body_lattice_planner:
  ros__parameters: {}
YAML

printf 'legacy terrain\n' > \
  "${workspace}/src/utree_dog_navigation/rviz/hesai_navigation.rviz"
printf 'flat obstacle\n' > \
  "${workspace}/src/utree_dog_navigation/rviz/flat_obstacle_navigation.rviz"
printf 'custom\n' > \
  "${workspace}/src/utree_dog_navigation/rviz/custom_navigation.rviz"
cp -- "${workspace}/src/utree_dog_navigation/rviz/hesai_navigation.rviz" \
  "${workspace}/src/utree_dog_navigation/rviz/renamed/flat_obstacle_navigation.rviz"

common_environment=(
  FAKE_BIN="${fake_bin}"
  FAKE_ROS2_TRACE="${trace_file}"
  PATH="${fake_bin}:/usr/bin:/bin"
  PLANNING_RVIZ=false
)
unset GO2_NAVIGATION_RVIZ_CONFIG GO2_ALLOW_CUSTOM_NAVIGATION_RVIZ_CONFIG

: > "${trace_file}"
set +e
default_unconfirmed_output="$(
  env -u GO2_VERIFIED_FLAT_START -u GO2_PLANNING_MODE \
    -u GO2_FLAT_GROUND_CONFIRMED -u GO2_ENABLE_LEGACY_TERRAIN \
    "${common_environment[@]}" \
    "${workspace}/shell/start_navigation.sh" 2>&1
)"
default_unconfirmed_status=$?
set -e
test "${default_unconfirmed_status}" -ne 0
[[ "${default_unconfirmed_output}" == \
  *"requires the robot to be standing and stationary before startup"* ]]
test ! -s "${trace_file}"

: > "${trace_file}"
default_flat_output="$(
  env -u GO2_VERIFIED_FLAT_START -u GO2_PLANNING_MODE \
    -u GO2_ENABLE_LEGACY_TERRAIN "${common_environment[@]}" \
    PLANNING_RVIZ=true \
    GO2_FLAT_GROUND_CONFIRMED=true \
    "${workspace}/shell/start_navigation.sh"
)"
grep -Fq ' Planning RViz: true' <<< "${default_flat_output}"
grep -Fq ' Verified flat start: false' <<< "${default_flat_output}"
grep -Fq ' Planning mode: flat_obstacle' <<< "${default_flat_output}"
grep -Fq ' Flat ground confirmed: true' <<< "${default_flat_output}"
grep -Fq ' Legacy terrain enabled: false' <<< "${default_flat_output}"
grep -Fq ' External diagnostic upload: not part of the runtime pipeline' \
  <<< "${default_flat_output}"
grep -Fq 'verified_flat_start:=false' "${trace_file}"
grep -Fq 'planning_mode:=flat_obstacle' "${trace_file}"
grep -Fq 'enable_legacy_terrain:=false' "${trace_file}"
grep -Fq 'allow_custom_rviz_config:=false' "${trace_file}"
grep -Fq 'flat_ground_confirmed:=true' "${trace_file}"
grep -Fq 'rviz_config:='"${workspace}"'/src/utree_dog_navigation/rviz/flat_obstacle_navigation.rviz' \
  "${trace_file}"

: > "${trace_file}"
set +e
unconfirmed_output="$(
  env -u GO2_ENABLE_LEGACY_TERRAIN "${common_environment[@]}" \
    GO2_PLANNING_MODE=flat_obstacle \
    "${workspace}/shell/start_navigation.sh" 2>&1
)"
unconfirmed_status=$?
set -e
test "${unconfirmed_status}" -ne 0
[[ "${unconfirmed_output}" == \
  *"requires the robot to be standing and stationary before startup"* ]]
test ! -s "${trace_file}"

: > "${trace_file}"
flat_output="$(
  env -u GO2_ENABLE_LEGACY_TERRAIN "${common_environment[@]}" \
    GO2_PLANNING_MODE=flat_obstacle \
    GO2_FLAT_GROUND_CONFIRMED=true \
    "${workspace}/shell/start_navigation.sh"
)"
grep -Fq ' Planning mode: flat_obstacle' <<< "${flat_output}"
grep -Fq ' Flat ground confirmed: true' <<< "${flat_output}"
grep -Fq ' Robot posture: STANDING and stationary required' <<< "${flat_output}"
grep -Fq 'planning_mode:=flat_obstacle' "${trace_file}"
grep -Fq 'enable_legacy_terrain:=false' "${trace_file}"
grep -Fq 'allow_custom_rviz_config:=false' "${trace_file}"
grep -Fq 'flat_ground_confirmed:=true' "${trace_file}"
grep -Fq 'rviz_config:='"${workspace}"'/src/utree_dog_navigation/rviz/flat_obstacle_navigation.rviz' \
  "${trace_file}"

: > "${trace_file}"
set +e
terrain_blocked_output="$(
  env -u GO2_ENABLE_LEGACY_TERRAIN "${common_environment[@]}" \
    GO2_PLANNING_MODE=terrain \
    "${workspace}/shell/start_navigation.sh" 2>&1
)"
terrain_blocked_status=$?
set -e
test "${terrain_blocked_status}" -ne 0
[[ "${terrain_blocked_output}" == \
  *"Legacy terrain mode requires GO2_ENABLE_LEGACY_TERRAIN=true"* ]]
test ! -s "${trace_file}"

: > "${trace_file}"
set +e
legacy_without_terrain_output="$(
  env "${common_environment[@]}" GO2_PLANNING_MODE=flat_obstacle \
    GO2_ENABLE_LEGACY_TERRAIN=true GO2_FLAT_GROUND_CONFIRMED=true \
    "${workspace}/shell/start_navigation.sh" 2>&1
)"
legacy_without_terrain_status=$?
set -e
test "${legacy_without_terrain_status}" -ne 0
[[ "${legacy_without_terrain_output}" == \
  *"GO2_ENABLE_LEGACY_TERRAIN=true requires GO2_PLANNING_MODE=terrain"* ]]
test ! -s "${trace_file}"

: > "${trace_file}"
set +e
flat_with_terrain_rviz_output="$(
  env "${common_environment[@]}" GO2_PLANNING_MODE=flat_obstacle \
    GO2_FLAT_GROUND_CONFIRMED=true \
    GO2_NAVIGATION_RVIZ_CONFIG="${workspace}/src/utree_dog_navigation/rviz/hesai_navigation.rviz" \
    "${workspace}/shell/start_navigation.sh" 2>&1
)"
flat_with_terrain_rviz_status=$?
set -e
test "${flat_with_terrain_rviz_status}" -ne 0
[[ "${flat_with_terrain_rviz_output}" == \
  *"flat_obstacle mode cannot use legacy terrain RViz config"* ]]
test ! -s "${trace_file}"

: > "${trace_file}"
set +e
custom_rviz_blocked_output="$(
  env -u GO2_ALLOW_CUSTOM_NAVIGATION_RVIZ_CONFIG "${common_environment[@]}" \
    GO2_PLANNING_MODE=flat_obstacle \
    GO2_FLAT_GROUND_CONFIRMED=true \
    GO2_NAVIGATION_RVIZ_CONFIG="${workspace}/src/utree_dog_navigation/rviz/custom_navigation.rviz" \
    "${workspace}/shell/start_navigation.sh" 2>&1
)"
custom_rviz_blocked_status=$?
set -e
test "${custom_rviz_blocked_status}" -ne 0
[[ "${custom_rviz_blocked_output}" == \
  *"Custom navigation RViz config requires GO2_ALLOW_CUSTOM_NAVIGATION_RVIZ_CONFIG=true"* ]]
test ! -s "${trace_file}"

: > "${trace_file}"
set +e
renamed_terrain_rviz_output="$(
  env "${common_environment[@]}" GO2_PLANNING_MODE=flat_obstacle \
    GO2_FLAT_GROUND_CONFIRMED=true \
    GO2_ALLOW_CUSTOM_NAVIGATION_RVIZ_CONFIG=true \
    GO2_NAVIGATION_RVIZ_CONFIG="${workspace}/src/utree_dog_navigation/rviz/renamed/flat_obstacle_navigation.rviz" \
    "${workspace}/shell/start_navigation.sh" 2>&1
)"
renamed_terrain_rviz_status=$?
set -e
test "${renamed_terrain_rviz_status}" -ne 0
[[ "${renamed_terrain_rviz_output}" == \
  *"flat_obstacle mode cannot use legacy terrain RViz config"* ]]
test ! -s "${trace_file}"

: > "${trace_file}"
custom_rviz_output="$(
  env "${common_environment[@]}" GO2_PLANNING_MODE=flat_obstacle \
    GO2_FLAT_GROUND_CONFIRMED=true \
    GO2_ALLOW_CUSTOM_NAVIGATION_RVIZ_CONFIG=true \
    GO2_NAVIGATION_RVIZ_CONFIG="${workspace}/src/utree_dog_navigation/rviz/custom_navigation.rviz" \
    "${workspace}/shell/start_navigation.sh"
)"
grep -Fq ' Custom RViz config authorized: true' <<< "${custom_rviz_output}"
grep -Fq 'allow_custom_rviz_config:=true' "${trace_file}"
grep -Fq 'rviz_config:='"${workspace}"'/src/utree_dog_navigation/rviz/custom_navigation.rviz' \
  "${trace_file}"

: > "${trace_file}"
terrain_output="$(
  env "${common_environment[@]}" GO2_PLANNING_MODE=terrain \
    GO2_ENABLE_LEGACY_TERRAIN=true \
    "${workspace}/shell/start_navigation.sh"
)"
grep -Fq ' Planning mode: terrain' <<< "${terrain_output}"
grep -Fq ' Flat ground confirmed: false' <<< "${terrain_output}"
grep -Fq ' Legacy terrain enabled: true' <<< "${terrain_output}"
grep -Fq 'planning_mode:=terrain' "${trace_file}"
grep -Fq 'enable_legacy_terrain:=true' "${trace_file}"
grep -Fq 'allow_custom_rviz_config:=false' "${trace_file}"
grep -Fq 'flat_ground_confirmed:=false' "${trace_file}"
grep -Fq 'rviz_config:='"${workspace}"'/src/utree_dog_navigation/rviz/hesai_navigation.rviz' \
  "${trace_file}"

: > "${trace_file}"
set +e
terrain_with_flat_rviz_output="$(
  env "${common_environment[@]}" GO2_PLANNING_MODE=terrain \
    GO2_ENABLE_LEGACY_TERRAIN=true \
    GO2_NAVIGATION_RVIZ_CONFIG="${workspace}/src/utree_dog_navigation/rviz/flat_obstacle_navigation.rviz" \
    "${workspace}/shell/start_navigation.sh" 2>&1
)"
terrain_with_flat_rviz_status=$?
set -e
test "${terrain_with_flat_rviz_status}" -ne 0
[[ "${terrain_with_flat_rviz_output}" == \
  *"terrain mode cannot use flat-obstacle RViz config"* ]]
test ! -s "${trace_file}"

: > "${trace_file}"
capture_output="$(
  env -u GO2_ENABLE_LEGACY_TERRAIN "${common_environment[@]}" \
    GO2_PLANNING_MODE=flat_obstacle \
    GO2_FLAT_GROUND_CONFIRMED=true GO2_MAP_CAPTURE=true \
    GO2_MAP_CAPTURE_DIR="${FIXTURE_ROOT}/map-exports" \
    "${workspace}/shell/start_navigation.sh"
)"
grep -Fq ' Diagnostic filtered 3D obstacle map capture: true' <<< "${capture_output}"
grep -Fq ' Filtered map output root: ' <<< "${capture_output}"
grep -Fq ' Filtered map limits: 120 snapshots, 100 MB' <<< "${capture_output}"
grep -Fq 'record_3d_maps:=true' "${trace_file}"

: > "${trace_file}"
enabled_output="$(
  env "${common_environment[@]}" GO2_PLANNING_MODE=terrain \
    GO2_ENABLE_LEGACY_TERRAIN=true GO2_VERIFIED_FLAT_START=true \
    "${workspace}/shell/start_navigation.sh"
)"
grep -Fq ' Verified flat start: true' <<< "${enabled_output}"
grep -Fq 'verified_flat_start:=true' "${trace_file}"

for invalid in TRUE False 1 yes; do
  : > "${trace_file}"
  set +e
  invalid_output="$(
    env "${common_environment[@]}" GO2_PLANNING_MODE=terrain \
      GO2_ENABLE_LEGACY_TERRAIN=true GO2_VERIFIED_FLAT_START="${invalid}" \
      "${workspace}/shell/start_navigation.sh" 2>&1
  )"
  invalid_status=$?
  set -e
  test "${invalid_status}" -ne 0
  [[ "${invalid_output}" == \
    *"GO2_VERIFIED_FLAT_START must be true or false, got: ${invalid}"* ]]
  test ! -s "${trace_file}"
done

for invalid in TRUE False 1 yes; do
  : > "${trace_file}"
  set +e
  invalid_output="$(
    env "${common_environment[@]}" \
      GO2_ALLOW_CUSTOM_NAVIGATION_RVIZ_CONFIG="${invalid}" \
      GO2_FLAT_GROUND_CONFIRMED=true \
      "${workspace}/shell/start_navigation.sh" 2>&1
  )"
  invalid_status=$?
  set -e
  test "${invalid_status}" -ne 0
  [[ "${invalid_output}" == \
    *"GO2_ALLOW_CUSTOM_NAVIGATION_RVIZ_CONFIG must be true or false, got: ${invalid}"* ]]
  test ! -s "${trace_file}"
done

for invalid in TRUE False 1 yes; do
  : > "${trace_file}"
  set +e
  invalid_output="$(
    env "${common_environment[@]}" GO2_ENABLE_LEGACY_TERRAIN="${invalid}" \
      GO2_FLAT_GROUND_CONFIRMED=true \
      "${workspace}/shell/start_navigation.sh" 2>&1
  )"
  invalid_status=$?
  set -e
  test "${invalid_status}" -ne 0
  [[ "${invalid_output}" == \
    *"GO2_ENABLE_LEGACY_TERRAIN must be true or false, got: ${invalid}"* ]]
  test ! -s "${trace_file}"
done

for invalid in flat FLAT_OBSTACLE 2d; do
  : > "${trace_file}"
  set +e
  invalid_output="$(
    env "${common_environment[@]}" GO2_PLANNING_MODE="${invalid}" \
      "${workspace}/shell/start_navigation.sh" 2>&1
  )"
  invalid_status=$?
  set -e
  test "${invalid_status}" -ne 0
  [[ "${invalid_output}" == \
    *"GO2_PLANNING_MODE must be terrain or flat_obstacle, got: ${invalid}"* ]]
  test ! -s "${trace_file}"
done

for invalid in TRUE False 1 yes; do
  : > "${trace_file}"
  set +e
  invalid_output="$(
    env "${common_environment[@]}" GO2_FLAT_GROUND_CONFIRMED="${invalid}" \
      "${workspace}/shell/start_navigation.sh" 2>&1
  )"
  invalid_status=$?
  set -e
  test "${invalid_status}" -ne 0
  [[ "${invalid_output}" == \
    *"GO2_FLAT_GROUND_CONFIRMED must be true or false, got: ${invalid}"* ]]
  test ! -s "${trace_file}"
done

echo "PASS: navigation is independent of external diagnostics and locks flat-obstacle defaults and RViz pairing"
