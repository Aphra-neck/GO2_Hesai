#!/usr/bin/env bash
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
test_binary="/tmp/go2_test_measurement_synchronizer"

/usr/bin/g++ \
  -std=c++20 \
  -Wall \
  -Wextra \
  -Werror \
  -I "${repo_root}/src/Super-LIO/src/super_lio/include" \
  "${repo_root}/src/Super-LIO/src/super_lio/test/test_measurement_synchronizer.cpp" \
  -lgtest \
  -lgtest_main \
  -pthread \
  -o "${test_binary}"

"${test_binary}"
