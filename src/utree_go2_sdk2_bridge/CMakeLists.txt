cmake_minimum_required(VERSION 3.8)
project(utree_go2_sdk2_bridge)

if(NOT DEFINED ENV{ROS_DISTRO})
  message(FATAL_ERROR
    "ROS_DISTRO is not set. Source /opt/ros/humble/setup.bash before building.")
elseif(NOT "$ENV{ROS_DISTRO}" STREQUAL "humble")
  message(WARNING "This package is validated on ROS 2 Humble; detected '$ENV{ROS_DISTRO}'.")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(unitree_sdk2 REQUIRED)

# Some SDK2 releases omit the CycloneDDS C++ include directory from the
# exported target even though the SportClient headers include generated DDS
# types. Keep the lookup explicit so the bridge builds against the Jetson SDK.
find_path(UNITREE_DDSCXX_INCLUDE_DIR
  NAMES org/eclipse/cyclonedds/topic/datatopic.hpp
  PATH_SUFFIXES include/ddscxx)
if(NOT UNITREE_DDSCXX_INCLUDE_DIR)
  message(FATAL_ERROR
    "CycloneDDS C++ headers required by Unitree SDK2 messages were not found")
endif()

add_library(control_safety STATIC src/control_safety.cpp)
target_include_directories(control_safety PUBLIC include)
ament_target_dependencies(control_safety geometry_msgs)

add_executable(go2_sdk2_bridge_node
  src/go2_sdk2_bridge_node.cpp
  src/go2_sdk2_bridge_main.cpp)
target_include_directories(go2_sdk2_bridge_node PRIVATE include)
target_include_directories(go2_sdk2_bridge_node PRIVATE ${UNITREE_DDSCXX_INCLUDE_DIR})
target_link_libraries(go2_sdk2_bridge_node control_safety unitree_sdk2)
ament_target_dependencies(go2_sdk2_bridge_node
  geometry_msgs nav_msgs rclcpp rcl_interfaces std_srvs tf2 tf2_geometry_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_control_safety test/test_control_safety.cpp)
  target_link_libraries(test_control_safety control_safety)
  ament_add_pytest_test(
    test_go2_sdk2_bridge_launch
    test/test_go2_sdk2_bridge_launch.py)
  ament_add_pytest_test(
    test_official_move_surface
    test/test_official_move_surface.py)
endif()

install(TARGETS go2_sdk2_bridge_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.py[co]" EXCLUDE)

# Exact cleanup for artifacts installed by the retired obstacle-unaware direct
# bridge. Incremental colcon installs do not remove files whose source target
# disappeared, so remove only the known package-local leftovers.
install(CODE [[
  set(_retired_sdk2_bridge_artifacts
    "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/utree_go2_sdk2_bridge/go2_sdk2_direct_bridge_node"
    "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsimple_navigation_geometry.a"
    "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsimple_navigation_controller.a"
    "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/utree_go2_sdk2_bridge/config/go2_sdk2_direct_bridge.yaml"
    "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/utree_go2_sdk2_bridge/launch/go2_sdk2_direct_bridge.launch.py")
  foreach(_artifact IN LISTS _retired_sdk2_bridge_artifacts)
    if(EXISTS "${_artifact}" OR IS_SYMLINK "${_artifact}")
      file(REMOVE "${_artifact}")
    endif()
  endforeach()
  set(_retired_sdk2_bridge_pycache
    "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/utree_go2_sdk2_bridge/launch/__pycache__")
  if(IS_DIRECTORY "${_retired_sdk2_bridge_pycache}")
    file(REMOVE_RECURSE "${_retired_sdk2_bridge_pycache}")
  endif()
]])

ament_package()
