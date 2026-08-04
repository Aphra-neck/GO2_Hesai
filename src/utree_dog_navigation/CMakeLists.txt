cmake_minimum_required(VERSION 3.8)
project(utree_dog_navigation)

# Supported platform: Ubuntu 22.04 with ROS 2 Humble.
if(NOT DEFINED ENV{ROS_DISTRO})
  message(FATAL_ERROR
    "ROS_DISTRO is not set. Source /opt/ros/humble/setup.bash before building.")
elseif(NOT "$ENV{ROS_DISTRO}" STREQUAL "humble")
  message(WARNING
    "utree_dog_navigation is validated on ROS 2 Humble; detected '$ENV{ROS_DISTRO}'.")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(utree_dog_msgs REQUIRED)

add_library(terrain_mapping_core
  src/terrain_map_builder.cpp)
target_include_directories(terrain_mapping_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(terrain_mapping_core
  builtin_interfaces utree_dog_msgs)

add_executable(terrain_mapper_node
  src/terrain_mapper_node.cpp
  src/terrain_mapper_main.cpp)
target_link_libraries(terrain_mapper_node terrain_mapping_core)
ament_target_dependencies(terrain_mapper_node
  builtin_interfaces nav_msgs rclcpp sensor_msgs utree_dog_msgs)

add_library(body_planning_core
  src/lattice_planner.cpp)
target_include_directories(body_planning_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(body_planning_core
  utree_dog_msgs)

add_executable(body_lattice_planner_node
  src/body_lattice_planner_node.cpp
  src/body_lattice_planner_main.cpp)
target_link_libraries(body_lattice_planner_node body_planning_core)
ament_target_dependencies(body_lattice_planner_node
  geometry_msgs nav_msgs rclcpp tf2 tf2_geometry_msgs utree_dog_msgs)

install(TARGETS
  terrain_mapping_core
  body_planning_core
  terrain_mapper_node
  body_lattice_planner_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_terrain_map_builder test/test_terrain_map_builder.cpp)
  target_link_libraries(test_terrain_map_builder terrain_mapping_core)
  ament_target_dependencies(test_terrain_map_builder utree_dog_msgs)

  ament_add_gtest(test_lattice_planner test/test_lattice_planner.cpp)
  target_link_libraries(test_lattice_planner body_planning_core)
  ament_target_dependencies(test_lattice_planner utree_dog_msgs)
endif()

ament_package()
