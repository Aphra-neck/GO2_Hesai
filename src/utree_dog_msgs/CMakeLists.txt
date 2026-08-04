cmake_minimum_required(VERSION 3.8)
project(utree_dog_msgs)

# Supported platform: Ubuntu 22.04 with ROS 2 Humble.
if(NOT DEFINED ENV{ROS_DISTRO})
  message(FATAL_ERROR
    "ROS_DISTRO is not set. Source /opt/ros/humble/setup.bash before building.")
elseif(NOT "$ENV{ROS_DISTRO}" STREQUAL "humble")
  message(WARNING
    "utree_dog_msgs is validated on ROS 2 Humble; detected '$ENV{ROS_DISTRO}'.")
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TerrainGrid.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
