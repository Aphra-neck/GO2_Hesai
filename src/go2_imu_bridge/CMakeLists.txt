cmake_minimum_required(VERSION 3.8)
project(go2_imu_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unitree_sdk2 REQUIRED)

add_executable(go2_imu_bridge_node src/go2_imu_bridge.cpp)
target_compile_options(go2_imu_bridge_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(go2_imu_bridge_node rclcpp sensor_msgs)
target_link_libraries(go2_imu_bridge_node unitree_sdk2)

install(
  TARGETS go2_imu_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
