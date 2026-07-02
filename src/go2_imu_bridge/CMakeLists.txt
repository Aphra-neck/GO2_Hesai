cmake_minimum_required(VERSION 3.0.2)
project(go2_imu_bridge)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

find_package(unitree_sdk2 REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(go2_imu_bridge_node src/go2_imu_bridge.cpp)

target_link_libraries(go2_imu_bridge_node
  ${catkin_LIBRARIES}
  unitree_sdk2
)
