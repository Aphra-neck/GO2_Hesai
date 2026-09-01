cmake_minimum_required(VERSION 3.10)
project(go2_sdk2_motion_stall_reader LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)
find_package(unitree_sdk2 REQUIRED CONFIG)

add_executable(go2_sdk2_motion_stall_reader state_reader.cpp)
target_compile_options(go2_sdk2_motion_stall_reader PRIVATE
  -Wall
  -Wextra
  -Wpedantic)
target_link_libraries(go2_sdk2_motion_stall_reader PRIVATE
  unitree_sdk2
  Threads::Threads)
