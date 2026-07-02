#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
source /home/unitree/catkin_ws/devel/setup.bash

LOG_DIR="/home/unitree/slam_logs"
mkdir -p "$LOG_DIR"

echo "======================================"
echo " Go2 + Hesai XT-16 + Super-LIO 启动"
echo "======================================"

cleanup() {
  echo ""
  echo "正在关闭 SLAM 相关节点..."

  rosnode kill /super_lio_node 2>/dev/null || true
  rosnode kill /super_lio 2>/dev/null || true
  rosnode kill /go2_imu_bridge 2>/dev/null || true
  rosnode kill /hesai_ros_driver_node 2>/dev/null || true

  pkill -f "roslaunch hesai_ros_driver start.launch" 2>/dev/null || true
  pkill -f "rosrun go2_imu_bridge go2_imu_bridge_node" 2>/dev/null || true
  pkill -f "roslaunch super_lio hesai_XT16.launch" 2>/dev/null || true

  echo "已关闭。"
}
trap cleanup EXIT INT TERM

echo "[1/3] 启动 Hesai 雷达驱动..."
roslaunch hesai_ros_driver start.launch rviz:=false > "$LOG_DIR/hesai.log" 2>&1 &
sleep 5

echo "检查 /lidar_points..."
timeout 10 bash -c 'until rostopic echo -n 1 /lidar_points/header >/dev/null 2>&1; do sleep 1; done'
echo "/lidar_points 正常。"

echo "[2/3] 启动 Go2 IMU bridge..."
rosrun go2_imu_bridge go2_imu_bridge_node _net:=eth10 _publish_rate:=200 > "$LOG_DIR/go2_imu_bridge.log" 2>&1 &
sleep 3

echo "检查 /imu/data..."
timeout 10 bash -c 'until rostopic echo -n 1 /imu/data/header >/dev/null 2>&1; do sleep 1; done'
echo "/imu/data 正常。"

echo "[3/3] 启动 Super-LIO..."
echo ""
echo "启动完成后，另开命令检查位姿："
echo "  rostopic echo /lio/odom"
echo ""
echo "日志目录：$LOG_DIR"
echo ""

roslaunch super_lio hesai_XT16.launch
