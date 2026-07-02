# Go2 + XT-16 + Super-LIO 部署与启动说明

本文档记录 Unitree Go2 机器狗接入 Hesai XT-16 雷达、Go2 内部 IMU，并使用 ROS1 Noetic + Super-LIO 进行定位与建图的流程。

当前稳定链路：

```text
XT-16 雷达
  -> Hesai ROS Driver
  -> /lidar_points

Go2 内部 IMU
  -> Unitree SDK2 DDS rt/lowstate
  -> go2_imu_bridge
  -> /imu/data

/lidar_points + /imu/data
  -> Super-LIO
  -> /lio/odom
  -> /lio/cloud_world
```

## 1. 工程路径与编译

### 1.1 工作空间路径

ROS1 工作空间：

```bash
~/catkin_ws
```

源码目录：

```bash
~/catkin_ws/src
```

建议启动脚本目录：

```bash
~/catkin_ws/shell
```

### 1.2 编译整个工作空间

第一次部署、修改任意源码、修改 `go2_imu_bridge` 后，建议先编译整个工作空间：

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

### 1.3 单独编译 go2_imu_bridge

如果只修改了 IMU bridge，可以单独编译：

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg go2_imu_bridge
source devel/setup.bash
```

### 1.4 编译后检查

检查 ROS 是否能找到 `go2_imu_bridge` 节点：

```bash
rosrun go2_imu_bridge go2_imu_bridge_node --help
```

如果没有提示找不到包或找不到节点，说明编译和环境加载正常。

检查常用包是否能被 ROS 找到：

```bash
rospack find hesai_ros_driver
rospack find go2_imu_bridge
rospack find super_lio
```

## 2. 一键启动流程

### 2.1 创建启动脚本

建议脚本放在：

```bash
~/catkin_ws/shell/start_slam.sh
```

创建目录：

```bash
mkdir -p ~/catkin_ws/shell
gedit ~/catkin_ws/shell/start_slam.sh
```

脚本内容：

```bash
#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
source /home/unitree/catkin_ws/devel/setup.bash

LOG_DIR="/home/unitree/slam_logs"
mkdir -p "$LOG_DIR"

echo "======================================"
echo " Go2 + Hesai XT-16 + Super-LIO start"
echo "======================================"

cleanup() {
  echo ""
  echo "Stopping SLAM nodes..."

  rosnode kill /super_lio_node 2>/dev/null || true
  rosnode kill /super_lio 2>/dev/null || true
  rosnode kill /go2_imu_bridge 2>/dev/null || true
  rosnode kill /hesai_ros_driver_node 2>/dev/null || true

  pkill -f "roslaunch hesai_ros_driver start.launch" 2>/dev/null || true
  pkill -f "rosrun go2_imu_bridge go2_imu_bridge_node" 2>/dev/null || true
  pkill -f "roslaunch super_lio hesai_XT16.launch" 2>/dev/null || true

  echo "Stopped."
}
trap cleanup EXIT INT TERM

echo "[1/3] Starting Hesai XT-16 driver..."
roslaunch hesai_ros_driver start.launch rviz:=false > "$LOG_DIR/hesai.log" 2>&1 &
sleep 5

echo "Checking /lidar_points..."
timeout 10 bash -c 'until rostopic echo -n 1 /lidar_points/header >/dev/null 2>&1; do sleep 1; done'
echo "/lidar_points OK."

echo "[2/3] Starting Go2 IMU bridge..."
rosrun go2_imu_bridge go2_imu_bridge_node _net:=eth10 _publish_rate:=200 > "$LOG_DIR/go2_imu_bridge.log" 2>&1 &
sleep 3

echo "Checking /imu/data..."
timeout 10 bash -c 'until rostopic echo -n 1 /imu/data/header >/dev/null 2>&1; do sleep 1; done'
echo "/imu/data OK."

echo "[3/3] Starting Super-LIO..."
echo ""
echo "Check pose in another terminal:"
echo "  rostopic echo /lio/odom"
echo ""
echo "Logs: $LOG_DIR"
echo ""

roslaunch super_lio hesai_XT16.launch rviz:=false
```

赋予执行权限：

```bash
chmod +x ~/catkin_ws/shell/start_slam.sh
```

启动：

```bash
~/catkin_ws/shell/start_slam.sh
```

如果当前在 `/home/unitree`，也可以：

```bash
./catkin_ws/shell/start_slam.sh
```

停止：

```text
Ctrl+C
```

脚本会尝试自动关闭 Hesai、IMU bridge 和 Super-LIO 相关节点。

## 3. 启动后的检查命令

### 3.1 检查雷达点云

```bash
rostopic hz /lidar_points
rostopic echo -n 1 /lidar_points/header
```

正常现象：

```text
/lidar_points 约 10 Hz
frame_id: "hesai_lidar"
```

### 3.2 检查 Go2 IMU

```bash
rostopic hz /imu/data
rostopic echo -n 1 /imu/data
```

正常现象：

```text
/imu/data 约 160-200 Hz
frame_id: "go2_imu"
orientation_covariance[0] = -1.0
```

### 3.3 检查 Super-LIO 位姿

```bash
rostopic hz /lio/odom
rostopic echo /lio/odom
```

正常现象：

```text
/lio/odom 约 10 Hz
机器狗静止时 position 基本稳定
twist.linear 不应出现几十 m/s 的异常速度
```

### 3.4 检查建图输出

```bash
rostopic hz /lio/cloud_world
rostopic list | grep lio
```

常见输出：

```text
/lio/odom
/lio/cloud_world
/lio/imu/odom
/lio/robo/odom
/lio/path
/lio/path_robot
```

## 4. 参数修改位置与当前稳定值

### 4.1 Hesai 雷达 SDK 参数

配置文件通常位于：

```bash
~/catkin_ws/src/HesaiLidar_ROS_2.0/hesai_ros_driver/config/config.yaml
```

如果路径不确定：

```bash
grep -R "use_timestamp_type" -n ~/catkin_ws/src
```

XT-16 当前实测网络参数：

```yaml
device_ip: 192.168.123.20
host_ip: 192.168.123.18
lidar_recv_port: 2368

lidar_ip: 192.168.123.20
pcap_play: false
device_udp_src_port: 10000
udp_port: 2368
```

Hesai 配置中的关键参数：

```yaml
lidar:
  - driver:
      source_type: 1
      lidar_udp_type:
        device_ip_address: 192.168.123.20
        host_ip_address: 192.168.123.18
        udp_port: 2368
        device_udp_src_port: 10000
        use_ptc_connected: true

      use_timestamp_type: 1
      transform_flag: false
      enable_packet_loss_tool: true

    ros:
      ros_frame_id: hesai_lidar
      ros_send_point_cloud_topic: /lidar_points
      ros_send_imu_topic: /lidar_imu
      ros_send_packet_loss_topic: /lidar_packets_loss
      send_point_cloud_ros: true
      send_imu_ros: true
```

最关键参数：

```yaml
use_timestamp_type: 1
```

它让点云使用机器狗接收时间，与 `/imu/data` 的系统时间对齐。

### 4.2 Go2 IMU bridge 参数

启动命令：

```bash
rosrun go2_imu_bridge go2_imu_bridge_node _net:=eth10 _publish_rate:=200
```

关键参数：

```text
_net:=eth10
_publish_rate:=200
```

当前稳定策略：

```text
通过 eth10 监听 Unitree SDK2 DDS 数据
订阅 rt/lowstate
发布 /imu/data
限制 IMU 发布频率约 160-200 Hz
orientation_covariance[0] = -1.0
```

### 4.3 Super-LIO 参数

配置文件：

```bash
~/catkin_ws/src/Super-LIO/src/super_lio/config/hesai_XT16.yaml
```

当前稳定参数：

```yaml
lio:
  ros:
    lidar_topic: "/lidar_points"
    imu_topic: "/imu/data"

  sensor:
    lidar_type: 2
    blind: 0.5
    maxrange: 150.0
    filter_rate: 1.0
    enable_downsample: true
    voxel_fliter_size: 0.3
    gravity_norm: 9.4188
    imu_type: 1
    imu_na: 0.1
    imu_ng: 0.1
    imu_nba: 0.0001
    imu_nbg: 0.0001

  extrinsic:
    lidar_imu: [0.171, 0.0, 0.0908,
                1.0,   0.0, 0.0,
                0.0,   1.0, 0.0,
                0.0,   0.0, 1.0]
    odom_robo: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

  output:
    robot: false
    plan_env_world: false
    plan_env_body: false
    planner: false
    map: true
    dense: true
    pub_step: 10
```

## 5. 参数功能说明

### 5.1 雷达网络参数

`device_ip_address` / `device_ip` / `lidar_ip`

XT-16 雷达 IP：

```text
192.168.123.20
```

`host_ip_address` / `host_ip`

Go2 接收雷达数据的 `eth10` 网口 IP：

```text
192.168.123.18
```

`udp_port` / `lidar_recv_port`

Go2 接收点云的 UDP 端口：

```text
2368
```

`device_udp_src_port`

雷达发送点云使用的 UDP 源端口：

```text
10000
```

可通过 tcpdump 验证：

```bash
sudo tcpdump -i eth10 -nn 'udp port 2368' -c 10
```

正常应看到：

```text
192.168.123.20.10000 > 192.168.123.18.2368
```

`pcap_play`

是否播放 pcap 包：

```text
false: 使用实时雷达
true: 播放 pcap 数据
```

当前使用实时雷达，因此为：

```yaml
pcap_play: false
```

### 5.2 Hesai SDK 参数

`source_type`

数据源类型：

```text
1: 实时雷达
2: pcap
3: packet rosbag
4: serial
```

当前为：

```yaml
source_type: 1
```

`use_timestamp_type`

点云时间戳类型：

```text
0: 使用雷达内部时间
1: 使用机器狗接收时间
```

当前必须使用：

```yaml
use_timestamp_type: 1
```

否则 `/lidar_points` 和 `/imu/data` 时间戳不一致，Super-LIO 可能没有输出或直接发散。

`transform_flag`

Hesai SDK 内部是否对点云做坐标变换：

```yaml
transform_flag: false
```

外参由 Super-LIO 的 `lidar_imu` 处理，不在 Hesai SDK 中重复变换。

`ros_send_point_cloud_topic`

点云发布话题：

```text
/lidar_points
```

`ros_send_packet_loss_topic`

雷达丢包统计话题：

```text
/lidar_packets_loss
```

### 5.3 Super-LIO 话题参数

`lidar_topic`

Super-LIO 订阅的点云话题：

```yaml
lidar_topic: "/lidar_points"
```

`imu_topic`

Super-LIO 订阅的 IMU 话题：

```yaml
imu_topic: "/imu/data"
```

### 5.4 Super-LIO 传感器参数

`lidar_type`

雷达类型。当前 XT-16 使用：

```yaml
lidar_type: 2
```

`blind`

近距离盲区过滤，小于该距离的点会被忽略：

```yaml
blind: 0.5
```

之前 `blind: 2.0` 会过滤太多室内近距离点，使点云约束变弱。

`filter_rate`

点云抽点比例：

```yaml
filter_rate: 1.0
```

XT-16 只有 16 线，不建议过度抽点。

`voxel_fliter_size`

体素降采样尺寸：

```yaml
voxel_fliter_size: 0.3
```

数值越小点云越密、算力压力越大；数值越大点云越稀、定位约束越弱。

`gravity_norm`

IMU 静止时重力模长。本机实测：

```text
9.4188
```

当前配置：

```yaml
gravity_norm: 9.4188
```

测量命令：

```bash
python3 - <<'PY'
import rospy, math
from sensor_msgs.msg import Imu

vals = []

def cb(msg):
    a = msg.linear_acceleration
    vals.append(math.sqrt(a.x*a.x + a.y*a.y + a.z*a.z))
    if len(vals) >= 500:
        rospy.signal_shutdown("done")

rospy.init_node("imu_norm_check", anonymous=True)
rospy.Subscriber("/imu/data", Imu, cb, queue_size=1000)
rospy.spin()

print("count:", len(vals))
print("avg:", sum(vals) / len(vals))
print("min:", min(vals))
print("max:", max(vals))
PY
```

### 5.5 外参参数

`lidar_imu`

表示 XT-16 雷达相对于 Go2 内部 IMU 的平移和旋转。

当前文档给定：

```text
平移: x=0.171, y=0, z=0.0908
旋转: 无相对旋转
```

配置：

```yaml
lidar_imu: [0.171, 0.0, 0.0908,
            1.0,   0.0, 0.0,
            0.0,   1.0, 0.0,
            0.0,   0.0, 1.0]
```

前三个数是平移，后九个数是旋转矩阵。

### 5.6 输出参数

`map`

是否发布世界坐标系点云地图：

```yaml
map: true
```

`dense`

是否发布较密集点云：

```yaml
dense: true
```

`pub_step`

控制点云地图发布频率：

```yaml
pub_step: 10
```

表示大约每 10 帧发布一次。调试 RViz 时如果想更频繁显示，可改为：

```yaml
pub_step: 1
```

## 6. Go2 IMU bridge 工作原理

`go2_imu_bridge` 不是 ROS2 bridge，它是一个 DDS 到 ROS1 的小适配器。

数据流：

```text
Go2 本体控制系统
  -> Unitree SDK2 / DDS 话题 rt/lowstate
  -> go2_imu_bridge_node
  -> ROS1 sensor_msgs/Imu
  -> /imu/data
  -> Super-LIO
```

启动命令：

```bash
rosrun go2_imu_bridge go2_imu_bridge_node _net:=eth10 _publish_rate:=200
```

`_net:=eth10` 表示让 Unitree SDK2 从 `eth10` 网口监听 Go2 的 DDS 数据。`eth10` 是机器狗本体的 `192.168.123.x` 内部网段，因此可以收到 `rt/lowstate`。

bridge 内部主要做四件事。

第一，初始化 Unitree SDK2 通道：

```cpp
unitree::robot::ChannelFactory::Instance()->Init(0, net);
```

第二，订阅 Go2 低状态 DDS 话题：

```cpp
TOPIC_LOWSTATE = "rt/lowstate"
```

第三，从 `LowState` 中取出 IMU：

```cpp
state->imu_state()
```

主要使用三组数据：

```cpp
quaternion()      // 姿态四元数，Unitree 顺序为 w, x, y, z
gyroscope()       // 角速度，单位 rad/s
accelerometer()   // 线加速度，单位 m/s^2
```

第四，转换成 ROS1 标准消息并发布：

```cpp
sensor_msgs::Imu
```

发布话题：

```bash
/imu/data
```

bridge 使用：

```cpp
msg.header.stamp = ros::Time::now();
```

这让 IMU 时间戳使用机器狗当前系统时间，并与 Hesai 点云 `use_timestamp_type: 1` 后的时间对齐。

为了避免 Go2 lowstate 中的绝对姿态四元数影响 Super-LIO，当前设置：

```cpp
msg.orientation_covariance[0] = -1.0;
```

该 bridge 只订阅状态，不发送控制命令，不会控制机器狗。

## 7. 常见问题

### 7.1 `/lidar_points` 没数据

检查 Hesai 驱动：

```bash
rosnode list | grep hesai
rostopic info /lidar_points
rostopic hz /lidar_points
```

检查 UDP 是否进来：

```bash
sudo tcpdump -i eth10 -nn 'udp port 2368' -c 10
```

正常应看到：

```text
192.168.123.20.10000 > 192.168.123.18.2368
```

### 7.2 `/imu/data` 没数据

检查 bridge：

```bash
rosnode list | grep go2_imu
rostopic info /imu/data
rostopic hz /imu/data
```

如未启动：

```bash
rosrun go2_imu_bridge go2_imu_bridge_node _net:=eth10 _publish_rate:=200
```

### 7.3 `/lio/odom` 没数据

检查 Super-LIO 是否订阅正确话题：

```bash
rosnode info /super_lio_node
```

应看到：

```text
Subscriptions:
 * /lidar_points [sensor_msgs/PointCloud2]
 * /imu/data [sensor_msgs/Imu]
```

检查时间戳：

```bash
rostopic echo -n 1 /lidar_points/header
rostopic echo -n 1 /imu/data/header
date +%s
```

`secs` 应接近当前系统时间。

### 7.4 位姿发散

典型表现：

```text
position 突然变成几百或几千
twist.linear 变成几十 m/s
```

优先检查：

```text
1. Hesai use_timestamp_type 是否为 1
2. Super-LIO gravity_norm 是否约为 9.4188
3. /imu/data 是否稳定在约 160-200 Hz
4. blind 是否过大
5. filter_rate 是否过大
```

当前稳定参数：

```yaml
blind: 0.5
filter_rate: 1.0
voxel_fliter_size: 0.3
gravity_norm: 9.4188
```

### 7.5 录制数据包

建议稳定后录制一包数据，便于后续回放和对比参数：

```bash
rosbag record /lidar_points /imu/data /lio/odom /lio/cloud_world /tf
```

## 8. 常用命令速查

启动 SLAM：

```bash
~/catkin_ws/shell/start_slam.sh
```

查看位姿：

```bash
rostopic echo /lio/odom
```

查看雷达频率：

```bash
rostopic hz /lidar_points
```

查看 IMU 频率：

```bash
rostopic hz /imu/data
```

查看 Super-LIO 输出：

```bash
rostopic list | grep lio
```

查看节点连接：

```bash
rosnode info /super_lio_node
```
