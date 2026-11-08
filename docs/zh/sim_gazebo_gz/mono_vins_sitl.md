# PX4 单目 VINS SITL 仿真教程

本文记录当前 `px4` WSL 系统中，如何用 PX4 Gazebo `gz_x500_mono_cam` 给 ROS2/OpenVINS 提供单目相机仿真输入。

这份教程面向刚开始接触 PX4 仿真和 VINS 的使用者，重点说明“为什么要这样配”和“每一步应该看到什么”。

## 0. 先理解几个名词

### SITL

SITL 是 Software In The Loop，意思是“软件在环仿真”。

在 SITL 中，PX4 飞控程序运行在电脑上，不运行在真实飞控板上。Gazebo 负责模拟飞机、传感器、物理环境。

因此：

- SITL 适合调试算法、话题、坐标系和控制流程。
- SITL 不会改写真实飞控板固件。
- 本教程使用的 `gz_x500_mono_cam` 是仿真机型，不是 `my_px4` 实机板卡。

### Gazebo world

Gazebo world 是仿真环境文件，描述地面、墙、灯光、纹理等场景。

本项目新增：

```text
Tools/simulation/gz/worlds/mono_vins.sdf
```

它只服务于单目 VINS 仿真，让相机看到足够多纹理。

### airframe

airframe 是 PX4 启动时使用的机型配置。

本项目使用：

```text
ROMFS/px4fmu_common/init.d-posix/airframes/4010_gz_x500_mono_cam
```

它是 Gazebo x500 单目相机仿真机型的启动配置。

### uXRCE-DDS

PX4 和 ROS2 之间通过 uXRCE-DDS 通信。

你可以把它理解成：

```text
PX4 uORB 消息 <-> uXRCE-DDS <-> ROS2 topic
```

ROS2 侧要先启动 `MicroXRCEAgent udp4 -p 8888`，PX4 才能和 ROS2 互相看到 `/fmu/*` 话题。

## 先理解整体链路

单目 VINS 的完整链路分成两个 WSL 系统：

```text
PX4 WSL:  PX4 SITL + Gazebo + x500_mono_cam
          |
          | Gazebo camera topic: /cam0/image_raw, /cam0/camera_info
          | uXRCE-DDS topic:    /fmu/out/*, /fmu/in/*
          v
ROS2 WSL: ros_gz_bridge + imu_bridge + image_mono_bridge
          |
          v
          OpenVINS
          |
          v
          vio_to_px4 -> /fmu/in/vehicle_visual_odometry
          |
          v
PX4 SITL: listener vehicle_visual_odometry
```

PX4 侧负责三件事：

1. 启动无人机仿真。
2. 让 Gazebo 发布单目相机图像。
3. 通过 uXRCE-DDS 接收 ROS2 回传的视觉里程计。

ROS2 侧负责图像转换、IMU 转换、OpenVINS 估计和 VIO 回传。

## 推荐终端布局

建议至少打开 4 个终端：

```text
终端 A：ros2 WSL，MicroXRCEAgent
终端 B：px4  WSL，PX4 SITL + Gazebo
终端 C：ros2 WSL，OpenVINS pipeline
终端 D：ros2 WSL，offboard 测试和 topic 检查
```

PX4 侧只需要终端 B。其余 ROS2 操作请参考 `ros2_uav/docs/ros2_basics_for_px4_vins_beginner.md` 和 `ros2_uav/docs/mono_vins_simulation_tutorial.md`。

## 为什么新增 `mono_vins` 世界

PX4 默认 Gazebo 空世界只有灰色地面。单目 VINS 依赖图像特征，如果图像里几乎没有角点，OpenVINS 会一直初始化失败。

常见失败日志：

```text
[init]: not enough feats to compute disp: 12,12 < 15
[init]: disparity is 0.000,0.000 (10.00 thresh)
```

因此新增了一个只用于仿真的世界：

```text
Tools/simulation/gz/worlds/mono_vins.sdf
Tools/simulation/gz/models/vins_texture/
```

它做了两件事：

- 前方放一面高对比特征墙，方便静止时确认相机能看到纹理。
- 起飞区域铺高对比纹理地面，飞行时让 OpenVINS 一直能跟踪到足够特征。

## 仿真安全边界

本次 PX4 侧只动仿真相关文件：

```text
ROMFS/px4fmu_common/init.d-posix/airframes/4010_gz_x500_mono_cam
Tools/simulation/gz/worlds/mono_vins.sdf
Tools/simulation/gz/models/vins_texture/
```

没有修改真实板卡烧录固件目录：

```text
boards/px4/mypx4_v1/
```

`4010_gz_x500_mono_cam` 是 SITL airframe，不是实机板卡配置。

## 第 1 步：启动 XRCE Agent

在 ROS2 WSL 系统中先启动 Agent：

```bash
MicroXRCEAgent udp4 -p 8888
```

如果你习惯用 tmux：

```bash
tmux new-session -s mono_agent 'MicroXRCEAgent udp4 -p 8888'
```

这个进程是 PX4 和 ROS2 互相看到 `/fmu/*` 话题的桥。

## 第 2 步：启动 PX4 单目相机仿真

在 PX4 WSL 系统中运行：

```bash
cd ~/PX4-Autopilot
HEADLESS=1 PX4_GZ_WORLD=mono_vins make px4_sitl gz_x500_mono_cam
```

参数说明：

- `PX4_GZ_WORLD=mono_vins`：使用给 VINS 准备的高纹理世界。
- `gz_x500_mono_cam`：使用带单目相机的 x500 机型。
- `HEADLESS=1`：不打开 Gazebo 图形界面，适合 WSL 后台联调。

成功时 PX4 终端应看到类似输出：

```text
Starting gazebo with world: .../worlds/mono_vins.sdf
world: mono_vins, model: x500_mono_cam_0
Ready for takeoff!
synchronized with time offset ...
```

如果你看到 `Ready for takeoff!`，说明 PX4 commander 认为当前仿真可以解锁。

如果没有看到，先不要急着跑 ROS2，先在 PX4 shell 输入：

```bash
commander check
```

它会告诉你当前卡在哪个健康检查。

## 第 3 步：确认 Gazebo 相机话题存在

在 ROS2 WSL 系统中，ROS2 pipeline 会负责桥接 Gazebo 话题。启动前可以先知道期望话题：

```text
/cam0/image_raw
/cam0/camera_info
/clock
```

OpenVINS 不直接吃 Gazebo 原始图像，而是由 ROS2 侧转换成：

```text
/cam0/image_mono
```

这样做是因为 OpenVINS 对单通道灰度图更稳定，且当前配置按 `640x480` 下采样后的相机内参设置。

## 第 3.5 步：确认确实加载了 `mono_vins` 世界

PX4 终端启动日志里必须出现：

```text
Starting gazebo with world: .../worlds/mono_vins.sdf
world: mono_vins, model: x500_mono_cam_0
```

如果看到的是 `default.sdf` 或其他 world，说明环境变量没有生效。

正确命令必须把 `PX4_GZ_WORLD=mono_vins` 放在 `make` 前面：

```bash
PX4_GZ_WORLD=mono_vins make px4_sitl gz_x500_mono_cam
```

如果是在同一个终端重复启动，建议先完全退出旧 PX4/Gazebo，再重新运行。

## 第 4 步：为什么 airframe 里设置 `NAV_DLL_ACT`

`4010_gz_x500_mono_cam` 增加了：

```sh
param set-default NAV_DLL_ACT 0
```

原因：当前 SITL 测试通常不连接 QGroundControl。如果保留默认 datalink failsafe 行为，PX4 可能因为“没有 GCS 连接”拒绝解锁。

这个设置只影响该 SITL airframe，目的是让无人机能在纯 ROS2/XRCE 联调时解锁测试。

## 第 5 步：PX4 侧验证视觉里程计是否收到

ROS2 侧 OpenVINS 初始化成功并通过 `vio_to_px4` 回传后，在 PX4 shell 输入：

```bash
listener vehicle_visual_odometry 1
```

成功时应看到类似字段：

```text
TOPIC: vehicle_visual_odometry
timestamp: ...
position: [...]
q: [...]
velocity: [...]
quality: 100
```

看到这条消息，说明 ROS2 的 `/fmu/in/vehicle_visual_odometry` 已经进入 PX4。

## 常见问题

### 1. OpenVINS 一直说特征不够

检查是否真的用了 `mono_vins` 世界：

```text
Starting gazebo with world: .../worlds/mono_vins.sdf
```

再检查 ROS2 侧 `/ov_msckf/trackhist`。如果画面里只有灰色地面，说明世界没有重启加载，或相机没有看到纹理地面。

### 2. PX4 无法解锁

先在 PX4 shell 里看：

```bash
commander check
```

本仿真 airframe 已默认设置 `NAV_DLL_ACT 0`，正常情况下不需要 QGC 也能进入 offboard 测试。

### 3. ROS2 能看到 `/fmu/out/*`，但 PX4 收不到 VIO

按顺序检查：

```bash
ros2 topic hz /ov_msckf/odomimu
ros2 topic hz /fmu/in/vehicle_visual_odometry
```

如果第一条没有，问题在 OpenVINS 初始化。

如果第一条有、第二条没有，问题在 `vio_to_px4`。

如果第二条有、PX4 listener 没有，问题在 XRCE Agent 或 PX4 uXRCE client。

## 已验证结果

本次联调中，完整测试通过：

- PX4 SITL 使用 `mono_vins` 世界启动成功。
- ROS2 OpenVINS 动态初始化成功。
- Offboard square 任务完成解锁、起飞、四个航点、降落。
- PX4 `listener vehicle_visual_odometry 1` 能看到视觉里程计输入。

关键日志：

```text
[init-d]: 6 states, 47 feats (47 valid)
[init]: successful initialization
```

## 日常清理建议

PX4 仓库很大，构建产物也多。不要随便删除 `build/`，否则下一次编译会变慢。

可以安全清理的源码缓存：

```bash
find ~/PX4-Autopilot \
  -path ~/PX4-Autopilot/build -prune -o \
  -type d -name __pycache__ -print -exec rm -rf {} +
```

不建议提交：

```text
build/
logs/
*.pyc
__pycache__/
*.tmp
*.bak
```

如果你要检查本次仿真改动是否混入实机固件，运行：

```bash
cd ~/PX4-Autopilot
git status --short
```

本教程相关改动应只出现在：

```text
ROMFS/px4fmu_common/init.d-posix/airframes/4010_gz_x500_mono_cam
Tools/simulation/gz
docs/zh/sim_gazebo_gz/
```

不应把 `boards/px4/mypx4_v1/` 下的真实板卡固件改动纳入仿真提交。
