# AGENT.md — PX4 双轨开发工作指南（硬件固件 + ROS/GZ 仿真）

> 2026-09-08 硬件轨入口更新：当前状态以 `/mnt/d/work/PX4飞控项目设计/00_项目管理/AI上下文/` 为准，
> 先读 CURRENT_STATE.md / PROJECT_STATE.json / LOG_INDEX.md，再读对应记录。板级静态事实见
> `boards/px4/mypx4_v1/AGENT_hardware.md`。历史仿真版本说明不代表最新硬件固件或阶段放行。

## 项目定位
你当前在同一个仓库内并行推进两个方向：

1. 硬件固件方向：`mypx4_v1` 实板开发（NuttX / 板级驱动 / 固件构建与烧录）
2. 仿真联调方向：PX4 SITL + Gazebo gz + ROS2（跨 WSL 协同）

本文件是总入口。执行具体任务时，请先明确你当前属于哪一条轨道，再按对应约束操作。

---

## 快速入口

- 硬件固件轨（实板）：`boards/px4/mypx4_v1/AGENT_hardware.md`
- 仿真轨（WSL-A）：本文件下方“仿真轨规范”章节

---

## 通用原则（两条轨道都适用）

- 先最小改动、后逐步扩展；避免一次改动跨越多个系统层级。
- 修改前先确认目标构建产物，修改后至少完成一次对应目标编译。
- 文档与代码冲突时，以代码为准；文档应随代码同步更新。
- 变更要保持“单轨闭环”：硬件问题先在硬件轨闭环，仿真问题先在仿真轨闭环。

---

## 硬件轨规范（摘要）

目标板：`mypx4_v1`

核心目录：
- `boards/px4/mypx4_v1/`
- `platforms/nuttx/NuttX/nuttx/*mypx4*config`

常用命令：
```bash
make px4_mypx4_v1_default
make px4_mypx4_v1_bootloader
```

详细规范请看：`boards/px4/mypx4_v1/AGENT_hardware.md`

---

## 仿真轨规范（WSL-A）

### 环境基本信息

| 项目 | 值 |
|---|---|
| PX4 版本 | v1.16 |
| 仿真器 | Gazebo gz（Harmonic） |
| WSL 发行版 | Ubuntu 22.04 |
| DDS Agent | Micro-XRCE-DDS-Agent |
| ROS2 所在环境 | 独立 WSL-B |
| 必须使用系统用户账号 | 用户名 he；密码不写入仓库 |


### 硬性约束

- 禁止使用 Gazebo Classic 资产、插件或启动方式。
- 禁止使用 `iris_stereo_camera` 或其他 Classic 时代模型作为基础。
- `x500_vision` 不是相机模板，禁止以其为基础加相机。
- 新模型参照 `Tools/simulation/gz/models/x500` 与 `x500_mono_cam` 风格。
- 机型配置放在 `ROMFS/px4fmu_common/init.d-posix/airframes/`，并在对应 `CMakeLists.txt` 注册。

### 仿真关键目录

```text
Tools/simulation/gz/models/            # gz 机体模型
Tools/simulation/gz/worlds/            # gz 世界
ROMFS/px4fmu_common/init.d-posix/airframes/  # SITL 机型脚本
```

### 常用命令（仿真）

```bash
# 编译并运行 SITL（示例机型）
make px4_sitl gz_x500_stereo_cam

# 仅启动仿真器
gz sim Tools/simulation/gz/worlds/default.sdf

# 启动 DDS Agent
MicroXRCEAgent udp4 -p 8888
```

### gz -> ROS2 桥接示例

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /camera_left@sensor_msgs/msg/Image[gz.msgs.Image \
  /camera_right@sensor_msgs/msg/Image[gz.msgs.Image \
  /camera_left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
  /camera_right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo
```

### 单目 OpenVINS 联调（PX4 侧）

仅针对单目测试，先使用最小改动策略：

1. 机型建议使用 `gz_x500_mono_cam`（airframe: `4010_gz_x500_mono_cam`）。
2. PX4 侧已在该 airframe 中设置默认参数：
   - `EKF2_EV_CTRL=11`（水平位置+垂直位置+航向，不启用 EV 速度融合）
   - `EKF2_EV_DELAY=0`
   - `EKF2_HGT_REF=0`（以气压计作为高度参考，便于单目初测稳定）
   - `EKF2_EV_POS_X/Y/Z=0`
3. ROS2 到 PX4 输入话题：`/fmu/in/vehicle_visual_odometry`（`px4_msgs/msg/VehicleOdometry`）。
4. PX4 到 ROS2 输出话题：`/fmu/out/sensor_combined`。
5. 若使用仿真时间，ROS2 节点需 `use_sim_time=true`，并确保 `/clock` 连续发布。

PX4 侧快速检查：

```bash
# 1) 看 VIO 输入是否持续更新
listener vehicle_visual_odometry 1

# 2) 看 EV 相关参数（不同版本字段可能有差异）
param show EKF2_*EV*

# 3) 看 EKF 是否真的在融合 EV 位置/航向
listener estimator_aid_src_ev_pos 1
listener estimator_aid_src_ev_yaw 1
```

若出现创新量突增、航向跳变或融合拒绝，优先排查：
- ROS2 发送给 PX4 的坐标与姿态约定（NED / FRD）是否正确
- 四元数顺序与方向是否正确
- 时间戳偏差是否过大（必要时调 `EKF2_EV_DELAY`）
- VIO 频率是否在 20 到 50 Hz


---

## 双轨协同边界

- 硬件轨不处理：ROS2 包编译、OpenVINS 参数调优、仿真世界编辑。
- 仿真轨不处理：实板引脚定义、NuttX defconfig、bootloader 兼容性。
- 若同一需求横跨双轨（例如“先仿真验证再上板”），分两阶段提交：
  1. 仿真侧确认算法/话题与时序
  2. 硬件侧做驱动与接口对齐

---

## 让 Agent 做什么

可以让 Agent：
- 在仿真轨修改 gz 模型、airframe、桥接命令
- 在硬件轨修改 `mypx4_v1` 板级配置、启动脚本与构建配置
- 进行“按轨道”的问题定位与最小修复

不建议让 Agent：
- 在一个变更中同时大改硬件轨与仿真轨
- 在缺少约束信息时直接重写底层板级初始化

---

## 当前建议的执行方式

1. 你下发任务时先注明：`[硬件轨]` 或 `[仿真轨]`
2. 若跨轨任务，注明先后顺序：例如“先仿真后上板”
3. 每次改动结束后，至少保留一条可复现实验/构建命令

这样可以显著减少上下文混淆，提升改动可回溯性。

---

## 夜间复盘入口

- 2026-03-29 复盘与待办：TONIGHT_2026-03-29_复盘与待办.md
