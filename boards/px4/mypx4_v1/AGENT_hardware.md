# mypx4_v1 硬件固件工作入口

更新时间：2026-09-08。这里描述源码事实，不代表实物验收或飞行放行。

## 先读当前项目状态

项目目录：`D:/work/PX4飞控项目设计`，WSL 对应 `/mnt/d/work/PX4飞控项目设计`。
先依次读取其中 `00_项目管理/AI上下文/CURRENT_STATE.md`、`PROJECT_STATE.json`、
`LOG_INDEX.md`，再读取对应项目记录。遵守项目根 `AGENTS.md` 的证据规则。
`firmware.latest_build` 与 `firmware.flight_evidence_build` 必须分别引用。
项目当前阶段门是 RT-01 READY、整机 HOLD；本次维护没有实机验证。

## 2026-10-06 v1.0.8 / revision19 当前合同

- 4001及退役ID的目标启用IMU_GYRO_DNF_EN=2（FFT位）和FFT_EN=1；其他滤波/PID参数不改。
- 0..16完整黄金同步直接到19；17仅TELEM3波特率和上述两项；18仅上述两项。
- 19重复启动保留人工后续编辑（包括关闭陷波），>19不降级，其他机型不强制开启。
- 维持保存/重载验证后再推进标志、失败停止启动、GUID校准和计数保护。
- 下文18/17为历史合同；本次目标为未覆盖v4表加用户授权的陷波差异，不是现场读回。

## 2026-10-05 v1.0.6 / revision18 当前固件合同

- UART5就是TELEM3/ttyS4；NuttX初始波特率和SER_TEL3_BAUD目标均57600。
- 0..16按下列v3合同加串口覆盖直接同步到18；已部署17仅一次性写57600并保存/重载验证，不重放957项同步；18重复启动保留人工改值，>18不降级。
- 黄金v3原工作簿不修改；本次用户授权串口覆盖记录在golden_manifest.json。

## 2026-09-29 v1.0.5 / revision17 基础合同（历史）

- 4001和退役4081/4082/4014直接进入黄金v3事务，不再执行旧0..16迁移链。
- 961项原始位值与Excel核对；957项硬同步，MYPX4_CFG_VER由事务推进17；三个飞行累计量保留。
- 原机GUID才使用67项CAL_/SENS_BOARD_值；其他板保留自身校准。其他机型不硬同步。
- param mypx4-golden apply在board_defaults中、机型选择/传感器/commander前运行；逐项原始位值写入、保存、重新加载核验，再提交标志。两次存储阶段不是硬件原子事务。
- 每次启动恢复同一黄金运行默认值，airframe之后再次恢复；保存的人工值优先。临时异值默认仅在同步事务内使用；失败停rcS且关闭autosave，需重启排查。
- command收到vehicle_status后拒绝运行。repeat marker17不再硬写；future marker>17不降级。
- 下文revision15内容保留为历史说明；本版本以test_startup.py和verify_entry_release.py的新合同验收。

## 源码事实与入口

- 仓库 `/home/he/PX4-Autopilot`，通常为 DIY 分支；每次先运行 `git status --short --branch`。
- 目标 `px4_mypx4_v1_default`；板型 ID **53**、最大 image **1,966,080 B**；以 firmware.prototype 为准。
- STM32H753II，无 PX4IO。八个常规 PWM 加 PE11 CAP 复用通道；CAP 是元数据分类，可复用输出。
- TIM5=MAIN1..4、TIM4=MAIN5..6、TIM12=MAIN7..8、TIM1=CAP；组内协议/频率共享。
- 默认包含 MC、Ackermann、Differential；Mecanum 另见 rover.px4board；不据此声称车/船已实测。
- 默认未启用 UAVCAN/Cyphal/DShot；IWDG 未启用；协议能力看 build/.../boardconfig 和 NuttX/nuttx/.config。
- BMI088 accel/gyro on SPI3；ICM42605 on SPI1。固定旋转 4/4/4；BMI088 优先级100、ICM42605为50。
- QMC5883P I2C4/0x2c rotation14、BMP388 I2C4/0x76；磁航向仍待实物四向验证。
- FRAM SPI5：参数32KiB、任务96KiB；SDMMC2日志。ADC12默认分压5.7，电流通道禁用；电压尚待标定。
- UART映射：GPS1 ttyS0、GPS2 ttyS7、TEL1 ttyS6、TEL2 ttyS3、TEL3 ttyS4、TEL4 ttyS5、EXT2 ttyS1。
- 4001入门新装默认：TEL1/TEL2/TEL4=921600，TEL3=57600；MAV_0/1/2_RATE=0；TEL3 Normal；TEL4 DDS domain42。E34对端57600仍须实物核对。
- 独立 RC 已有 CRSF/SBUS/DSM/GHST 分驱动，使用 RC_CRSF_PRT_CFG / RC_SBUS_PRT_CFG 等指定空闲端口；
  不再推荐旧 RC_PORT_CONFIG。先核对生成 rc.serial，避免占用已有通信端口。

## 2026-11-08 v1.0.18 差速机架 GNSS
`50000` 使用和 `4001` 同一套 UM982 专用接入、严格定位主备和副路状态补显。
从黄金表仅取 12 项 GNSS 运行默认值；已有保存值优先，不运行四旋翼迁移，不推进 CFG26。
输出映射、反转、控制参数及天线安装偏角按车/船实物核对；无独立船型预设。

## revision 15 的边界

- 默认选择上游4001 Generic Quadcopter；旧4081/4082/4014在本板启动前映射到4001。4081脚本移除，不再重放其历史PID和输出迁移。
- 1.0.1 实机读回证明 revision10 标志已保存，但低速参数仍是旧值；根因链指向先 `set-default` 再 `set` 导致持久写入被跳过。首版1.0.2 revision11 已修复低速值，但 IMU_GYRO_FFT_EN 和 SENS_IMU_MODE 在重启后仍退回0/1；revision12 对这两项先切换后写目标。第二版1.0.2在再次重启后丢失四项Quad-X几何值。revision13实机首次读回仍有CA_ROTOR0_PX=0、MAV_TYPE=0。revision14在4001机架脚本执行后，为全部14项Quad-X几何/机型参数设置不同的运行时默认值、强制写目标、核验并保存后推进标志；每次后续启动维持这些不同默认值以防参数导出时被省略。覆盖已有 revision0..13 的 4001 板载配置。实机验证结果以项目记录为准。
- revision15针对实机revision13仍残留3/1.5m/s上下和10m/s Position手动速度，再次一次性强制写入完整慢速包络，连同上游4001 Quad-X几何经读回、保存后推进标志；revision15后再次启动保留人工改值。白名单包含4°手动目标倾角、上下/Position手动0.3m/s、Slow限值、15°/s偏航、Joystick only、TELEM3 115200/Normal、诊断日志849、MAV_TYPE=2、双IMU模式及按设备ID识别的优先级。ALTCTL水平速度没有0.3m/s硬上限。
- 同一次迁移把四电机混控重建为上游4001 Quad-X几何与默认电机系数/轴向/零倾转，不采用自定义电机特性；修复9月28日Rotor0力臂为0的读回。revision15及未来版本重复启动保留操作者后续改值；必须在实机烧写后与连续再次启动后分别读回验证。
- 不强制迁移PID、电机端点、CAL优先级、EKF拖曳参数、failsafe动作或可选光流端口；异常实机保存值必须在无桨飞行前检查中逐项复核。
- 板级传感器设备ID优先级纠正与EKF2_MULTI_IMU 3→2保护仍在；不清零MYPX4_CFG_VER，不默认整机参数重置。降级前备份参数，旧固件行为可能不同。

## 验证与交付

1. `python3 boards/px4/mypx4_v1/tests/test_startup.py /home/he/PX4-Autopilot`
2. 审核 `git diff --check`、精确暂存、提交；从 clean commit 构建。
3. `make px4_mypx4_v1_default`；必要时只对该目标运行 `ninja -C build/px4_mypx4_v1_default clean`。
4. `python3 boards/px4/mypx4_v1/tests/test_startup.py /home/he/PX4-Autopilot --generated`
5. `python3 boards/px4/mypx4_v1/tests/verify_entry_release.py /home/he/PX4-Autopilot <实际HEAD>`
6. Windows项目 `02_固件与调试/scripts/build_and_archive.ps1 -SkipBuild` 验证包内Git和BIN，再归档。
7. 产物复制到桌面 `PX4飞控固件` 的新独立目录，逐文件核对 SHA256；保留回滚版本。

tests 是主机参数替身测试；verify_entry_release 校验4001生成脚本、包内身份与镜像。旧verify_release保留为revision7历史合同。
未来改变控制/硬件配置时，应显式更新验证合同和基线，不能为了通过检查删去断言。
构建成功不证明硬件时序、FRAM持久化、电调兼容、链路延迟或稳定飞行。
ULog 取证默认断电拔卡；MAVFTP 必须先 Disarmed、logger stop，结束 logger start。

## 后续优化顺序

先完成实物基线/电源标定与控制链时序，再验证升级读回、长时日志、独立RC与failsafe。
PWM分组、电源宏、IWDG、CAN/DShot、模块裁剪和车/船专用配置需各自独立评审，避免混入维护发布。

1.0.8实机复核修正：有限迁移在修改runtime defaults前捕获所有适用黄金条目的当前有效原始位值；目标条目用授权值，其他条目用捕获值。所有条目使用与待保存值不同的临时默认值、显式API写入并逐项save/reload核验，不能仅核验授权delta而让其他保存值被BSON省略。19重复启动不写用户值；该修正不扩展黄金目标或改变既有校准/累计例外。

## 2026-10-07 HE_PX4_v1.0.9 当前合同

黄金v5，CFG20；UART7=TELEM1=/dev/ttyS6，115200、Normal、RATE1200、FLOW_CTRL0、FORWARD0；MPC_ALT_MODE1。PID全保持。19只一次迁移上述波特率/流控/转发/高度四项；18另带FFT，17另带TELEM3波特率，0..16全表。20重复启动不改已保存值，>20不降级。全量捕获、临时不同默认值强制序列化和两次保存/重新加载验证不得删除；原UID校准、运行计数及其他机型保护保持。用户无桨、仅USB授权烧写不放行飞行；未来飞行证据独立引用。

## 2026-10-15 HE_PX4_v1.0.10 当前离线候选合同

不新增参数；本板4001及退役4081/4082/4014的FFT陷波固定20Hz，保留中心跟踪、设备/采样率/时间有效性与启用保护。其他板及其他机型保留原宽度公式。状态输出显示实际设备、启用、中心和带宽。
黄金v5原件保持；授权覆盖仅横滚/俯仰RATE_K=0.90和CFG21；独立P/I/D、D低通20Hz及其他目标不改。20只迁移两个K；19带先前UART7/高度差异再加K；18另带FFT；17另带TELEM3；0..16完整目标。21重复启动保留人工保存值，>21不降级，其他机型不强制；GUID校准、计数及全量捕获/保存/重载校验保持。
源码、提交、构建与主机验证为离线范围，未刷写。FFT20与K0.90同时编入此候选，不是单变量实飞证据；闭环/碳塑共用参数未验收，整机HOLD不变。

## 2026-10-15 HE_PX4_v1.0.11 当前离线候选合同

单轴首轮候选：MC_PITCHRATE_D=0.0033（原0.003）；MC_ROLLRATE_D=0.003、两轴P/I/K、姿态P、FFT固定20Hz、gyro40/D20Hz、手柄平滑0、手动倾角8°及起飞ramp3秒保持。无新增参数，未改变控制算法。
CFG22：21只一次迁移俯仰D；20为两个K加俯仰D；19加之前UART7/高度，18再加FFT，17再加TELEM3，0..16完整授权目标。22重复启动保留后续人工编辑，>22不降级，其他机型不强制；原GUID校准、计数、全量捕获/不同临时默认值/两次保存重载核验及失败停止启动保持。
仅离线构建与双份归档，未刷写/飞行验证。D+10%为待验证阻尼候选，不宣称已降低过冲或满足鲁棒性；整机HOLD及既有阶段门不变。横滚D、外环、输入平滑和限角是后续独立轮次，未混入本版本。

## 2026-10-16 HE_PX4_v1.0.12 UART8 UM982 离线合同

物理GPS2=UART8=/dev/ttyS7，作为主GPS逻辑实例：GPS_1_CONFIG202、GPS_1_PROTOCOL6、GPS_2_CONFIG0、SER_GPS2_BAUD230400；UART8初始波特率同为230400。不把仅有的UM982放在依赖主GPS首次发布的副实例。支持定位/UNIAGRICA三维速度/UNIHEADINGA双天线航向，沿用已有驱动，不改上游驱动。
新装EKF2_GPS_CTRL15；已部署17..22只OR8保留原融合位。GPS_YAW_OFFSET0由用户确认暂保留，实物天线方向未定；0仅适用于主前副后，安装后另验收。不关罗盘，不添加4G或差分服务，不宣称RTK Fixed。
CFG23：22只接收六项UM982配置；17..21叠加各自未完成的历史差异；0..16全目标（原957加已有SER_GPS2_BAUD，共958）。23重复启动保留人工修改，>23不降级，其他机型不强制，校准/GUID/计数及事务保存重载/失败停止保持。1.0.11俯仰D候选与既有FFT/PID目标继承，22升级不会重写PID。仅离线交付，无现场操作，HOLD不变。

## 2026-10-16 HE_PX4_v1.0.13 双GPS主备与航向回退合同

UM982物理GPS2/UART8作为逻辑0，NMEA6/230400；原GPS物理GPS1作为逻辑1，u-blox1/自动波特率。两路持续运行，SENS_GPS_MASK0/PRIME0严格主备。仅本板4001/退役ID及该端口配置启用定制选择；其他板/其他机型保留原逻辑。主实例在启动副实例前预留空uORB topic0，不伪造测量，避免冷启动UM982缺席阻塞备用。
接收数据2秒失联、无效fix/坐标/速度直接不可用；静态质量连续1秒不合格切换，回切主机需连续5秒合格。质量读取当前EKF2_GPS_CHECK可直接比较的静态项和REQ阈值；完整静止漂移、创新/融合检查仍由EKF执行，不宣称独立复现全EKF检查。无RTK不自动判无效。仅航向缺失不弃用正常UM982位置，备用GPS不提供双天线航向。保留原生GNSS yaw与健康罗盘自动回退，MAG_TYPE1只校正航向，不强迫使用受干扰罗盘。
CFG24：23只迁移十项双GPS/融合设置，保留已有安装偏角及全部其他参数；22及更老版本另继承当时未完成差异和原暂定0°。新装15、部署GPS_CTRL旧值OR15；24重复启动保留人工修改，>24不降级。目标959项（新增已有GPS_2_PROTOCOL），黄金v5不改。真实切换、完整参数读回及断电保持待现场验收，未刷写，HOLD不变。

## 2026-10-16 HE_PX4_v1.0.14 塑料桨PID恢复合同

用户停止1047全碳桨适配，计划改回塑料桨。基于1.0.13保留双GPS/航向回退及FFT固定20Hz；仅将所列两轴角速度K/P/I/D恢复1.0.9目标：K1.00/P0.15/I0.20/D0.003（八个参数）。不是完整1.0.9代码/滤波回滚，不宣称实物换桨已完成。
CFG25：24只一次恢复八个增益；17..23恢复八个增益并叠加当时尚未完成的历史差异，23保留安装偏角，24保留全部GNSS现场值（包括EKF2_GPS_CTRL原始位）。0..16完整959条目标。25重复启动保留人工修改，>25不降级，其他机型不强制。原GUID校准、计数、全量捕获/不同默认值强制保存/两次重载及失败停止启动保持。
黄金v5原表不改，参数定义不新增。离线构建不证明塑料桨稳定性、FRAM断电保持或GNSS现场性能，HOLD不变，无连接/刷写/执行器动作。Git只提交，固件项目与桌面独立版本目录备份，无Git历史包。

## 2026-10-18 v1.0.15 / CFG26
UM982主UART8兼容修复：固定波特率、启动配置AGRICA/UNIHEADINGA及NMEA五赫兹，无解CRC有效回包发布fix1/速度无效，断流保持失联；不放宽融合质量门。航向状态/数值/基线校验、初始化、新鲜度回退与统计修复。部署25仅推进标志，全量参数保留；旧版本继承先前适用迁移，其他机型和已保存偏角保持。黄金v6同步959目标和CFG26，标清离线目标。Git仅提交，固件双份备份，无现场刷写。
