# 蘑菇采摘平台当前进度

> 更新时间：2026-08-03
> 证据范围：当前源码/配置、Git 状态、151 项 Host 离线测试、已验收 STM32 tag 及仓库内
> 历史构建/硬件记录。编译和离线测试不视为硬件验收。上层运动专项详见
> [UPPER_MOTION_CONTROL_HANDOFF.md](UPPER_MOTION_CONTROL_HANDOFF.md)。

## 1. 技术结论

项目处于“底层执行器组件初步可用、Host 上层边界刚形成、系统协调待开发”阶段。

- 最成熟的是已锁定为 `stm32-motion-v0.1.0` 的 STM32 Slide/Z/吸盘和 machine protocol；
- MG4010E 已具备 CAN、协议、单关节、肩肘配置和最小 Planar 2R 命令桥；
- Feetech 已确认为 `SM-45BL-C001`、RS-485、自动方向 USB 转换板、115200 baud 和 ID 1；
  已完成 ping/raw read、正方向和零点确认，并把当前调试参数固化为项目配置；
- 最大缺口是 arrival wait、motion timeout、统一停止/故障传播、坐标变换和采摘状态机；
- 下一里程碑应是固化本次基线并完成 Feetech 机械标定，不是直接做整机采摘。

## 2. 总体进度矩阵

| 子系统 | 当前状态 | 已实现能力 | 主要缺口 | 验证级别 |
| --- | --- | --- | --- | --- |
| STM32 Slide | Implemented/Compiles/部分实机记录 | STEP/DIR、相对/绝对运动、停止/禁用、StallGuard homing | 最终全行程、异常场景复验 | 历史 bench/mechanical evidence；本轮未重测 |
| STM32 Z | Implemented/Compiles/部分实机记录 | STEP/DIR、位置运动、停止/禁用、开关 homing | 最终行程、普通运动 endstop 策略 | 历史 bench/mechanical evidence；本轮未重测 |
| STM32 machine protocol | Implemented/Compiles | v1 sequence、同步响应、异步事件、稳定错误码 | 高日志/长时间通信与正式系统联调 | 固件已验收；Host offline tested |
| Vacuum | Implemented/Compiles | 吸附/释放状态机、互锁、查询、停止 | 无真空反馈、参数待机械验证 | 固件已验收；完整吸取未验证 |
| TMC5160 diagnostics/homing | Implemented/Compiles | SPI 配置、DIAG、Slide/Z homing | 最终参数和全异常矩阵 | 部分 bench/mechanical evidence |
| MG4010E CAN/protocol | Implemented/Offline tested | transport、`0x94/92/9A/9C/A4/81` | 长时间双电机实机稳定性 | Offline tested |
| MG4010E single joint | Implemented/Offline tested | 绝对位置解释、软限位、命令、软件停止 | 到位、超时、运行中故障协调 | Offline tested；部分历史实测说明 |
| Shoulder/elbow calibration | Implemented/Not fully verified | 零点、方向、限位、速度配置 | 独立原始校准记录和复测 | Code/test evidence，校准证据不足 |
| Planar 2R | Implemented/Offline tested | FK/IK、双解、不可达/奇异、按关节限位筛解 | 实际连杆长度、碰撞和连续性 | Mathematical offline tested |
| Feetech rotation | Implemented/部分 Bench/Mechanical tested | C001 profile、项目安装配置、ping/raw read、位置/反馈、六字节位置命令、torque disable、dry-run | 最终限位/负载速度、完整反馈、重复性验证 | ID 1 ping/raw read；方向与零点受控确认 |
| Multi-joint coordination | 初步桥接 | 肩肘背靠背下发、失败尽力停止 | 到位/超时/统一故障传播/严格协调 | Offline tested only |
| Coordinate transforms | Planned | 架构文档 | camera/base/slide/tool frames 均未落地 | Not verified |
| Harvesting task | Planned | 架构文档 | 视觉、接近、下探、吸附确认、搬运、释放、恢复 | Not verified |

## 3. 仓库与版本状态

### 根仓库

- path：项目根目录；branch：`main`；upstream：`origin/main`；
- HEAD：`673a373`，`feat(host): integrate upper motion control backends`；
- 既有上层运动整合已形成 commit；当前工作树 dirty，包含本次 `SM45BL-C001` profile、
  CLI、tests 和文档更新，以及既存的 `docs/hardware`、`docs/interfaces` 和旧进度文件调整；
- `feetech_arm.zip` 保持原文件且被 Git 忽略；
- 风险：本次确认的 C001 配置与 151 tests 尚未绑定到新的 root commit。

### STM32 submodule

- path：`firmware/stm32_motion_controller`；branch：`main`；
- HEAD：`cb075675e32cd5c5e9e5d1d43ddaa5e539fdc8d4`；
- tag：`stm32-motion-v0.1.0`；remote：GitHub SSH remote；
- root 通过 `.gitmodules` 和 gitlink 正式跟踪，当前锁定验收 commit；
- 工作树 clean，本轮未改变 submodule 或 gitlink。

未提交文件的逐项归属见上层专项交接第 2 节和最终 `git status --short`。

## 4. 系统架构

```text
STM32 Slide/Z/Vacuum firmware
        ↑ ASCII serial protocol v1
STM32MotionClient ───────┐
                        │
MG4010 CanRotaryJoint ───┼─> future MultiAxisCoordinator
        ↑                │              ↑
Planar 2R IK/FK          │      coordinate transforms
                        │              ↑
FeetechRotationAxis ─────┘      harvesting task state machine
```

- reusable Host libraries：`host/drivers/`、`host/robot/`、`host/kinematics/`；
- platform firmware：`firmware/stm32_motion_controller`；
- algorithm-only：`host/kinematics/planar_2r.py`；
- 最小 actuator bridge：`host/robot/planar_arm.py`；
- capability 声明：`host/motion/capabilities.py`；
- 未完成集成层：正式 coordinator、坐标系、`host/tasks/` 采摘流程。

## 5. STM32 固件进度

### 5.1 Slide and Z Motion

两轴均有 µm 机器单位、相对/绝对运动、状态、停止、禁用和 enable。当前位置为 STEP
脉冲估计，不是编码器闭环。当前源码软限位为 Z `0..60800 step`、Slide
`0..35555 step`，注释明确仍是保守临时值。

### 5.2 TMC5160 Configuration and Diagnostics

两轴 TMC5160、SPI、DIAG 和电流配置已进入验收 commit。现有证据支持“代码实现、可编译、
有部分台架记录”，不支持所有驱动故障和全机械行程均完成验收。

### 5.3 Homing and Position Validity

Z 使用低有效感应开关，Slide 使用 StallGuard sensorless homing。成功 homing 后建立
machine zero 与 valid；上电、禁用和相关故障后位置可能失效。

### 5.4 UART Logging and Machine Protocol

协议为 `@seq command` / `=seq response` / `!seq event`。支持 `QS MR MA HM ST DI EN SA
CF QH SQ SU SR SX VR`。根 Host 已新增正式 `STM32MotionClient`，并以测试锁定固件 header
和 README 中的 protocol contract。

### 5.5 Vacuum Pump and Release Valve

固件提供非阻塞吸附/释放、pump/valve 互锁和安全关闭。无真空传感器，故 `DONE V 1` 不等于
吸附成功。

### 5.6 Fault, Limit, and Emergency Handling

协议有 fault/error、单轴 stop/disable 和 `SA` 全轴禁用。软件停止不替代硬件急停；吸盘的
紧急释放策略和跨后端统一停止尚未形成系统政策。

## 6. MG4010E 关节控制进度

- transport：`CanMotorBus` 支持 `gs_usb` 与 SocketCAN、共享锁、timeout/retry、旧帧清理、
  ID 与 frame 验证；
- protocol/driver：当前只开放 `0x94`、`0x92`、`0x9A`、`0x9C`、`0xA4`、`0x81`；
- joint：`CanRotaryJoint` 使用输出轴 rad，处理 36:1、zero、direction、soft limits、最大速度；
- position source：`0x94` 是跨重启单圈机械解释来源，`0x92` 只服务当前上电周期 A4 目标；
- shoulder：ID 1、zero 100°、direction +1、-60°..+70°；
- elbow：ID 2、zero 158°、direction -1、-152°..+152°；
- 两者 max speed 50°/s；
- 缺失：自动 enable/clear fault/home、arrival、stable window、motion timeout 和轨迹协调。

## 7. 标定与运动学进度

肩肘配置与离线测试一致，但 `docs/calibration/` 没有原始测量记录。Planar 2R 使用 x 向前、
y 向左、z 向上的右手系，提供 forward kinematics（正运动学，FK）与 inverse kinematics
（逆运动学，IK）、两支解、reachability 和 singularity 处理。当前工作树的
`Planar2RArmController` 已按肩肘软限位筛解并连接真实关节命令 API，但只做背靠背下发，
不等待到位。实际 `L1/L2` 尚未项目化，不得从示例值猜测。

## 8. 系统协调与采摘任务

六类后端已可在 fake 下装配并查询 capability。Slide/Z/Vacuum 走 STM32，Shoulder/Elbow
走 MG4010E，Rotation 走 Feetech。仍缺 common arrival contract、deadline、统一 stop、fault
propagation、camera-to-robot transform、vacuum confirmation，以及接近/下探/搬运/释放/
恢复状态机。目录或架构说明不视为实现。

## 9. 验证结果

### 9.1 Firmware Builds

历史记录：在 STM32 submodule 执行 Debug/Release CMake build 均成功；验收 commit/tag 已
固定。本轮没有重跑固件 build，也没有修改固件。

### 9.2 Host Unit Tests

工作目录 `host/`，命令：

```bash
.venv/bin/python -m unittest discover -s tests -q
```

exit 0，`Ran 151 tests`，全部通过，0 failures，0 skips；硬件未参与。

### 9.3 Mathematical Tests

Planar 2R、joint conversion、Feetech raw/rad conversion 均包含在上述离线测试。

### 9.4 Electrical Bench Tests

2026-08-03 使用 `/dev/cu.usbmodem5B790798091`、115200 baud 对
`SM-45BL-C001` ID 1 执行只读台架测试：ping 成功，首次和随后三次 `0x38` 位置读取均为
`position_raw=2047`（`179.912109375°`），程序退出后 `lsof` 未发现端口占用。未读取完整
feedback，未执行 torque 或位置写入。STM32 和 MG4010 的历史说明不能替代完整台架证据。

### 9.5 Mechanical Tests

Feetech 已完成受控小角度方向确认，用户确认 `direction_sign=+1`、逻辑正方向为 `+X`，
机械零点最终微调为 `zero_raw=2130`。当前 `±45°` 和 500 raw 是调试配置，尚无最终机械
限位、负载速度或重复回零验收记录。

### 9.6 Integrated System Tests

只有 fake backend smoke test；无整机采摘测试。

## 10. 资源与性能

- STM32 protocol frame 最大 96 bytes；
- 固件协议与 debug log 共用 USART1，正式运行需控制日志量；
- Feetech 与 STM32 Python transport 均有有限 timeout、无无限重试；
- MG4010 transport 有配置化 timeout/retry，并以共享锁串行化同一 CAN 总线事务；
- 本轮 Host 测试 151 项约 0.20 s；
- 本轮没有重新采集 STM32 FLASH/RAM 或真实串口/CAN 吞吐量。

## 11. 安全默认行为

- STM32 上电不运动、不自动 home，轴默认 disabled；pump/valve 默认 off；
- STM32 Host client 构造不打开串口，也不发送命令；
- MG4010 driver/joint 不自动 enable、clear fault、home 或发送位置；
- MG4010 CLI 不加显式 motion flag 时为预览，`0x81` 仅软件停止；
- Feetech import/构造不打开串口，CLI 默认 dry-run；真实访问要求 `--execute`、显式 port/
  baud，enable torque 还需额外显式参数；
- 通信失败抛出异常；MG4010 A4 结果未知时尽力发 `0x81`；Feetech 不无限重试；
- STM32 `DI/SA` 和相关异常会使开环位置 invalid；突然断电后必须重新确认位置/归零；
- 当前未发现 Host 自动真实运动宏；STM32 boot-test 与 real-motion 宏默认值仍以验收 commit
  源码为准，本轮没有改动。

## 12. 已知问题和风险

### Confirmed issues

- 根工作树含本次尚未提交的 C001 profile、CLI、tests 和文档更新；
- Feetech zip 没有 license/provenance，原始代码存在错误的 4-byte position write；
- Feetech 型号/总线/baud、当前 port、servo ID 1、方向和零点已确认；`±45°` 与 500 raw
  仅为当前调试约束，不能视为最终机械验收值；
- `docs/calibration/` 缺肩肘独立记录；
- STM32 App README 的旧 `0..200 step` 描述与当前源码软限位不一致；
- 无 vacuum feedback；无跨后端 arrival/timeout/coordinator/task state machine。

### Risks

- C001 feedback、write status 和 USB 转换板行为仍需实机只读验证；
- 临时 Slide/Z soft limits 和 sensorless homing 参数可能不覆盖完整机构；
- 肩肘背靠背命令不能保证同时到位；
- 软件 stop 不是硬件急停，故障传播策略未统一；
- 算法离线正确不等于真实连杆、碰撞和负载安全。

## 13. 开放决策

- Feetech 最终机械 limits、负载 max speed 与 write status 行为；设备路径继续运行时注入；
- Slide/Z 最终 travel 与 homing 参数；
- vacuum PWM/timing、传感器和 emergency release 策略；
- shoulder/elbow 实际 link lengths、安装偏移和校准复验；
- IK branch/连续性/碰撞筛选；
- coordinator 的 arrival window、timeout、stop/fault policy；
- camera/base/slide/tool 坐标系；
- 未提交修改的提交拆分及 `feetech_arm.zip` 是否归档到外部制品存储。

## 14. 下一阶段建议

| Priority | Goal | 影响文件 | 证据/硬件 | 验收标准 | 安全要求 |
| --- | --- | --- | --- | --- | --- |
| P0 | 固化 root 可复现基线 | 当前 dirty Host/docs | 无硬件 | clean checkout 安装后 151 tests pass | 分组提交，不覆盖用户修改 |
| P1 | Feetech 只读识别 | driver、calibration docs | 铭牌、接线、官方手册、适配器 | ping/read 重复成功，错误/断线明确失败 | 不 enable torque |
| P1 | Feetech 低速标定 | config、calibration docs | 空载/安全机械区域 | zero/direction/limits 可复测 | 小角度、低速、可断电 |
| P2 | arrival/timeout/fault | `host/motion/`、tests | fake 后再台架 | 所有后端 deadline 和失败 stop 有测试 | 软件 stop 不替代急停 |
| P2 | 低速多轴协调 | coordinator、tests | 分阶段台架 | 点到点到位或安全失败 | 单后端逐步接入 |
| P3 | 完整采摘 workflow | transforms、tasks | 视觉/真空/整机 | 吸附确认、搬运、释放、恢复闭环 | 故障注入与人工急停 |

## 15. 交接信息

- overall stage：组件实现 + 离线集成；
- active task：复验 Feetech 最终机械限位、负载安全速度和 feedback/status 行为；
- read first：本文件、`UPPER_MOTION_CONTROL_HANDOFF.md`、`host/README.md`、各 driver/tests；
- run first：`git status --short`、`git submodule status`、Host 151 tests；
- confirmed hardware config：Feetech C001/RS-485/115200/4096 counts、ID 1、
  `zero_raw=2130`、`direction_sign=+1`/`+X`，shoulder/elbow 当前代码配置与 STM32 验收 tag；
- must not guess：Feetech 最终 limits/负载速度、真实 link lengths、最终 travel、vacuum success；
- uncommitted：本次 C001 profile、CLI、tests 和文档更新，另有既存文档目录调整；
  STM32 submodule clean；
- next milestone：Feetech 完整反馈和调试范围复验，随后实现 arrival/timeout 和 coordinator。
