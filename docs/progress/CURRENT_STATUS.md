# 蘑菇采摘平台当前进度

更新时间：2026-08-06（Asia/Shanghai）

证据范围：当前源码、配置、提交历史、STM32 子模块文档、Host 编译、553 项离线测试，以及 Robot Service dry-run CLI 的 startup/status/observe/plan-observation/quit 冒烟结果；本轮未执行网络同步、固件构建或真实硬件命令。

## 1. 技术结论

项目处于 Host 子系统离线集成阶段。STM32 Slide/Z、物理开关归零和 machine protocol 仍是硬件证据最成熟的部分；Host 现在把五轴点到点控制、关节 holding、吸盘命令、Base 规划、视觉协议、拍照快照、抓取计划与应用状态统一收口到 `MushroomRobotService`。

V1–V3 分别实现版本化 Vision Gateway、原子 PickPlan/Workflow 和顶层 Robot Service；V4 同步本报告与接口文档。553 项 Host 离线测试全部通过，dry-run CLI 不再构造硬件 runtime，状态中明确记录 `submitted_hardware_commands=0`。这证明软件框架和合成路径可复现，不证明真实五轴运动、培养槽作业、吸附或自动采摘已完成硬件验证。

当前最大缺口是真实数据与机械系统验证：仍缺真实视觉 producer、已验证 `tool_T_camera`、经确认的 GraspProfile、真空反馈和完整碰撞模型。下一主要里程碑应先完成手眼/视觉/grasp 数据闭环，再进行受控小范围硬件验证。

## 2. 总体进度矩阵

| 子系统 | 当前状态 | 已实现能力 | 主要缺口 | 验证级别 |
| --- | --- | --- | --- | --- |
| STM32 Slide | Implemented | STEP/DIR、mm 换算、绝对/相对机器坐标、软限位、开关归零 | 最终全行程与异常矩阵仍需复验 | 文档记录 mechanically tested；本轮未复测 |
| STM32 Z | Implemented | 最高点 Home、向下负坐标、`-190..0 mm`、位置有效性 | S1 重新烧录后的归零与边界回归 | 文档记录 mechanically tested；本轮未复测 |
| STM32 machine protocol | Implemented | ASCII v2 request/response/event、序号、错误码、状态与停止 | 长时串口压力和日志干扰 | Bench evidence + Host contract tests |
| TMC5160 | Implemented | SPI 配置、STEP/DIR、状态/故障轮询、物理开关 homing | DIAG/堵转和负载边界证据不完整 | 组件级硬件记录 |
| Vacuum pump/release | Implemented | `SU/SR/SX`、Host `grip/release/idle` 适配 | 无压力/真空传感器与吸附成功判定 | Offline tested |
| MG4010E CAN | Implemented | transport、协议 codec、驱动、读状态、位置与 holding 命令 | 长时双电机总线、掉线恢复 | Offline tested；历史实机证据有限 |
| Shoulder/Elbow | Implemented | 零点、方向、36:1、软限位、enable/disable/state、到位检测 | 原始标定记录与负载回归 | Offline tested；参数声明来自实测 |
| Rotation | Implemented | RS-485 协议、位置换算、`-150..150°`、torque 生命周期 | 无已验证独立 stop；负载速度未验收 | Offline tested；历史小角度证据 |
| 五轴运动学 | Implemented | FK/IK、偏移工作区、关节限位过滤、确定性候选选择 | 真实几何与多姿态机械复核 | Mathematical/offline tested |
| 多执行器协调 | Implemented | 五轴点到点提交、到位稳定窗、timeout、peer stop、startup/return | 不是轨迹同步；无碰撞/扫掠模型 | Offline integrated |
| 坐标与工作区策略 | Implemented | Base/Slide/Arm/Tool 链、Tray 最终 TCP、arm-local offset、startup/clearance envelope | 完整机械/碰撞包络尚未建模 | Mathematical/offline tested |
| 视觉通信与观察 | Implemented | JSON v1、Fake/Socket gateway、timeout/长度/frame/request 校验、CaptureSnapshot | 无真实视觉 producer、内参/深度证据 | Offline tested；真实 producer unavailable |
| 视觉到机器人 | Implemented with gate | q_capture 绑定、Camera→Tool→Base、低质量/过期/手眼缺失 fail-closed | `tool_T_camera` 当前缺失/未验证 | Offline tested；真实运动 blocked |
| 抓取规划与流程 | Implemented with gate | GraspProfile、pre/contact/retreat 原子计划、dry-run/execute 阶段、FAULT 规则 | 真实 profile、真空确认、放置/释放、自动恢复 | Offline tested；hardware-blocked |
| 顶层运行服务 | Implemented | 单一 Service、状态、capabilities、CLI、JSONL 记录、三种模式 | execute 尚无本轮实机证据 | Offline tested |
| 完整采摘任务 | Not verified | 软件侧观察/规划/抓取框架已集成 | 真实感知、物理吸附确认、搬运/释放和系统验证 | unavailable / not system validated |

## 3. 仓库与版本状态

### 3.1 仓库拓扑

| 仓库 | 关系 | 分支 / upstream | 当前提交序列 | 最终工作树 | 可复现性 |
| --- | --- | --- | --- | --- | --- |
| `/Users/sd/Projects/mushroom-picking-platform` | 根仓库 | `main` / `origin/main` | 起点 `f3f3802`；V1 `86bb07a`、V2 `c04b787`、V3 `d17a887`、V4 本报告提交 | V4 后目标为 clean | 本地提交可追溯；尚未 push |
| `firmware/stm32_motion_controller` | `.gitmodules` + root gitlink 正式子模块 | `refactor/generic-stepper-driver` / 无 upstream | S1 `6ee9a62b210e798e6199a97889cde424afca6b8f` | clean | root 已锁定 S1；跨机器取用前需先发布子模块提交 |

根仓库只有一个 worktree。本轮未 fetch，也未 push，因此不把本地 upstream 计数解释为远端实时状态。

### 3.2 Robot Service 集成提交

| 标识 | Commit | Message | 结果 |
| --- | --- | --- | --- |
| V1 | `86bb07a` | `feat(host): add versioned vision gateway protocol` | 协议、Fake/Socket gateway、snapshot、配置、测试与协议文档 |
| V2 | `c04b787` | `feat(host): add grasp planning and pick workflow` | GraspProfile、三阶段原子计划、Workflow、结果语义与测试 |
| V3 | `d17a887` | `feat(host): add top-level robot service runtime` | Service 状态/API、chained plan、纯离线后端、CLI、JSONL 与测试 |
| V4 | 本报告所在提交 | `docs: document vision and pick runtime boundaries` | runtime/pick/grasp 文档、README 与当前进度同步 |

### 3.3 未提交与本机配置

本轮以 `f3f3802` clean 为起点；V4 终检目标是根仓库、根 index 和子模块工作树均 clean。以下本机文件继续由 Git ignore 保护，未进入任何提交：

- `host/config/local/hardware.py`
- `host/config/local/motion.py`
- `host/config/local/five_axis_geometry.json`
- `host/config/local/frame_transforms.json`
- `host/config/local/tray_workspace.json`

本轮未修改五个既有 local 文件，也未创建 `local/vision_runtime.json` 或 `local/grasp_profile.json`。没有使用 `git add -f`、stash、reset、rebase 或 force checkout；没有硬件 I/O 或 push。

## 4. 系统架构

```text
STM32 Slide / Z / Vacuum firmware
        ↑ ASCII serial machine protocol v2
STM32MotionClient ───────────────────┐
                                    │
MG4010E CAN joint layer ────────────┼─> UnifiedMotionController
                                    │        ├─ holding / suction lifecycle
Feetech Rotation axis ──────────────┘        ├─ BaseFrameFiveAxisSolver
                                             ├─ BaseMoveTransitionPlanner
                                             └─ MushroomRobotController ← PickPlanner
                                                      ↑                    ↑
                                               VisionPickWorkflow ← VisionGateway
                                                      ↑
                                               MushroomRobotService
```

- `firmware/stm32_motion_controller/`：独立版本化的 STM32 平台代码、协议与硬件参数。
- `host/drivers/`、`host/robot/`：平台协议、transport 与执行器适配。
- `host/motion/`：可复用的统一控制、到位、holding 与 suction 语义。
- `host/kinematics/`、`host/geometry/`：算法与坐标变换，不直接访问硬件。
- `host/application/`、`host/vision/`：Service 状态/记录、培养槽门禁、抓取工作流、视觉协议/gateway 和 fail-closed 解析。
- `host/scripts/`：人工入口；真实动作需显式授权/确认，import 不应产生硬件 I/O。
- `host/config/` 根：typed models、loaders 和 schema。
- `host/config/project/`：当前项目机器的 tracked 参数与规划策略。
- `host/config/examples/`：可提交模板，不会自动作为 validated 配置加载。
- `host/config/local/`：机器专属运行参数和真实标定结果；除 `__init__.py` 外均 ignored。
- `host/calibration/`：标定算法、状态模型和 capture/solve 代码；不保存真实机器 JSON 结果。
- `host/config/project/workspace_planning.py`：arm-local 偏置求解策略；不含 Base clearance。
- `host/config/project/robot_motion_envelope.py`：startup/return 与中间安全阶段策略；不是碰撞模型。
- `host/tests/`：按 config、geometry、kinematics、calibration、protocol、motion、application、vision、cli、hardware_adapter、integration 分组的离线测试。
- CubeMX/Core 等生成代码与手写 `App/` 代码在子模块中保持原有边界，本轮未重构。

## 5. STM32 固件进度

### 5.1 Slide and Z Motion

`App/Src/motion_platform_config.c` 固化 Slide `0..33333 step` 与 Z `-60800..0 step`；Host 对应为 Slide `0..799.988 mm`、Z `-190..0 mm`。Z Home 位于最高点，向下为负。S1 只固化已存在且此前声明实机测试过的方向、行程与文档，没有执行本轮实机复验。

### 5.2 TMC5160 Configuration and Diagnostics

固件保留 TMC5160 SPI 配置、STEP/DIR 运动、状态/故障读取和周期轮询。当前证据支持组件实现与既有硬件调试，不足以证明所有负载、DIAG 与堵转边界。

### 5.3 Homing and Position Validity

Slide/Z 使用物理开关归零；成功后才设置 `homed`/`position_valid`。上电不自动 home。stop、fault 或位置不可信路径不会被离线测试描述为“已安全到位”。Z 搜索方向、距离与负坐标协议向量已随 S1 同步。

### 5.4 UART Logging and Machine Protocol

machine protocol v2 提供带 command id 的接受响应与终态 event；Host contract tests 读取子模块文档/向量锁定格式。最大 frame 为 96 bytes。日志与 machine protocol 共用 UART 时仍需做长时间压力验证。

### 5.5 Vacuum Pump and Release Valve

固件命令 `SU`、`SR`、`SX` 分别映射吸附、释放与 idle；Host 提供语义别名和 `SuctionController`。上电 pump/release 默认 off。本项目没有真空传感器，因此“命令完成”不等于“已抓住蘑菇”。

### 5.6 Fault, Limit, and Emergency Handling

固件维持独立软限位和 fault 保护；Host limit 不是底层保护的替代品。软件 stop 不是硬件急停；本轮没有修改急停硬件或宣称 abrupt stop 后位置必然有效。

## 6. MG4010E 关节控制进度

- CAN transport 具有 timeout、retry 与共享锁；protocol/driver 增加了 enable、disable 和协议定义状态读取。
- `CanRotaryJoint` 使用 36:1、输出绝对角逻辑零点、方向和软限位；shoulder 为 `[-65,65]°`，elbow 为 `[-160,160]°`。
- `logical-angle` 是只读诊断，按有符号最短角差输出 `[-180,180)`，不会初始化、使能或运动。
- `UnifiedMotionController` 管理 holding 生命周期、rollback、到位稳定窗、timeout、stop 与故障传播。
- 离线协议、driver、joint、maintenance CLI 和 controller 测试通过；本轮没有真实 CAN 命令。
- 缺口仍包括原始标定记录、双电机负载/长时通信、异常断电恢复和整机 stop 策略。

## 7. 标定与运动学进度

- Shoulder：输出绝对角 `100°` 为逻辑零点，方向 `+1`，范围 `[-65,65]°`。
- Elbow：输出绝对角 `158°` 为逻辑零点，方向 `-1`，范围 `[-160,160]°`。
- Rotation：`zero_raw=2130`、方向 `+1`，范围 `[-150,150]°`。
- 正偏移 arm-local 工作区：X `[50,450] mm`、Y `[150,350] mm`；负偏移：X `[50,450] mm`、Y `[-350,-150] mm`。
- 培养槽最终 TCP 目标使用 Base frame 绝对坐标：X `[20,480] mm`、Y `[20,700] mm`、Z `[0,180] mm`。
- 五轴 solver 支持 FK/IK、可达性、关节限位过滤、偏移候选与确定性选择；transition planner 生成 `DIRECT` 或 `LIFT/TRANSIT/LOWER` 阶段。
- 真实几何来自被忽略的 `host/config/local/five_axis_geometry.json`，Base 外参来自 `host/config/local/frame_transforms.json`。数学测试通过不等于机构无碰撞或实机目标正确。
- 当前 `tool_T_camera` 缺失或未验证；Base 手工运动可用，Camera 目标在 FK/planner/submit 前被拒绝。

## 8. 系统协调与采摘任务

统一控制器覆盖 Slide、Z、shoulder、elbow、rotation 和 suction/holding 生命周期。五轴位置提交是低速点到点协调，不是严格同步轨迹。group failure/timeout 会 best-effort stop 可停止的 peer；Rotation 没有已验证独立 stop，不能承诺所有轴同时停止。

`MushroomRobotController` 提供 startup、Base pose plan/move、return、stop、holding、suction、status 与 shutdown。最终任务目标必须先通过 `TrayWorkspace`；startup、return、`LIFT`、`TRANSIT` 不被普通最终目标 Z 门限误拦截。

`MushroomRobotService` 是唯一进程级应用入口。Vision Gateway 只通信/校验，snapshot 把 observation 绑定到拍摄姿态，resolver 只组合 Camera→Base，PickPlanner 生成 pre/contact/retreat，Workflow 只编排阶段；最终运动仍经 controller 的 Base 出口。

当前 hand-eye missing，tracked grasp example 保持 `validated=false`/null，所以 Camera observation 可显示，但 `plan-observation`/`pick` 明确拒绝。无真空反馈时即使 retreat 完成也只能返回 `PHYSICAL_PICK_UNVERIFIED`。完整采摘仍不能标记为 System validated。

## 9. 验证结果

### 9.1 Firmware Builds

本轮未运行 firmware build，也未重新采集 FLASH/RAM。既有 build 只能证明当时可编译，不能证明 S1 参数已在当前硬件上复验。

### 9.2 Host Unit Tests

下表只保留可核实的最近结果和新目录下的命令格式。工作目录为 `/Users/sd/Projects/mushroom-picking-platform/host`。

| 阶段 | 命令 | 结果 |
| --- | --- | --- |
| 起始基线 | `.venv/bin/python -m unittest discover -s tests -q` | exit 0；Ran 524；OK；附件提供且本轮前复核基线一致 |
| V1 定向 | vision protocol/gateway/snapshot + 既有 resolver/controller | exit 0；Ran 23；OK |
| V2 定向 | GraspProfile/PickWorkflow + 既有 controller/resolver | exit 0；Ran 24；OK |
| V3 定向 | Robot Service/CLI/workflow/controller/integration/bootstrap | exit 0；Ran 63；OK |
| V4 前全量 | `.venv/bin/python -m unittest discover -s tests -q` | exit 0；Ran 553；OK；0.904s；无真实硬件 I/O |
| dry-run CLI | `robot_service.py --mode dry-run --fake-position 0 0 100` 后 startup/status/observe/plan-observation/quit | exit 0；observation 成功；hand-eye 明确拒绝；`submitted_hardware_commands=0` |

### 9.3 Mathematical Tests

五轴 FK/IK、Base solver、offset workspace、motion envelope、transition planner、RigidTransform、
frame chain、tray boundary、vision protocol/gateway/snapshot、grasp planner 与状态机均包含在通过的 553 项测试中。输入为合成值或本地
example，不读取真实机器标定来证明精度。

### 9.4 Electrical Bench Tests

本轮未执行。历史证据支持 STM32 protocol/执行器组件和少量 Rotation/MG4010 读取，但没有新增 bench log。

### 9.5 Mechanical Tests

本轮未执行。S1 保存的是此前测试过的 Z Home 与负机器坐标行为，不是新一次硬件验收。

### 9.6 Integrated System Tests

本轮 Host 离线集成测试随 553 项 discovery 通过；没有真实五轴、托盘、视觉、吸附或完整采摘系统测试。

## 10. 资源与性能

- 本轮 Host 全量 553 项测试用时 0.904 秒。
- Vision socket 默认 timeout `2.0 s`、最大单消息 `65536 bytes`；不自动重连。
- STM32 protocol 最大 frame 为 96 bytes；UART 日志流量仍可能影响 machine protocol 压力边界。
- CAN transport 有 timeout/retry 与锁；真实双电机长期吞吐和错误恢复未验收。
- 没有本轮可信的新 FLASH/RAM、ISR timing 或机械节拍数据，不能猜测。

## 11. 安全默认行为

- STM32 上电不运动、不自动 home；Slide/Z enable 默认关闭；pump/release 默认 off。
- Robot Service read-only/dry-run 不构造硬件 runtime；execute 才构造既有 runtime，且 `runtime.open()` 本身不自动 home/move。
- 默认 runtime mode 为 `READ_ONLY`；真实运动需要显式模式和 CLI 确认。
- startup 执行顺序为 suction idle、rotary holding enable/verify、Z Home、Slide Home、startup pose；offline 模式不发送这些命令。
- stop 不自动移除 holding；disable 前要求确认静止并提示支撑机构。
- communication/fault/timeout 走 terminal error 与 best-effort stop；不能等同于硬件急停。
- hand-eye missing/provisional 时视觉运动 fail-closed；Base 手工运动不因此被禁用。
- grasp profile 缺失/未验证、no target、低 confidence、过期 observation 和规划拒绝均不运动；motion/suction failure best-effort stop 后进入 FAULT。
- 本轮未发现或修改任何 boot-test/自动真实运动宏；未开启真实运动默认项。

## 12. 已知问题和风险

### Confirmed issues

1. `tool_T_camera` 没有已验证值，视觉目标运动不可用。
2. 无真空传感器，无法从软件确认吸附成功。
3. Rotation 无已验证独立 stop；统一 stop 只能 best-effort。
4. Robot motion envelope 配置已集中 startup/clearance，但完整机械包络与碰撞/扫掠路径模型尚不存在。
5. 仓库没有本轮 Host 组合能力的真实硬件日志，离线通过不能升级为 hardware-tested。
6. S1 与根提交尚未 push；root gitlink 在其他机器可获取之前，必须先发布子模块提交。
7. 真实 socket vision producer 与 validated GraspProfile 均不存在；当前只有 Fake gateway 的合成验证。

### Risks

- 当前 tracked joints/Rotation 值兼具项目配置与实机标定属性，未来多机器部署可能需要 profile 化。
- Home 时 TCP Base Z 当前恰为 `180 mm`，与 tray `z_max` 相同；二者语义不同，机械变化时不能自动耦合。
- 失能会移除 holding，机构可能下坠；软件 stop 不是急停。
- `DIRECT/LIFT/TRANSIT/LOWER` 解决阶段顺序，不保证路径无碰撞。
- local JSON/Python 配置被正确忽略，但人工复制 example 后仍可能保留 placeholder。

## 13. 开放决策

- Slide 最终机械全行程、速度/加速度和异常矩阵。
- Z 重新烧录 S1 后的 Home、`-190..0 mm`、上下边界和故障恢复验收。
- Shoulder/Elbow/Rotation 最终负载速度、标定证据与安全失能策略。
- vacuum sensor、阈值、吸附成功定义和 emergency release policy。
- `tool_T_camera`、Camera frame/内参/深度、抓取 offset 与独立验证阈值。
- 真实视觉 producer 的部署地址、时间基准、断线运维策略和最大延迟。
- 是否扩展 Robot motion envelope 以表达经硬件验证的机械范围；碰撞区、阶段轨迹和跨执行器故障传播策略仍待设计。
- 是否将单机 tracked 参数迁移为多机器 profile；该决定不得与当前功能提交混合。

## 14. 下一阶段建议

### P0 — 发布本地可复现基线

- 目标：经人工 review 后，先 push 子模块 S1，再 push 根仓库 R1–R6、C1/C2、O1–O4，保证 gitlink 可获取。
- 文件：无新增源文件；只发布现有提交。
- 验收：新 clone 能初始化子模块并运行 524 项测试。
- 安全：发布不等于硬件验收；本轮未执行 push。

### P1 — Z/Slide 与 holding 受控硬件回归

- 目标：验证 S1 固件、Z 负坐标、双轴 Home、MG4010/Rotation holding 和 stop/fault 路径。
- 证据：日期、固件 hash、根 hash、命令、初始姿态、边界、日志、异常与人工急停准备。
- 验收：低速、空载、小范围通过后再验证边界；不得跨过未确认软限位。

### P2 — 五轴阶段与培养槽门限验证

- 目标：逐个验证 startup、`DIRECT/LIFT/TRANSIT/LOWER`、return 和 tray 最终目标拒绝。
- 验收：多姿态 FK 残差、实际到位、clearance、timeout/peer stop 与恢复记录完整。
- 安全：先建立机械包络和碰撞禁区，软件 stop 外另备硬件急停。

### P3 — 视觉与完整采摘验证

- 目标：接入真实 producer，完成 hand-eye、GraspProfile、真空确认、搬运/释放和恢复策略。
- 验收：真实 observation 与独立标定验证集通过后才允许 Camera motion；完整流程需重复成功、失败恢复和物理吸附证据。

本轮已完成软件侧 Robot Service/vision/pick 框架并通过 553 项离线测试；不得把该结果解释为真实硬件验收。

## 15. 交接信息

- 当前阶段：Host Robot Service、视觉 gateway/observation 和抓取规划框架已离线集成；真实完整采摘未验证。
- 当前任务：V1–V3 已提交，V4 同步文档与最终验证；未 push。
- 首读文件：本报告、`docs/interfaces/ROBOT_SERVICE_RUNTIME.md`、`VISION_GATEWAY_PROTOCOL.md`、`PICK_WORKFLOW.md`、`host/scripts/robot_service.py`、`host/application/robot_service.py`。
- 首跑命令：

```bash
cd /Users/sd/Projects/mushroom-picking-platform
git status --short
git submodule status
git -C firmware/stm32_motion_controller status --short
cd host
.venv/bin/python -m unittest discover -s tests -q
```

- 已确认并提交的关键语义：Z Home `0 mm`、向下负坐标、Z `-190..0 mm`；Shoulder `[-65,65]°`；Elbow `[-160,160]°`；Rotation `[-150,150]°`。
- 不得猜测：真实外参、视觉深度/方向、抓取 offset、碰撞包络、真空阈值、急停与失能机械后果。
- 非忽略未提交变更：V4 终检目标为无；既有 5 份机器专属 local 配置继续 ignored，新增 2 份 local 配置尚未创建。
- 下一里程碑：采集并独立验证 hand-eye、真实视觉 observation 和 GraspProfile，再设计有硬件急停保障的最小动作验证。
