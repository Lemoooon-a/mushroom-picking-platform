# 蘑菇采摘平台当前进度

更新时间：2026-08-06（Asia/Shanghai）

证据范围：当前源码、配置、提交历史、STM32 子模块文档以及整理前最近一次 Host 离线测试记录；本轮目录重组未执行测试、编译、网络同步、固件构建或真实硬件命令。

## 1. 技术结论

项目已从“跨层功能散落在脏工作树”推进到“依赖顺序明确、可在本地 Git 中复现的 Host 子系统集成”阶段。STM32 Slide/Z、物理开关归零和 machine protocol 仍是硬件证据最成熟的部分；Host 已形成五轴点到点控制、关节 holding 生命周期、吸盘语义、Base frame（基座坐标系）偏移规划、安全过渡、培养槽门限、应用控制器与视觉能力门禁。

S1、R1–R6、C1/C2 已固化功能基线；O1/O2 又将配置来源和离线测试按职责分层，O3 同步当前文档。
目录迁移前最近一次已知结果为 524 项 Host 测试通过；迁移后按明确指令没有重新运行测试。因此
当前结构是否仍满足全部断言尚需后续独立验证，也不能据此证明真实五轴、培养槽作业、吸附、视觉抓取或完整采摘流程已完成硬件验证。

当前最大缺口是完整机械系统验证：新增的 Robot motion envelope（机器人运动包络）只集中
startup 与中间安全阶段策略，不是完整碰撞/物理包络；仍缺五轴阶段运动日志、真空反馈、已验证
`tool_T_camera` 手眼外参、视觉抓取与故障恢复证据。下一主要里程碑应是受控的子系统硬件回归。

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
| 视觉到机器人 | Implemented with gate | observation、Camera→Tool→Base 解析、未验证外参 fail-closed | `tool_T_camera` 当前缺失/未验证 | Offline tested；真实运动 blocked |
| 完整采摘任务 | Not verified | 应用 API、startup、Base 目标、吸盘与返回接口已具备 | 感知、吸附确认、搬运/释放策略、恢复状态机 | 尚未 system validated |

## 3. 仓库与版本状态

### 3.1 仓库拓扑

| 仓库 | 关系 | 分支 / upstream | 当前提交序列 | 最终工作树 | 可复现性 |
| --- | --- | --- | --- | --- | --- |
| `/Users/sd/Projects/mushroom-picking-platform` | 根仓库 | `main` / `origin/main` | R1–R6、C1/C2、O1 `e55b628`、O2 `c9274db`、O3 本报告提交 | O3 后目标为 clean | 本地提交可追溯；尚未 push |
| `firmware/stm32_motion_controller` | `.gitmodules` + root gitlink 正式子模块 | `refactor/generic-stepper-driver` / 无 upstream | S1 `6ee9a62b210e798e6199a97889cde424afca6b8f` | clean | root 已锁定 S1；跨机器取用前需先发布子模块提交 |

根仓库只有一个 worktree。本轮未 fetch，也未 push，因此不把本地 upstream 计数解释为远端实时状态。

### 3.2 本轮提交

| 标识 | Commit | Message | 结果 |
| --- | --- | --- | --- |
| S1 | `6ee9a62` | `fix(motion): finalize Z home direction and machine range` | 子模块 5 文件；工作树 clean |
| R1 | `3a265b1` | `chore(host): sync verified axis and joint configuration` | gitlink、轴/关节配置与只读诊断同步 |
| R2 | `e886344` | `feat(host): add joint holding lifecycle and suction control` | holding、吸盘与统一接口 |
| R3 | `0da9633` | `feat(host): add offset workspace planning and safe transitions` | 偏移规划与阶段过渡 |
| R4 | `62d5011` | `feat(host): add tray-gated application and vision capability boundary` | 托盘、应用、手眼与视觉门禁 |
| R5 | `78ccb62` | `feat(host): integrate startup demo with application safety lifecycle` | demo/application 安全生命周期 |
| R6 | `cd95ea9` | `docs: synchronize repository status and interface evidence` | 上一轮状态与交接证据同步 |
| C1 | `bca3443` | `refactor(host): separate workspace and motion envelope config` | 配置模型、注入、CLI 输出与测试 |
| C2 | `022879a` | `docs(host): clarify configuration and workspace roles` | 配置职责与工作区文档同步 |
| O1 | `e55b628` | `refactor(host): organize project and local configuration layout` | project/examples/local 分层、loader 与 import 路径同步 |
| O2 | `c9274db` | `refactor(host): group offline tests by domain` | 41 个测试文件按领域分组并集中 helper |
| O3 | 本报告所在提交 | `docs: synchronize configuration and test layout` | README、进度、handoff、标定及接口路径同步 |

### 3.3 未提交与本机配置

C2 后以 `022879a` clean 为本轮起点；O3 终检目标是根仓库、根 index 和子模块工作树均 clean。以下本机文件已逐字节迁移，继续由 Git ignore 保护，未进入任何提交：

- `host/config/local/hardware.py`
- `host/config/local/motion.py`
- `host/config/local/five_axis_geometry.json`
- `host/config/local/frame_transforms.json`
- `host/config/local/tray_workspace.json`

五个 local 文件移动前后 SHA-256 逐项一致；没有使用 `git add -f`。本轮执行了受控 `git mv`，没有 stash、reset、rebase、force checkout、硬件 I/O 或 push。

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
                                             └─ MushroomRobotController
                                                      ├─ TrayWorkspace gate
                                                      └─ VisionTargetResolver gate
```

- `firmware/stm32_motion_controller/`：独立版本化的 STM32 平台代码、协议与硬件参数。
- `host/drivers/`、`host/robot/`：平台协议、transport 与执行器适配。
- `host/motion/`：可复用的统一控制、到位、holding 与 suction 语义。
- `host/kinematics/`、`host/geometry/`：算法与坐标变换，不直接访问硬件。
- `host/application/`、`host/vision/`：应用边界、培养槽门禁与视觉能力 fail-closed。
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

视觉边界可以把已验证的 Camera observation 解析为 Base tool goal，但在 hand-eye missing/provisional 时 fail-closed。完整采摘尚缺感知质量、抓取偏置、真空确认、搬运/放置策略、失败恢复与真实培养槽验证，因此不能标记为 Integrated 或 System validated。

## 9. 验证结果

### 9.1 Firmware Builds

本轮未运行 firmware build，也未重新采集 FLASH/RAM。既有 build 只能证明当时可编译，不能证明 S1 参数已在当前硬件上复验。

### 9.2 Host Unit Tests

下表只保留可核实的最近结果和新目录下的命令格式。工作目录为 `/Users/sd/Projects/mushroom-picking-platform/host`。

| 阶段 | 命令 | 结果 |
| --- | --- | --- |
| 最近一次目录迁移前全量 | `.venv/bin/python -m unittest discover -s tests -q` | exit 0；Ran 524；OK；无真实硬件 I/O |
| O1–O3 目录整理 | 未运行测试或编译命令 | 按明确指令未执行；不得写成迁移后 tests pass |
| 新目录指定模块格式 | `.venv/bin/python -m unittest tests.kinematics.test_base_frame_solver tests.motion.test_unified_controller -q` | 仅更新命令格式；本轮未执行 |

### 9.3 Mathematical Tests

五轴 FK/IK、Base solver、offset workspace、motion envelope 注入、transition planner、RigidTransform、
frame chain、tray boundary 和 vision transform 均包含在迁移前最近一次 524 项测试记录中。输入为合成值或本地
example，不读取真实机器标定来证明精度；本轮未复跑。

### 9.4 Electrical Bench Tests

本轮未执行。历史证据支持 STM32 protocol/执行器组件和少量 Rotation/MG4010 读取，但没有新增 bench log。

### 9.5 Mechanical Tests

本轮未执行。S1 保存的是此前测试过的 Z Home 与负机器坐标行为，不是新一次硬件验收。

### 9.6 Integrated System Tests

迁移前最近一次 Host 离线集成记录通过；本轮未运行集成测试，也没有真实五轴、托盘、视觉、吸附或完整采摘系统测试。

## 10. 资源与性能

- 历史记录中 Host 全量 524 项测试单次约 0.72–0.83 秒，进程 wall 约 1 秒；本轮未重新测量。
- STM32 protocol 最大 frame 为 96 bytes；UART 日志流量仍可能影响 machine protocol 压力边界。
- CAN transport 有 timeout/retry 与锁；真实双电机长期吞吐和错误恢复未验收。
- 没有本轮可信的新 FLASH/RAM、ISR timing 或机械节拍数据，不能猜测。

## 11. 安全默认行为

- STM32 上电不运动、不自动 home；Slide/Z enable 默认关闭；pump/release 默认 off。
- Host runtime 构造允许设备发现但不 open；`runtime.open()` 不自动 home/move。
- 默认 runtime mode 为 `READ_ONLY`；真实运动需要显式模式和 CLI 确认。
- startup 执行顺序为 suction idle、rotary holding enable/verify、Z Home、Slide Home、startup pose；offline 模式不发送这些命令。
- stop 不自动移除 holding；disable 前要求确认静止并提示支撑机构。
- communication/fault/timeout 走 terminal error 与 best-effort stop；不能等同于硬件急停。
- hand-eye missing/provisional 时视觉运动 fail-closed；Base 手工运动不因此被禁用。
- 本轮未发现或修改任何 boot-test/自动真实运动宏；未开启真实运动默认项。

## 12. 已知问题和风险

### Confirmed issues

1. `tool_T_camera` 没有已验证值，视觉目标运动不可用。
2. 无真空传感器，无法从软件确认吸附成功。
3. Rotation 无已验证独立 stop；统一 stop 只能 best-effort。
4. Robot motion envelope 配置已集中 startup/clearance，但完整机械包络与碰撞/扫掠路径模型尚不存在。
5. 仓库没有本轮 Host 组合能力的真实硬件日志，离线通过不能升级为 hardware-tested。
6. S1 与根提交尚未 push；root gitlink 在其他机器可获取之前，必须先发布子模块提交。

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
- 是否扩展 Robot motion envelope 以表达经硬件验证的机械范围；碰撞区、阶段轨迹和跨执行器故障传播策略仍待设计。
- 是否将单机 tracked 参数迁移为多机器 profile；该决定不得与当前功能提交混合。

## 14. 下一阶段建议

### P0 — 发布本地可复现基线

- 目标：经人工 review 和独立测试后，先 push 子模块 S1，再 push 根仓库 R1–R6、C1/C2、O1–O3，保证 gitlink 可获取。
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

### P3 — 视觉与完整采摘流程

- 目标：完成 hand-eye、目标质量门限、抓取 offset、真空确认、搬运/释放和恢复状态机。
- 验收：独立验证集通过，Camera target 才从 unavailable 升级；完整流程需重复成功与失败恢复证据。

本轮已完成配置和测试目录迁移，但按明确指令未运行测试。后续独立验证应确认 discovery、指定模块路径和
524 项历史测试数量没有回退，且不得连接真实硬件。

## 15. 交接信息

- 当前阶段：Host 子系统离线集成已固化；真实完整采摘未验证。
- 当前任务：O1/O2 已提交，O3 为本报告所在提交；等待静态终检与人工 review，未 push。
- 首读文件：本报告、`host/config/README.md`、`host/tests/README.md`、`host/config/project/robot_motion_envelope.py`、`host/scripts/run_motion_demo.py`、`host/kinematics/base_move_transition_planner.py`、`host/application/controller.py`。
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
- 不得猜测：真实 link/外参、最终行程、碰撞包络、真空阈值、抓取 offset、急停与失能机械后果。
- 非忽略未提交变更：O3 终检目标为无；5 份机器专属 local 配置继续 ignored。
- 下一里程碑：先发布可获取的子模块/root 提交，再执行有硬件急停保障的 P1 回归。
