# Unified Motion Interface Gap Analysis

> 审查日期：2026-08-03  
> 目标规范：`docs/interfaces/UNIFIED_MOTION_CONTROL_INTERFACE_PROTOCOL_v0.2.md`  
> 证据优先级：当前源码与配置 > 本轮离线测试 > 当前文档 > 历史说明  
> 审查边界：只盘点、对照和提出建议；未修改源码、测试、协议或 STM32 子模块，未创建 commit，未访问真实硬件。

## 1. Executive Conclusion

### 1.1 主结论

**B. 当前底层能力基本具备，但缺少统一中间层。**

这并不表示当前接口可以直接交给前端或运动学模块。现有三类后端已经覆盖协议第一版所需的大部分基础动作：

- Slide/Z：查询、绝对/相对运动、停止、使能/禁用、机械归零、清故障入口和完成事件；
- Shoulder/Elbow：绝对位置读取、逻辑零点/方向/减速比/软限位、非阻塞位置命令和 `0x81` 软件停止；
- Rotation：逻辑角到 raw count 的转换、位置/反馈读取、位置命令和显式 torque enable/disable。

但是，`host/motion/capabilities.py` 只是静态能力声明和后端对象聚合，不执行统一分发、单位转换、状态归一化或错误映射（`host/motion/capabilities.py:8-19,75-94`）。协议要求的 `AxisName`、`AxisDescriptor`、`AxisState`、`MotionResult`、`MotionErrorCode`、`SingleAxisMotionInterface`、`KinematicsMotionInterface` 和系统初始化边界均不存在。因此：

- **不能直接提供给前端使用**：前端仍会看到 `µm`、rad、raw speed、底层异常以及不同方法名；
- **不能把当前控制对象作为运动学稳定依赖**：当前 `Planar2RArmController` 直接依赖带 `JointConfig` 的 rad 制关节协议（`host/robot/planar_arm.py:37-55,127-165`）；
- **可以并行开发运动学算法**：纯数学层无 CAN/串口依赖，并且已有逻辑角、坐标系、可达性与软限位筛选证据；共享边界应先冻结为 Protocol/fake，而不是让运动学依赖具体后端。

### 1.2 量化结果

本报告把协议核心要求和专项审查项拆为 34 项：

| 状态 | 数量 | 含义 |
| --- | ---: | --- |
| `COMPLIANT` | 8 | 当前源码满足且有源码/测试证据 |
| `PARTIAL` | 11 | 有基础结构，但统一语义、能力或测试不完整 |
| `NON_COMPLIANT` | 1 | 当前公开 API 单位与协议明确冲突 |
| `MISSING` | 13 | 无对应统一接口或实现 |
| `NOT_VERIFIABLE` | 1 | 必须依赖硬件参数、标定或底层语义验证 |

最大三个缺口是：

1. 缺少统一类型、控制器、适配器、结果和错误模型；
2. 当前公开单位不统一，且 STM32 没有公开的“只等待接受、不等待完成”提交接口；
3. 缺少独立系统上电初始化层，以及肩肘/旋转轴可靠到位、超时和故障传播能力。

### 1.3 工作零点与完成语义结论

- 全仓库可执行 Host 代码未出现 `startup_position`、`work_zero`、`move_to_work_zero` 或等价计算；未发现工作零点进入正运动学（Forward Kinematics, FK）、逆运动学（Inverse Kinematics, IK）、逻辑角换算、电机目标或软限位。
- 这说明当前不存在“工作零点污染运动学”的已实现缺陷；但 `SystemStartupConfig` 和初始化协调器也完全缺失，不能把“尚未实现”误报为完整合规。
- MG4010 和相关 CLI 明确说明命令应答不代表机械到位（`host/drivers/mg4010_driver.py:89-99`、`host/robot/joint.py:349-354`、`host/scripts/test_joint_position.py:443-447`）。
- STM32 协议层能区分 `OK` 与 `DONE/ABORT/FAULT`，客户端也会等待并区分终态（`host/drivers/stm32_motion.py:309-323`）。
- 当前没有统一 `MotionResult`，所以不存在显式的 `completed=True` 误报代码，但也无法按协议向前端表达 `accepted=True, completed=None`。这是接口缺失，不是可接受的长期状态。

## 2. Repository and Test Baseline

### 2.1 Git 基线

| 项目 | 本轮开始时结果 |
| --- | --- |
| 根仓库分支 | `main` |
| 根仓库 HEAD | `673a373 feat(host): integrate upper motion control backends` |
| 最近提交 | `673a373`、`919213b`、`0fb785d`、`0e12380` |
| 工作树 | dirty；存在用户原有 modified/deleted/untracked 文件 |
| STM32 submodule | `cb075675e32cd5c5e9e5d1d43ddaa5e539fdc8d4 firmware/stm32_motion_controller (stm32-motion-v0.1.0)` |
| submodule 状态 | `git submodule status` 无 `+`、`-` 或 `U` 前缀；本轮未修改 |

审查开始前的 `git status --short` 为：

```text
 D docs/hardware/.gitkeep
 M docs/progress/CURRENT_STATUS.md
 M docs/progress/UPPER_MOTION_CONTROL_HANDOFF.md
 D docs/项目进度_2026-07-31.md
 M host/README.md
 M host/config/__init__.py
 M host/scripts/test_feetech_rotation.py
 M host/tests/test_feetech_rotation.py
?? docs/interfaces/
?? docs/progress/项目进度_2026-07-31.md
?? host/config/feetech.py
?? host/tests/test_feetech_config.py
```

这些改动均视为用户已有工作并被保留。本轮只新增本报告。

### 2.2 离线测试

执行目录：`host/`。

```bash
.venv/bin/python -m unittest discover -s tests -q
```

结果：

```text
Ran 151 tests in 0.207s

OK
```

- exit code：0；
- failures：0；
- errors：0；
- skips：0；
- 硬件参与：无。

无硬件 I/O 的依据：测试为 `FakeTransport`、fake CAN bus、fake serial、`MagicMock` 或 CLI dry-run；例如 `host/tests/test_stm32_motion.py:19-32`、`host/tests/test_upper_motion_smoke.py:14-26`、`host/tests/test_planar_arm.py:123-143`、`host/tests/test_feetech_rotation.py:117-131`。本轮没有调用任何真实脚本的 `--execute`、`--enable-motion`、串口 `open()`、CAN `open()`、torque enable、home 或位置命令。

测试通过只证明离线逻辑和 fake 交互，不证明最终机械行程、方向、负载速度、到位精度或真实通信稳定性。

## 3. Current Architecture

### 3.1 当前实际链路

```text
Slide / Z
  STM32SerialTransport
    -> STM32MotionClient
    -> STM32 ASCII machine protocol (S/Z, integer µm)

Shoulder / Elbow
  CanMotorBus
    -> MG4010Driver (motor degree)
    -> CanRotaryJoint (logical rad)
    -> Planar2RArmController / Planar2RKinematics

Rotation
  FeetechBus
    -> FeetechRotationAxis (logical rad + raw speed/acceleration)

Object aggregation only
  UpperMotionBackends + BackendCapabilities
```

协议期望但当前缺失的链路：

```text
Frontend / Kinematics / Startup Coordinator
                   -> UnifiedMotionController
                   -> per-backend adapters
                   -> existing backends
```

### 3.2 上层逻辑角到电机目标的现有转换链

#### Shoulder / Elbow

```text
上层逻辑角（当前为 rad）
  -> Planar2RArmController.command_target()
     host/robot/planar_arm.py:159-186
  -> CanRotaryJoint.command_position(position_rad, velocity_rad_s)
     host/robot/joint.py:349-447
  -> direction_sign + gear_ratio + current 0x92/0x94 snapshot
     host/robot/joint.py:396-445
  -> MG4010Driver.command_position(motor degree, motor degree/s)
     host/drivers/mg4010_driver.py:89-109
  -> 0xA4 raw payload
     host/drivers/mg4010_protocol.py:163-192
```

逻辑零点、方向和软限位位于 `host/config/joints.py:20-45`；转换层没有 `startup_position`。未发现 Host 上层绕过 `CanRotaryJoint` 直接发送肩肘运动目标。`host/scripts/read_motor_basic_params.py` 直接使用 `MG4010Driver`，但仅用于底层只读诊断；实际肩肘位置脚本通过 `CanRotaryJoint`（`host/scripts/test_joint_position.py:421-446`）。

#### Rotation

```text
上层逻辑角（库 API 当前为 rad；CLI 可接受 deg）
  -> FeetechRotationAxis.command_position()
     host/robot/feetech_rotation.py:203-233
  -> zero_raw + direction_sign + counts_per_turn + soft limit
     host/robot/feetech_rotation.py:93-124
  -> 0x2A position/time/speed raw payload
     host/robot/feetech_rotation.py:127-151,226-232
  -> FeetechBus.write_registers()
     host/drivers/feetech_protocol.py:264-279
```

项目零点、方向和调试限位位于 `host/config/feetech.py:73-85`。正式轴位置命令未绕过 `FeetechRotationAxis`；人工脚本的 `--disable` 分支直接写 torque register（`host/scripts/test_feetech_rotation.py:314-320`），属于底层维护工具，不应成为统一接口调用方式。

#### Slide / Z

```text
调用者当前直接提供 integer µm / µm/s / µm/s²
  -> STM32MotionClient.move_absolute()/move_relative()
     host/drivers/stm32_motion.py:372-408
  -> STM32 S/Z machine protocol
  -> firmware step conversion and machine coordinate
```

Host 目前没有协议要求的 mm 适配器。

### 3.3 导入和构造安全

- `STM32SerialTransport` 构造不打开串口，只有 `open()` 或 context manager 才打开（`host/drivers/stm32_motion.py:92-119,131-136`）；
- `CanMotorBus` 构造不扫描或打开 CAN，只有 `open()` 才扫描/创建 bus（`host/drivers/can_bus.py:66-99,112-161`）；
- `FeetechBus` 构造不打开串口，只有 `open()` 或 context manager 才打开（`host/drivers/feetech_protocol.py:139-184`）；
- 三类 axis/joint 对象构造均不 enable、不归零、不发位置命令；
- 真实运动脚本均要求显式开关，默认 dry-run/preview（`host/scripts/test_planar_2r_motion.py:40-67,179-199`、`host/tests/test_feetech_rotation.py:117-131`）。

未发现 import 或普通对象构造时自动扫描端口、ping、enable、torque enable、home、clear fault 或运动。

## 4. Protocol Compliance Matrix

状态定义严格使用协议要求的五类；`NON_COMPLIANT` 单独保留，不并入 `MISSING`。

| 协议章节 | 协议要求 | 当前实现 | 状态 | 证据 | 修改建议 |
| --- | --- | --- | --- | --- | --- |
| §2 | `UnifiedMotionController` + 三类 adapter | 只有后端对象聚合和静态能力表，不分发调用 | `MISSING` | `host/motion/capabilities.py:75-94` | 新增统一控制器；adapter 可先作为统一控制器私有类，避免文件爆炸 |
| §3 | 稳定 `slide/z/shoulder/elbow/rotation`，隐藏底层 ID | 聚合字典已有五个名称，但无 `AxisName`；Rotation 配置名仍为 `end_effector_rotation` | `PARTIAL` | `host/motion/capabilities.py:86-93`；`host/config/feetech.py:75-77` | 定义唯一 `AxisName`；描述符和分发只使用该枚举 |
| §4.1 | Linear 使用 mm，rotary 使用 deg，不暴露 µm/rad/raw | STM32 API 为 integer µm；关节为 rad；Rotation 为 rad + raw speed/acceleration | `NON_COMPLIANT` | `host/drivers/stm32_motion.py:372-405`；`host/robot/joint.py:349-353`；`host/robot/feetech_rotation.py:203-210` | adapter 边界完成 mm/deg 转换；不改变底层内部单位 |
| §4.2/§14.2 | 肩肘逻辑零点、方向、减速比、软限位在关节层 | 已集中在 `JointConfig` 并由转换函数和命令链应用 | `COMPLIANT` | `host/robot/joint.py:52-70,149-221,433-445`；`host/config/joints.py:20-45`；`host/tests/test_joint.py:394-417,465-472` | 保留现有 `CanRotaryJoint`；统一 adapter 仅做 deg↔rad |
| §4.2/§14.3 | Rotation 逻辑零点、方向、软限位隐藏 raw count | 位置转换已完成；但速度/加速度仍是 raw，最终机械配置未验收 | `PARTIAL` | `host/robot/feetech_rotation.py:93-151,203-233`；`host/config/feetech.py:73-85` | adapter 输出 deg；在物理速度映射确认前拒绝非空 deg/s、deg/s² 或明确声明不可配置 |
| §4.3/§11.3 | 只有 Slide/Z home；其他轴明确 `UNSUPPORTED_COMMAND` | 后端能力表正确声明肩肘/旋转无 home，但没有统一方法和统一错误结果 | `PARTIAL` | `host/motion/capabilities.py:22-58` | 在统一控制器按 capability 拒绝，绝不调用旋转轴逻辑零点重设 |
| §4.3 | Home 区分接受、DONE、ABORT、FAULT；成功后 homed/valid | STM32 客户端区分同步错误、FAULT、ABORT、DONE；状态能读取 homed/valid | `COMPLIANT` | `host/drivers/stm32_motion.py:309-338,410-417`；`host/tests/test_stm32_motion.py:70-83` | adapter 在 DONE 后再 `query_axis()`，把最终状态写入 `MotionResult.final_state` |
| §5 | `startup_position` 不进入坐标换算、软限位、FK/IK 或普通命令 | Host 可执行代码完全没有该字段或等价计算 | `COMPLIANT` | 全仓库关键词检索；`host/kinematics/planar_2r.py:21-128`；`host/robot/joint.py:183-193,433-445` | 用静态测试继续锁定；不要把缺少初始化层误报为完整初始化支持 |
| §5.4/§12 | 独立 `SystemStartupConfig` 和初始化协调器 | 无对应文件、类型或协调器 | `MISSING` | `host/motion/__init__.py:1`；全仓库无 `SystemStartupConfig` | P3 新增 `system_startup.py`；只通过普通绝对位置接口执行一次性目标 |
| §6 | `AxisDescriptor` 提供单位、限位和能力，不含 startup | 无该类型或等价描述对象 | `MISSING` | 全仓库无 `AxisDescriptor` | 在协议类型文件中定义，并由控制器只读返回 |
| §7 | 统一 `AxisCapabilities`，显式拒绝不支持命令 | 有 `BackendCapabilities`，但字段名/集合不同，缺 enable、clear_fault、wait_for_completion | `PARTIAL` | `host/motion/capabilities.py:8-19` | 复用/迁移为协议 `AxisCapabilities`；不要同时维护两份会漂移的真值 |
| §8 | 统一 `AxisState` 和 `None` 语义 | 三种底层状态对象直接暴露，字段和单位不同 | `MISSING` | `host/drivers/stm32_motion.py:169-179`；`host/robot/joint.py:121-138`；`host/robot/feetech_rotation.py:75-84` | adapter 构造统一状态；未知/不适用必须为 `None` |
| §9.1 | 稳定 `MotionErrorCode` 分类 | 各层异常较细，但无跨后端映射 | `MISSING` | `host/drivers/stm32_motion.py:20-53`；`host/robot/joint.py:20-49`；`host/drivers/feetech_protocol.py:22-48` | 集中异常映射；保留底层异常作为 cause，不向前端泄漏类型 |
| §9.2 | `MotionResult` 表达 ok/accepted/completed/final_state | 所有命令返回值不同，无统一结果 | `MISSING` | STM32 返回 event；joint 返回命令前状态；Feetech 返回 raw target | 新增不可变结果类型，测试每种接受/终态/失败组合 |
| §10 | 一个 `SingleAxisMotionInterface` 入口 | 不存在；调用方必须知道三类后端 | `MISSING` | `host/motion/__init__.py:1` | 先定义 Protocol/fake，再实现依赖注入控制器 |
| §11.1 | 所有轴统一 `move_absolute()`；限位、不自动使能/home、不附加 startup | 各后端已有绝对运动和限位；方法名、单位、可选速度/加速度和返回语义不统一 | `PARTIAL` | `host/drivers/stm32_motion.py:391-408`；`host/robot/joint.py:349-447`；`host/robot/feetech_rotation.py:203-233` | 统一 controller 做参数校验和 capability 检查，后端继续负责最终安全校验 |
| §11.2 | Relative 只对 Slide/Z 支持，其他轴返回统一错误 | 后端能力表标明差异，但无统一调用或错误结果 | `PARTIAL` | `host/motion/capabilities.py:22-58` | 在 dispatcher 前置检查并返回 `UNSUPPORTED_COMMAND` |
| §11.4 | 各轴 stop 映射正确，Rotation 不伪装 stop，stop 非急停 | Slide/Z、肩肘有 stop；Rotation 标为无 stop；但肩肘同一 `0x81` 又被声明为 disable | `PARTIAL` | `host/robot/joint.py:449-452`；`host/motion/capabilities.py:35-58`；`host/README.md:205-219` | 第一版肩肘只冻结为 software stop；disable capability 在厂商语义确认前设为 false/unsupported |
| §12 | 初始化状态、顺序、失败边界独立于运动学 | 无系统初始化接口或状态 | `MISSING` | `host/motion/` 仅两个极小文件 | P3 单独实现；安全顺序必须来自经验证配置，不写入 FK/IK |
| §13.1/§13.2 | `ArmJointState`/`ArmJointTarget` 使用 deg 且不含 startup | 当前只有 rad 制 `JointAngles`、`JointState`、`PlanarArmTarget` | `MISSING` | `host/kinematics/planar_2r.py:21-35`；`host/robot/planar_arm.py:29-35` | 新增共享 DTO；数学算法内部可继续使用 rad |
| §13.3 | `KinematicsMotionInterface` 只依赖统一层 | 当前 `PositionJoint` 要求具体 `JointConfig`、rad 方法和 `JointState` | `MISSING` | `host/robot/planar_arm.py:37-55` | 先提供 Protocol/fake；保留现有桥接器供 CLI，逐步适配而非重写算法 |
| §13/§21 | FK/IK 使用逻辑关节角，不读取零点、方向、减速比或 startup | 纯数学层只读取连杆长度和逻辑 rad | `COMPLIANT` | `host/kinematics/planar_2r.py:37-128`；`host/tests/test_planar_2r_kinematics.py:141-148` | 继续保持算法层硬件无关；共享 DTO 边界转换 deg↔rad |
| §13.2 | IK 可达性检查并按肩肘逻辑软限位筛选 | 已实现并有测试 | `COMPLIANT` | `host/robot/planar_arm.py:57-124`；`host/tests/test_planar_arm.py:24-74` | 保留现有筛选函数；统一层仍二次校验 |
| §13.3 | 不声称严格同步；部分下发失败尽力停止 | 代码注释明确背靠背而非同步，异常时停止两关节 | `COMPLIANT` | `host/robot/planar_arm.py:127-131,178-205`；`host/tests/test_planar_arm.py:102-119` | 修改错误文案为“submission failed”，避免 `complete` 一词造成到位歧义 |
| §13.3 | 返回每轴独立 `MotionResult`，accepted/completed 准确 | 当前返回两个 `JointState`；失败抛单个异常 | `MISSING` | `host/robot/planar_arm.py:159-205` | 新 `KinematicsMotionInterface` 通过统一单轴接口返回逐轴结果 |
| §14.1 | Slide/Z adapter 做 mm↔µm、状态/动作/事件映射 | 后端动作齐全，但 adapter 不存在，客户端运动 API 总是等终态 | `PARTIAL` | `host/drivers/stm32_motion.py:325-432` | 扩展公开 submit/wait 边界，支持 `wait=False`；不要调用私有 `_send/_wait` |
| §14.2 | Shoulder/Elbow adapter 做 deg↔rad 和 stop 映射 | 关节层安全语义齐全，但 adapter、统一结果和物理加速度语义缺失 | `PARTIAL` | `host/robot/joint.py:349-476` | adapter 只做单位/结果/错误映射；不要重写关节换算 |
| §14.3 | Rotation adapter 做 deg↔rad、torque enable/disable | 位置与 torque 动作具备，但速度/加速度工程单位、enabled 查询、统一 fault 缺失 | `PARTIAL` | `host/robot/feetech_rotation.py:154-233` | 先以 `enabled=None` 表示不可确认；未确认物理映射时不得吞掉 velocity/acceleration 参数 |
| §16 | 前端只依赖描述符、能力、统一状态/结果 | 当前无可供前端稳定消费的接口 | `MISSING` | 无 `list_axes()/describe_axis()` | 不允许前端直接引用 S/Z、CAN ID、servo ID、port 或 raw unit |
| §17 | 常规轴配置与系统启动配置分离 | 常规 shoulder/elbow/rotation 配置存在且无 startup；系统启动配置缺失 | `PARTIAL` | `host/config/joints.py:20-50`；`host/config/feetech.py:73-85` | 新启动配置放 `motion/system_startup.py` 或独立 config 模块，不加入现有轴 config |
| §18 | 不扩展为严格同步、连续轨迹、急停或抓取状态机 | 当前代码明确只做背靠背下发和软件停止 | `COMPLIANT` | `host/robot/planar_arm.py:127-131`；`host/README.md:247-248` | 保持本轮范围；协议实现不要夹带轨迹/抓取状态机 |
| §20 | 最终方向、限位、速度、L1/L2、启动位置来自真实配置/标定 | 多项参数仍只有调试值或无正式配置 | `NOT_VERIFIABLE` | `docs/progress/CURRENT_STATUS.md:220-229`；`host/config/feetech.py:73-85` | 明确保留待配置；不得从脚本示例猜测生产值 |
| §21 | 静态/单元测试锁定 startup 隔离和 unsupported home | 尚无统一类型，因而也无协议边界测试 | `MISSING` | `host/tests/` 无 unified/startup tests | 新增静态字段检查、fake dispatcher、错误/状态/完成矩阵测试 |
| 专项 5.13 | import/构造无自动 I/O 或运动副作用 | 三类 transport 都显式 open；smoke test 证明 fake 可装配 | `COMPLIANT` | `host/tests/test_upper_motion_smoke.py:25-63`；各 transport 生命周期源码 | 保持依赖注入和显式 open；统一控制器构造也不得连接硬件 |

## 5. Per-Axis Capability Matrix

本表表示**当前底层可提供的能力**，不是“已有统一接口”。`Supported` 也不等于真实硬件已完整验收。

| Capability | Slide | Z | Shoulder | Elbow | Rotation |
| --- | --- | --- | --- | --- | --- |
| Query state | Supported | Supported | Supported | Supported | Supported |
| Absolute move | Supported | Supported | Supported | Supported | Supported |
| Relative move | Supported | Supported | Unsupported | Unsupported | Unsupported |
| Stop | Supported | Supported | Supported | Supported | Unsupported |
| Enable | Supported | Supported | Unsupported | Unsupported | Supported |
| Disable | Supported | Supported | Partial | Partial | Supported |
| Reference home | Supported | Supported | Unsupported | Unsupported | Unsupported |
| Clear fault | Supported | Partial | Unsupported | Unsupported | Unsupported |
| Wait for completion | Supported | Supported | Unsupported | Unsupported | Unsupported |
| Position valid | Supported | Supported | Supported | Supported | Partial |

关键限制：

- Slide/Z 的“Supported”来自 `STM32MotionClient` 和固件 machine protocol，但当前公开方法使用 integer µm，且运动方法始终等待事件；尚不能原样满足统一接口默认 `wait=False`。
- Z 具有 `CF` 命令入口，但固件文档称当前只有 Slide stall fault 可清，因此 Z 的可清故障语义仅为 `Partial`（`firmware/stm32_motion_controller/App/README.md:143-152`）。
- Shoulder/Elbow 没有独立 enable；`0x81` 是明确的软件停止。现有 `BackendCapabilities.disable=True` 把同一命令同时当作 stop/disable，暂不能冻结为独立 disable 能力。
- Shoulder/Elbow 的 `position_valid=True` 依赖先执行稳定绝对位置初始化（`host/robot/joint.py:254-305,478-483`），不是机械 home。
- Rotation 成功读取并唯一解释绝对位置后可构造有效位置，但没有独立、持久的 `position_valid` 字段；断线和错误必须由 adapter 映射为 false。
- Rotation torque disable 不能替代 stop，当前 capability 正确标记 `stop=False`（`host/motion/capabilities.py:48-59`）。

## 6. Unit and Coordinate Semantics

### 6.1 单位转换表

| Axis | Public position（协议） | Current API input | Backend unit | Conversion location | Problem |
| --- | --- | --- | --- | --- | --- |
| Slide | mm | integer µm | firmware step | Host 无 mm 转换；firmware µm↔step | 前端需自行乘 1000；明确不合规 |
| Z | mm | integer µm | firmware step | Host 无 mm 转换；firmware µm↔step | 同 Slide |
| Shoulder | deg | rad；velocity rad/s | motor degree / degree/s | `CanRotaryJoint` 处理逻辑 rad↔电机 degree | 缺 deg adapter；无 acceleration 参数 |
| Elbow | deg | rad；velocity rad/s | motor degree / degree/s | 同 Shoulder | 同 Shoulder |
| Rotation | deg | rad；speed/acceleration raw | encoder count/register raw | `FeetechRotationAxis` 处理 position rad↔count | 位置只差 deg adapter；速度/加速度物理映射未确认 |

### 6.2 角度和坐标定义

- `Planar2RKinematics` 当前使用 rad；肩角相对全局 `+x`，肘角相对 link1，正方向从 `+x` 向 `+y`（`host/kinematics/planar_2r.py:37-43`）。
- `JointAngles` 是逻辑角而非电机角（`host/kinematics/planar_2r.py:29-35`）。算法内部使用 rad 并不与协议矛盾；矛盾在于当前缺少向其他模块承诺 deg 的共享边界。
- FK/IK 不应读取关节零点、方向和减速比；这些是执行器映射配置，不是几何模型参数。当前代码符合这一职责分离。
- 逆解按 `JointConfig` 逻辑软限位筛选（`host/robot/planar_arm.py:57-124`），而 `CanRotaryJoint` 在下发前再次检查（`host/robot/joint.py:454-476`）。
- 末端 rotation 完全未进入 Planar 2R 的 XY 求解，当前正确解耦。

### 6.3 Slide/Z 逻辑位置的协议歧义

协议要求 adapter 把 STM32 machine position 表达为“逻辑工作位置”，同时严格禁止把 `startup_position` 当坐标偏移。当前固件定义归零安全位置为 machine zero，并明确不保存 work-zero offset（`firmware/stm32_motion_controller/App/README.md:37-52`）。

因此统一层必须二选一并在实现前冻结：

1. **推荐第一版**：Slide/Z logical zero 等于 homed machine zero，adapter 只做 mm↔µm；培养槽/相机/TCP 偏移留给坐标变换层；
2. 如果业务确需另一套线性逻辑零点，则必须新增独立、经标定的坐标变换配置，不能复用 `startup_position`。

协议当前没有定义第二种偏移的字段与所有权，不能从现有代码推断。

### 6.4 加速度能力的协议缺口

`move_absolute()`/`move_relative()` 接受统一工程加速度，但 `AxisCapabilities` 没有表达“是否支持可配置速度/加速度”：

- STM32 完整支持 mm/s 与 mm/s²；
- MG4010 当前只支持最大速度，没有关节加速度 API；
- Feetech 有 `acceleration_raw`，但 raw 与 deg/s² 的物理映射尚未验证。

统一层不得静默忽略非空 acceleration。最小策略是在描述符默认值使用 `None`，对无法映射的非空参数返回 `UNSUPPORTED_COMMAND` 或 `INVALID_REQUEST`；更长期可为 capability 增加可选的 profile/parameter-support 描述。协议 v0.2 应明确错误选择，否则不同 adapter 会产生不一致行为。

## 7. Homing and Startup Position Separation

### 7.1 机械归零

`STM32MotionClient.home()` 能等待归零事件并区分：

- 同步 `ERR`：`STM32CommandError`；
- 异步 `DONE`：返回 event；
- 异步 `ABORT/FAULT`：`STM32CommandEventError`；
- 等待超时：`STM32MotionTimeoutError`。

证据为 `host/drivers/stm32_motion.py:300-323,410-417` 和 `host/tests/test_stm32_motion.py:70-83,100-102`。但 `home()` 返回 DONE event 后没有自动查询 `AxisStatus`，所以协议层适配必须再读一次状态，才能用证据保证 `homed=True`、`position_valid=True`，而不是只相信 event。

肩、肘和 Rotation 没有 home 方法；现有绝对位置初始化只读取稳定样本，不重写零点（`host/robot/joint.py:254-305`）。未发现“用当前位置重设肩肘逻辑零点”的代码。

### 7.2 工作零点隔离检查

| 检查项 | 当前状态 | 证据 | 是否符合 |
| --- | --- | --- | --- |
| 只存在于启动配置 | 启动配置尚不存在 | Host 源码无 `startup_position` | `MISSING`，非违规 |
| 不在 `AxisDescriptor` | 类型不存在 | 全仓库无该类型 | 符合隔离目标，但接口缺失 |
| 不在 `AxisCapabilities` | 当前 backend capability 无该字段 | `host/motion/capabilities.py:8-19` | 是 |
| 不在 `AxisState` | 统一类型不存在；底层状态无该字段 | 三类底层 state 定义 | 是 |
| 不在 `ArmJointState` | 类型不存在 | 全仓库无该类型 | 符合隔离目标，但接口缺失 |
| 不在 `ArmJointTarget` | 类型不存在 | 全仓库无该类型 | 符合隔离目标，但接口缺失 |
| 不进入 FK/IK | 无任何引用或偏移 | `host/kinematics/planar_2r.py:37-128` | 是 |
| 不进入电机目标换算 | 换算仅使用 logical target、zero、direction、ratio/current snapshot | `host/robot/joint.py:396-445`；`host/robot/feetech_rotation.py:112-124` | 是 |
| 不作为软限位基准 | limits 直接属于逻辑坐标 | `host/config/joints.py:20-45`；`host/config/feetech.py:75-83` | 是 |
| 不作为普通前端命令 | 无前端统一接口，也无 `move_to_work_zero()` | 全仓库关键词检索 | 是 |

结论：**没有发现工作零点、逻辑零点和机械归零在当前 Host 代码中混用。** 机械归零只在 STM32 Slide/Z；肩肘与 Rotation 使用绝对位置 + 标定逻辑零点；工作初始位置尚未实现。

### 7.3 系统初始化边界

当前可供未来初始化协调器复用的基础动作：

- Slide/Z：home、query、absolute move、event wait；
- Shoulder/Elbow：稳定绝对位置初始化、query、absolute move、software stop；
- Rotation：read position/feedback、torque enable/disable、absolute move。

仍缺：

- `SystemStartupConfig`、`StartupAxisTarget`；
- 初始化状态和初始化完成门禁；
- 经机械验证的安全顺序；
- Slide/Z home 后状态复核；
- 逐步失败时统一停止/失能政策；
- 进入正常运行后禁止重复使用 startup config 的测试。

初始化顺序属于系统协调器，因为它要处理轴间干涉、后端状态和失败恢复；放入运动学会污染纯几何算法并造成 startup position 成为错误坐标偏移。

## 8. State, Completion, and Error Semantics

### 8.1 状态字段表

| State field | Slide/Z | Shoulder/Elbow | Rotation | Can be unified now? |
| --- | --- | --- | --- | --- |
| `connected` | 无字段；成功 query 可推断本次通信成功 | 无字段；CAN query 成功可推断 | 无字段；serial query 成功可推断 | Partial；只能表达本次调用，不是持续连接监控 |
| `enabled` | 明确 bool | `motor_state` raw，可按已知 `0x00` 判断 enabled；其他值语义有限 | 当前未读取 torque enable 状态 | 是；Rotation 必须为 `None` |
| `busy` | 明确 bool | 由 motor speed 和阈值计算 `moving` | feedback 有 raw `moving` | Partial；Rotation 字段和阈值需实机复验 |
| `homed` | 明确 bool | 不适用 | 不适用 | 是；旋转轴必须为 `None`，不能为 false |
| `position_valid` | 明确 bool | 初始化并唯一映射后 true | 无显式字段；成功唯一映射后可视为 true | Partial；断线/解析失败必须 false |
| `current_position` | integer µm | logical rad | logical rad | 是，经 adapter 转为 mm/deg |
| `target_position` | 不保存 | 不保存 | 命令只返回 raw target，不保存 | 是，但当前只能为 `None` 或由 controller 跟踪 |
| `position_unit` | 隐含 µm | 隐含 rad | 隐含 rad | 是，由 descriptor/adapter 固定为 mm/deg |
| `faulted` | `fault` integer | `error_state` integer | `error_raw` | Partial；需要稳定映射和硬件语义验证 |
| `fault_code` | 稳定 firmware code/axis fault | raw MG error byte | raw Feetech byte | Partial；可保留 raw code，但需统一分类 |
| `fault_message` | 异常文本可构造 | 异常文本可构造 | 异常文本可构造 | 是，由 adapter 生成 |

一个具体的 `None` 风险：`CanRotaryJoint._compose_state()` 在没有 status 时把 `moving` 填成 `False`（`host/robot/joint.py:493-519`）。`initialize()` 正是无 status 调用该函数（`host/robot/joint.py:287-297`）。因此统一 adapter 不应把初始化返回快照直接映射为 `busy=False`；应重新 `get_state()`，或把底层 `moving` 改为可空类型。

### 8.2 accepted 与 completed

#### STM32

底层协议正确区分 `OK` 和终态，但 Host `move_absolute()`、`move_relative()`、`home()` 都调用 `_nonblocking()` 后**同步等待终态**（`host/drivers/stm32_motion.py:309-323,372-417`）。因此：

- 当前能实现 `wait=True`；
- 当前无法通过公开 API 安全实现 `wait=False` 的“只返回 accepted”；
- adapter 不应调用私有 `_send()`/`_wait()`，否则会把协议生命周期和 pending queue 细节泄漏到统一层。

建议让 `STM32MotionClient` 新增公开的提交句柄/sequence API 与 `wait_for_event()`，或让 motion command 接受 `wait` 并返回明确的 submission/event 对象。

#### Shoulder / Elbow

`MG4010Driver.command_position()` 只确认 `0xA4` 通信应答，不等机械到位（`host/drivers/mg4010_driver.py:89-99`）；`CanRotaryJoint.command_position()` 注释和测试也明确非阻塞（`host/robot/joint.py:349-354`；`host/tests/test_joint.py:457-463`）。统一结果应为：

```text
accepted=True
completed=None
```

若目标已在 tolerance 内且没有发送 `0xA4`，可以返回 `completed=True`，但现有返回类型未显式告诉 adapter 是否实际发送。最小实现可保守返回 `completed=None`；若要准确表达 no-op 完成，应让关节层返回带 `command_sent` 的结果。

#### Rotation

`command_position()` 等待 write status（取决于 `expect_write_status`）后返回 `target_raw`，没有到位等待（`host/robot/feetech_rotation.py:203-233`）。成功写入最多表示命令被通信层接受，统一结果必须是 `accepted=True, completed=None`。torque disable 不能被包装成 stop 或 completed move。

#### 现有双关节桥

`Planar2RArmController` 没有把命令发送成功描述成机械到位；CLI 明确打印“accepted; mechanical arrival ... not implied”（`host/scripts/test_planar_2r_motion.py:167-175`）。但异常文案 `dual-joint position submission did not complete`（`host/robot/planar_arm.py:201-203`）中的 `complete` 容易与运动完成混淆，建议后续改成 `submission failed`。

### 8.3 当前异常与统一映射建议

| 当前异常来源 | 已有区分 | 建议统一类别 |
| --- | --- | --- |
| STM32 configuration/protocol | 参数、帧格式 | `INVALID_REQUEST` / `BACKEND_ERROR` |
| STM32 sync error code | busy/not ready/not homed/soft limit/fault/unsupported 等 raw code | 一对一映射 `BUSY`、`NOT_HOMED`、`SOFT_LIMIT`、`DEVICE_FAULT`、`UNSUPPORTED_COMMAND` 等 |
| STM32 timeout | 同步或 event timeout | `TIMEOUT` |
| STM32 ABORT/FAULT event | event kind + arguments | `INVALID_STATE`/`DEVICE_FAULT`，`completed=False` |
| CAN not open/timeout/frame | 未打开、超时、帧错误 | `BACKEND_UNAVAILABLE` / `TIMEOUT` / `COMMUNICATION_ERROR` |
| MG command result unknown | A4 可能已接收，已尽力 stop | `COMMUNICATION_ERROR`，accepted/completed 均不能伪造；message 保留 unknown |
| Joint init/position/limit/moving/fault/disabled | 已有独立异常类 | `INITIALIZATION_REQUIRED`、`POSITION_INVALID`、`SOFT_LIMIT`、`BUSY`、`DEVICE_FAULT`、`INVALID_STATE` |
| Feetech not open/timeout/protocol/device | 已有独立异常类 | `BACKEND_UNAVAILABLE`、`TIMEOUT`、`COMMUNICATION_ERROR`、`DEVICE_FAULT` |
| Feetech rotation limit/position | 限位和无法唯一解释 | `SOFT_LIMIT` / `POSITION_INVALID` |

当前库异常通常向上抛出，没有在核心库中静默吞掉。CLI 在顶层捕获后记录并返回 2，这是工具行为，不是统一错误结果。统一层应集中映射，不应在每个调用点复制 `except` 表。

## 9. Kinematics Integration Readiness

### 9.1 当前可以稳定依赖的内容

运动学开发者现在可以依赖：

- `Planar2RKinematics` 的纯数学 FK/IK、可达性和奇异点行为；
- 坐标约定：x 向前、y 向左、z 向上；肩角相对全局 +x，肘角相对 link1；
- `joint_limited_solutions()` / `select_joint_target()` 对当前肩肘逻辑软限位的筛选行为；
- 肩肘角是逻辑角，不是 motor degree；
- Rotation 不参与 Planar 2R 的 XY 逆解；
- 工作初始位置不进入运动学。

运动学开发者目前**不能稳定依赖**：

- `AxisName`、`ArmJointState`、`ArmJointTarget`、`KinematicsMotionInterface` 的实际 Python 类型，因为它们尚不存在；
- `Planar2RArmController` 作为最终共享接口，因为它暴露 rad、具体 `JointConfig` 和底层 `JointState`；
- 真实 `L1/L2`、最终关节限位、到位/超时、严格同步或系统初始化完成状态。

### 9.2 可以立即冻结的名称和语义

可以冻结：

- 轴 ID：`slide`、`z`、`shoulder`、`elbow`、`rotation`；
- 统一公开位置/速度/加速度：linear 为 mm/mm/s/mm/s²，rotary 为 deg/deg/s/deg/s²；
- 肩、肘和 Rotation 的角度都是逻辑角；
- 逻辑零点、方向、减速比、encoder count 由控制后端处理；
- `startup_position` 只供初始化协调器使用，不是角度或位置偏移；
- Rotation 与 Planar 2R XY 解耦；
- `accepted=True, completed=None` 只表示接受；
- 第一版不承诺严格同步、连续轨迹或硬件急停。

暂时不能冻结：

- 各轴最终软限位、默认速度/加速度和 startup target；
- Shoulder/Elbow 的独立 disable/enable 语义；
- Rotation 工程速度/加速度映射、torque enabled 查询和 fault 位解释；
- Shoulder/Elbow/Rotation 的 wait/arrival contract；
- Slide/Z 是否需要 machine zero 之外的线性逻辑坐标偏移；
- 真实 `L1/L2` 和上电安全顺序。

### 9.3 并行开发建议

可以并行开展运动学。最小前置不是完整硬件 controller，而是先提供：

1. `AxisName`、`ArmJointState`、`ArmJointTarget`、`MotionResult` 的数据类型；
2. `KinematicsMotionInterface` Protocol；
3. 零硬件 I/O 的 fake，实现 `accepted=True, completed=None`；
4. 单元测试锁定 deg 边界和 startup isolation。

运动学算法内部可以继续使用 rad；在 `ArmJointTarget` 边界进行一次 deg↔rad 转换即可，不需要重写 FK/IK。

## 10. Missing Interfaces

### 10.1 必要文件

| 建议文件 | 是否必要 | 内容 |
| --- | --- | --- |
| `host/motion/unified_protocol.py` | 是 | `AxisName`、descriptor/capability/state/result/error、SingleAxis/Kinematics Protocol、Arm DTO |
| `host/motion/unified_controller.py` | 是 | 依赖注入、轴分发、adapter、单位/状态/错误/结果映射 |
| `host/motion/system_startup.py` | P3 必要 | 启动目标、初始化状态、协调器；严格隔离 startup position |
| `host/tests/test_unified_protocol.py` | 是 | 字段、枚举、单位、startup 禁止字段静态边界 |
| `host/tests/test_unified_controller.py` | 是 | 五轴 fake、能力、转换、错误和 accepted/completed 矩阵 |
| `host/tests/test_system_startup.py` | P3 必要 | 顺序、home 门禁、失败停止、初始化后隔离 |

### 10.2 可避免的过度设计

- 第一版不必为 STM32、MG4010、Feetech 各建一个公开 adapter 文件；可把三个小型私有 adapter 放在 `unified_controller.py`，等行为变复杂后再拆。
- `AxisMotionConfig` 可先由 `AxisDescriptor` + 现有后端 config 生成，避免新建一套重复配置真值。
- `SystemStartupConfig` 必须与普通轴配置分离，可定义在 `system_startup.py`；不建议放进 `unified_protocol.py` 的普通控制 DTO 区域。
- 依赖注入足以完成分发：控制器接收一个 STM32 client、两个 `CanRotaryJoint` 和一个 `FeetechRotationAxis`；构造不得创建或打开硬件 transport。

### 10.3 现有文件的最小适配

- `host/motion/capabilities.py`：应迁移或改为生成协议 `AxisCapabilities`；不要保留两份独立真值。可保留 `UpperMotionBackends` 作为依赖容器，但应移除 vacuum 对统一位置轴枚举的干扰。
- `host/drivers/stm32_motion.py`：需要公开拆分“提交/接受”和“等待 event”，否则 `wait=False` 无法正确实现；保留现有解析、pending queue 和命令方法核心。
- `host/robot/joint.py`：不应重写角度解析和 A4 目标逻辑。可选择让 `JointState.moving` 支持 `None`，以及返回更明确的 command submission 信息。
- `host/robot/feetech_rotation.py`：保留位置换算和寄存器写入；如果后续确认 torque 状态/工程速度映射，再增加只读能力，不要在 adapter 猜测。
- `host/robot/planar_arm.py`：应保留。它已有软限位筛解和部分下发失败 stop 的价值，可作为现有 CLI/底层桥；新上层运动学入口通过统一 Protocol 实现，后续再让该文件适配或瘦身。
- `host/kinematics/planar_2r.py`：算法无需修改；只需在新的共享边界做 deg↔rad。
- `host/config/joints.py`、`host/config/feetech.py`：保留现有后端标定职责，不加入 startup position。

## 11. Risks and Unverified Assumptions

### 11.1 已确认风险

1. **单位泄漏**：当前前端若直接接后端，必须自行处理 µm/rad/raw，容易产生 1000 倍或角度制错误。
2. **STM32 wait 语义不匹配**：现有公开运动方法只能等待最终 event，无法正确实现默认 `wait=False`。
3. **stop/disable 混名**：肩肘的 `0x81` 同时被能力表当 stop 和 disable，协议层必须先冻结一种保守语义。
4. **未知状态被压成 false**：关节初始化状态的 `moving=False` 可能只是未读取速度，不是确认空闲。
5. **缺少结构化结果**：后端返回 event、旧状态或 raw target，前端无法统一显示接受、完成和失败。
6. **无初始化门禁**：当前任何具有后端引用的代码都可绕过系统初始化状态直接调用命令。
7. **工作树 dirty**：协议、Feetech 配置和测试包含未提交内容，复现性取决于当前工作树而非单一 commit。

### 11.2 必须等待硬件或标定确认

- Slide/Z 最终全行程、软限位、方向和 homing 重复性；
- Shoulder/Elbow 零点、方向、限位、速度的独立校准记录与机械复验；
- MG4010 `motor_state`、`0x81` 是否能被业务定义为独立 disable，以及长时间双机通信；
- Rotation 已记录的 `direction_sign=+1`/`zero_raw=2130` 的独立原始标定与重复性复验、最终限位、负载安全速度、raw acceleration 的物理含义、write status、feedback/error 位和 torque state 读取；
- 实际 `L1/L2`、安装偏移、碰撞约束和 IK 分支连续性；
- 各轴 startup position 和机械安全初始化顺序；
- 软件 stop 后位置是否仍可安全视为有效，以及跨后端故障时的系统停止政策。

### 11.3 协议与底层能力冲突/歧义

- 协议公开 acceleration，但 MG4010/Rotation 没有已验证的 deg/s² 能力；必须明确非空参数的错误行为。
- 协议公开 `wait=False`，STM32 client 当前只提供阻塞到终态的方法；需要扩展客户端公开边界。
- 协议的 `AxisCapabilities` 没有区分可配置 velocity/acceleration，也没有表达 enabled 可查询与可命令的差异。
- 协议希望统一 `disable()`，但 MG4010 当前只有 `0x81` software stop；不能为了填表强行声称存在独立 disable。
- 协议提到 Slide/Z “逻辑工作位置”，但没有定义 machine zero 与逻辑 zero 的偏移配置；第一版应明确二者相同，或另行定义坐标变换，绝不能借用 startup position。
- `MotionResult` 对 unsupported/validation failure 时 `completed` 应为 `False` 还是 `None` 尚未明文定义；建议冻结：未接受时 `accepted=False, completed=False`，已接受但未知到位为 `None`。
- 协议 §20 仍把 Rotation 正方向写为“待 Feetech 标定”，而当前工作树配置和进度证据已记录 `direction_sign=+1`、逻辑正方向 `+X`、`zero_raw=2130`（`host/config/feetech.py:75-85`；`docs/progress/CURRENT_STATUS.md:168-172`）。实现前应同步双方认可的配置状态，但本轮不修改协议。

## 12. Minimal Implementation Plan

### P0：冻结接口类型和单位

- 修改目标：定义统一枚举、DTO、错误码和 Protocol；冻结 deg/mm 与 accepted/completed 语义。
- 涉及文件：新增 `host/motion/unified_protocol.py`、`host/tests/test_unified_protocol.py`；更新 `host/motion/__init__.py`。
- 是否阻塞运动学：**是，短期唯一阻塞项**；完成后运动学可用 fake 并行开发。
- 验收条件：五轴名称固定；普通 DTO 无 startup 字段；rotary 公共角度全为 deg；Protocol 可由 fake 实现；静态边界测试通过。
- 不能声称：真实后端已接入、运动完成可确认、系统已初始化。

### P1：实现统一单轴分发

- 修改目标：实现 list/describe/get_state/command dispatcher、capability 拒绝和依赖注入。
- 涉及文件：新增 `host/motion/unified_controller.py`、`host/tests/test_unified_controller.py`；适配 `host/motion/capabilities.py`。
- 是否阻塞运动学：不阻塞；运动学可继续使用 Protocol/fake。
- 验收条件：纯 fake 下五轴分发正确；未知轴/unsupported/limit/invalid 参数返回稳定错误；构造零 I/O。
- 不能声称：真实 transport、机械方向、限位或到位已验证。

### P2：接入现有三个后端

- 修改目标：实现 mm↔µm、deg↔rad、状态/异常/结果映射；为 STM32 增加公开 submit/wait 边界。
- 涉及文件：`host/drivers/stm32_motion.py`、`host/motion/unified_controller.py`、`host/motion/capabilities.py` 和对应 tests；现有 joint/rotation 文件仅做必要小改。
- 是否阻塞运动学：不阻塞。
- 验收条件：所有转换边界有精确测试；STM32 wait true/false 都不混淆；肩肘/Rotation 成功命令返回 `completed=None`；不支持的 acceleration 不被忽略。
- 不能声称：肩肘/Rotation 到位、严格同步、硬件急停或最终机械参数完成。

### P3：增加系统上电初始化边界

- 修改目标：独立 startup config、状态与协调器；Slide/Z home 门禁；安全顺序配置化。
- 涉及文件：新增 `host/motion/system_startup.py`、`host/tests/test_system_startup.py`；必要时新增独立 config 文件。
- 是否阻塞运动学：不阻塞算法；只阻塞真实运动学任务进入运行态。
- 验收条件：fake 流程验证 home→valid→startup targets；任一步失败有明确状态；正常 DTO/FK/IK/目标换算不引用 startup config。
- 不能声称：安全顺序已机械验收、初始化失败自动恢复完成。

### P4：补充到位等待与故障传播

- 修改目标：肩肘/Rotation stable window、deadline、timeout stop、逐轴结果和跨轴 best-effort stop。
- 涉及文件：优先 `host/motion/` 和 tests；必要时给 `joint.py`/`feetech_rotation.py` 增加只读状态能力。
- 是否阻塞运动学：不阻塞规划；阻塞 `wait=True` 和真实闭环任务。
- 验收条件：fake 故障矩阵覆盖 accepted、arrival、abort/fault、timeout、stop failure；低速台架验证后才能开启真实 wait。
- 不能声称：严格同步、硬件急停、连续轨迹。

### P5：真实硬件验证

- 修改目标：完成标定、逐轴状态/限位/超时/停止和分阶段初始化验证。
- 涉及文件：`docs/calibration/`、配置、硬件测试记录；仅在证据要求下调整实现。
- 是否阻塞运动学：不阻塞纯算法；阻塞生产参数和整机声明。
- 验收条件：每轴有可复现原始记录；断线/故障/超限/停止场景；初始化顺序在安全机械区域验证。
- 不能声称：完整采摘状态机、碰撞安全或系统验证，除非另有整机证据。

## 13. Interfaces Safe to Freeze

### 13.1 建议立即冻结

```text
AxisName = slide | z | shoulder | elbow | rotation

Linear public units:
  position mm
  velocity mm/s
  acceleration mm/s²

Rotary public units:
  position deg
  velocity deg/s
  acceleration deg/s²

Logical angle:
  zero/direction/gear/raw conversion belong below unified interface

Startup position:
  system initialization only; never a normal coordinate offset

Completion:
  accepted != completed
  accepted=True, completed=None means command accepted only
```

可以同步冻结 Protocol 的方法名称：

```text
list_axes
describe_axis
get_state
move_absolute
move_relative
stop
enable
disable
home_reference
clear_fault
get_arm_joint_state
command_arm_joint_target
```

### 13.2 暂不承诺

```text
Shoulder/Elbow enable or independent disable
Shoulder/Elbow/Rotation wait_for_completion
Rotation stop
Rotation physical velocity/acceleration mapping
final axis limits/default profiles
actual L1/L2
startup positions and safe order
strict synchronization
hardware emergency stop
```

## 14. Final Recommendation

1. 按 **B 类**推进：保留三类现有后端，实现一个薄的统一中间层；不要重写已测试的 STM32 parser、MG4010 joint conversion、Feetech position conversion 或 Planar 2R 数学算法。
2. 先完成 P0 的 Protocol/fake，让运动学团队立即并行；运动学只依赖逻辑角、deg DTO 和统一结果，不直接依赖 CAN/串口对象。
3. P1/P2 采用依赖注入和小型 adapter；`capabilities.py` 成为协议能力真值或由描述符生成，避免重复配置漂移。
4. 优先修正 STM32 submit/wait 边界和肩肘 stop/disable 语义；在此之前不向前端承诺统一完成状态或独立 disable。
5. P3 单独建立系统初始化层。`startup_position` 只能在该层出现，初始化完成后不得继续参与 FK/IK、轴描述、状态、软限位或目标换算。
6. 真实硬件参数仍未充分确认；统一层离线测试通过后，只能声称“接口实现/离线测试”，不能声称“机械验证/系统验证”。

最终判断：当前底层基础足以支撑协议实现和运动学并行开发，但**当前仓库还不能直接作为协议 v0.2 的前端/运动学稳定接口交付**。最小正确路径是先冻结统一类型与 fake，再薄适配现有后端，最后独立实现系统初始化和硬件到位验证。
