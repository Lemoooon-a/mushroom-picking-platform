# 上层统一运动控制接口协议

> 文档状态：Draft v0.2  
> 适用范围：Host 上层运动控制、前端单轴控制、平面二连杆运动学调用  
> 更新日期：2026-08-03  
> 关键修订：严格限制“工作零点”只用于上电初始化，不参与后续运动学、坐标换算或常规控制。

---

## 1. 目的

本协议用于统一采蘑菇平台各运动轴的上层调用方式，使前端、运动学模块和后续任务协调器不需要直接了解 STM32 串口协议、MG4010E CAN 协议或 Feetech 舵机寄存器。

本协议规定的是 Host 内部的软件接口和语义，不新增串口、CAN、TCP、WebSocket 或 ROS 2 通信协议。

当前纳入统一控制的运动轴如下：

| Axis ID | 机械轴 | 类型 | 上层位置单位 | 当前底层 |
| --- | --- | --- | --- | --- |
| `slide` | 水平滑轨 | Linear | mm | STM32 |
| `z` | Z 轴 | Linear | mm | STM32 |
| `shoulder` | 肩关节 | Rotary | deg | MG4010E |
| `elbow` | 肘关节 | Rotary | deg | MG4010E |
| `rotation` | 末端旋转轴 | Rotary | deg | Feetech |

吸盘不属于位置轴，不纳入本协议，应保留为独立执行器接口。

---

## 2. 分层关系

```text
Frontend / Kinematics / Task Coordinator
                    |
                    v
        UnifiedMotionController
                    |
        +-----------+-----------+
        |           |           |
        v           v           v
 STM32 Axis     CAN Joint   Feetech Axis
  Adapter        Adapter       Adapter
        |           |           |
        v           v           v
 STM32Motion   CanRotary    FeetechRotation
   Client        Joint          Axis
```

统一控制层负责：

1. 统一轴名称；
2. 统一工程单位；
3. 统一命令入口；
4. 统一状态与错误表达；
5. 根据轴类型调用正确的底层接口；
6. 将运动学输出的逻辑关节角发送给肩、肘和末端旋转轴；
7. 为系统上电初始化流程提供必要的单轴动作能力。

统一控制层不负责：

1. 逆运动学求解；
2. 连续轨迹规划；
3. 严格多轴同步；
4. 碰撞检测；
5. 抓取状态机；
6. 自动决定机械安全初始化顺序；
7. 修改底层电机协议；
8. 将软件停止描述为硬件急停。

---

## 3. 稳定轴标识

```python
from enum import Enum


class AxisName(str, Enum):
    SLIDE = "slide"
    Z = "z"
    SHOULDER = "shoulder"
    ELBOW = "elbow"
    ROTATION = "rotation"
```

轴名称是前端、运动学和控制层之间的稳定标识。

上层不得使用以下信息代替轴名称：

- STM32 协议中的 `S` 或 `Z`；
- CAN ID；
- Feetech 舵机 ID；
- 电机型号；
- 串口设备路径。

---

## 4. 坐标、零点与单位约定

### 4.1 统一工程单位

统一接口不得向上层暴露脉冲数、编码器计数、减速器输入角度或电机原始角度。

| 轴类型 | 位置 | 速度 | 加速度 |
| --- | --- | --- | --- |
| Linear axis | mm | mm/s | mm/s² |
| Rotary axis | deg | deg/s | deg/s² |

底层单位转换由对应适配器完成：

```text
Slide / Z:
  mm -> integer µm
  mm/s -> integer µm/s
  mm/s² -> integer µm/s²

Shoulder / Elbow:
  logical deg -> rad -> CanRotaryJoint

Rotation:
  logical deg -> rad -> FeetechRotationAxis -> raw count
```

前端和运动学模块不得直接处理：

- `µm`；
- `step`；
- motor degree；
- raw encoder count；
- gear ratio；
- protocol register value。

### 4.2 逻辑零点

逻辑零点是正常工作坐标系中数值为 `0` 的位置。

对于肩、肘和末端旋转轴：

- 上层调用使用逻辑角度；
- 角度正方向由轴配置规定；
- 逻辑零点由轴标定配置规定；
- 软限位相对于同一套逻辑坐标定义；
- 控制层负责将逻辑角度转换为电机原始位置。

对于 Slide 和 Z：

- 机械归零后建立底层位置参考；
- 控制层再将底层 machine position 表达为统一的逻辑工作位置；
- 上层只使用 mm，不使用 STM32 的脉冲位置。

### 4.3 机械归零

机械归零用于建立位置参考和位置有效性。

只有以下轴支持机械归零：

```text
slide
z
```

统一接口名称：

```python
home_reference(axis)
```

成功后应满足：

```text
homed = True
position_valid = True
```

以下轴不提供机械归零：

```text
shoulder
elbow
rotation
```

肩、肘和末端旋转轴通过绝对位置读取、逻辑零点和软限位解释当前角度。

对这些轴调用 `home_reference()` 必须返回：

```text
UNSUPPORTED_COMMAND
```

不得使用当前位置重新设置逻辑零点。

---

## 5. 工作零点的严格定义与限制

### 5.1 定义

本项目中的“工作零点”只表示：

> 系统上电后，各轴完成必要的位置初始化后，需要移动到的预设工作初始位置。

为避免与逻辑零点混淆，代码和配置中推荐使用：

```text
startup_position
```

而不是在常规运动接口中使用 `work_zero_position`。

文档中保留“工作零点”这一机械含义，但软件语义统一解释为：

```text
startup work position
```

### 5.2 工作零点不是真正的坐标零点

工作零点：

- 不一定等于数值 `0`；
- 不等于逻辑零点；
- 不等于机械归零位置；
- 不改变坐标系；
- 不重置当前编码器位置；
- 不重新定义关节角；
- 不建立新的运动学参考系。

示例：

```text
肩关节逻辑零点：0 deg
肩关节逻辑限位：-60 deg ~ +70 deg
肩关节上电工作初始位置：+20 deg
```

上电初始化时肩关节移动到 `+20 deg`。

初始化完成后：

- 肩关节当前位置仍然表示为 `+20 deg`；
- 后续目标 `0 deg` 仍表示逻辑零点；
- 运动学不会计算 `target - 20 deg`；
- 正运动学和逆运动学不会读取 `startup_position`；
- `+20 deg` 不会成为新的角度原点。

### 5.3 强制约束

工作零点只能用于系统上电初始化流程。

必须满足以下约束：

1. `startup_position` 只存在于系统初始化配置中；
2. 不放入普通 `AxisDescriptor`；
3. 不作为常规单轴能力；
4. 不提供给运动学模块；
5. 不参与正运动学；
6. 不参与逆运动学；
7. 不参与逻辑角度与电机角度换算；
8. 不作为软限位计算基准；
9. 不用于当前位置归一化；
10. 不作为后续控制目标的默认偏移；
11. 不允许普通前端在运行阶段反复调用“回工作零点”；
12. 系统初始化完成后，常规控制只使用逻辑工作坐标。

禁止出现以下计算：

```python
kinematics_angle = logical_angle - startup_position
```

禁止出现以下计算：

```python
motor_target = requested_position + startup_position
```

正确关系是：

```text
运动学输出逻辑角度
        |
        v
统一控制层按逻辑零点、方向和减速比转换
        |
        v
底层电机目标
```

`startup_position` 不在该转换链路中。

### 5.4 工作零点的配置位置

工作零点应放在单独的系统初始化配置中：

```python
from dataclasses import dataclass


@dataclass(frozen=True)
class StartupAxisTarget:
    axis: AxisName
    position: float
    velocity: float | None = None
    acceleration: float | None = None


@dataclass(frozen=True)
class SystemStartupConfig:
    targets: tuple[StartupAxisTarget, ...]
```

示例：

```python
SystemStartupConfig(
    targets=(
        StartupAxisTarget(AxisName.SLIDE, position=...),
        StartupAxisTarget(AxisName.Z, position=...),
        StartupAxisTarget(AxisName.SHOULDER, position=...),
        StartupAxisTarget(AxisName.ELBOW, position=...),
        StartupAxisTarget(AxisName.ROTATION, position=...),
    )
)
```

这些值由系统初始化协调器读取。

普通单轴控制接口和运动学接口不得直接依赖 `SystemStartupConfig`。

---

## 6. 轴描述信息

```python
from dataclasses import dataclass


@dataclass(frozen=True)
class AxisDescriptor:
    name: AxisName
    display_name: str
    kind: str

    position_unit: str
    velocity_unit: str
    acceleration_unit: str

    minimum_position: float
    maximum_position: float

    capabilities: "AxisCapabilities"
```

推荐值：

```text
kind:
  "linear"
  "rotary"

position_unit:
  "mm"
  "deg"
```

注意：

```text
AxisDescriptor 中不得包含 startup_position。
```

原因是工作零点不是常规坐标属性，也不是运动学参数。

前端可以通过 `AxisDescriptor` 获取：

- 轴类型；
- 显示单位；
- 逻辑软限位；
- 支持的控制能力。

前端不能通过 `AxisDescriptor` 获取或修改上电工作初始位置。

---

## 7. 轴能力定义

```python
from dataclasses import dataclass


@dataclass(frozen=True)
class AxisCapabilities:
    query_state: bool
    move_absolute: bool
    move_relative: bool
    stop: bool
    enable: bool
    disable: bool
    reference_home: bool
    clear_fault: bool
    wait_for_completion: bool
```

`AxisCapabilities` 中不得包含：

```text
move_to_work_zero
```

因为工作零点不是常规运行阶段的轴操作。

第一版预期能力如下，最终实现必须以真实底层 API 为准：

| Axis | Query | Absolute | Relative | Stop | Enable | Disable | Reference Home | Wait |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `slide` | Yes | Yes | Yes | Yes | Yes | Yes | Yes | Yes |
| `z` | Yes | Yes | Yes | Yes | Yes | Yes | Yes | Yes |
| `shoulder` | Yes | Yes | No | Yes | 按实际接口 | 按实际语义 | No | No |
| `elbow` | Yes | Yes | No | Yes | 按实际接口 | 按实际语义 | No | No |
| `rotation` | Yes | Yes | No | No | Yes | Yes | No | No |

注意：

- `shoulder` 和 `elbow` 的软件停止不是硬件急停；
- `rotation` 的 torque disable 不能伪装成 stop；
- 不支持的操作必须明确返回错误；
- `wait_for_completion=False` 表示当前不能可靠确认到位，不代表不能下发运动命令。

---

## 8. 通用轴状态

```python
from dataclasses import dataclass


@dataclass(frozen=True)
class AxisState:
    axis: AxisName

    connected: bool
    enabled: bool | None
    busy: bool | None

    homed: bool | None
    position_valid: bool

    current_position: float | None
    target_position: float | None
    position_unit: str

    faulted: bool
    fault_code: str | int | None
    fault_message: str | None
```

字段语义如下。

### 8.1 `connected`

表示当前后端是否可以正常通信。

### 8.2 `enabled`

- `True`：底层确认已使能；
- `False`：底层确认未使能；
- `None`：当前后端无法可靠查询。

### 8.3 `busy`

- `True`：确认正在运动；
- `False`：确认空闲；
- `None`：底层无法提供可靠状态。

### 8.4 `homed`

- `slide`、`z`：机械归零是否成功；
- `shoulder`、`elbow`、`rotation`：固定为 `None`。

不适用不得表示为 `False`。

### 8.5 `position_valid`

表示当前位置是否可以作为控制和运动学输入使用。

- `slide`、`z`：通常机械归零后才有效；
- 旋转轴：完成初始化并成功读取绝对位置后可以有效；
- 断线、位置解释失败或相关故障时应为 `False`。

### 8.6 `current_position`

统一使用逻辑工作坐标：

```text
slide / z: mm
shoulder / elbow / rotation: deg
```

`AxisState` 不包含 `startup_position`，也不提供“相对工作零点的位置”。

---

## 9. 错误码与结果

### 9.1 错误码

```python
class MotionErrorCode(str, Enum):
    INVALID_REQUEST = "invalid_request"
    UNKNOWN_AXIS = "unknown_axis"
    BACKEND_UNAVAILABLE = "backend_unavailable"
    UNSUPPORTED_COMMAND = "unsupported_command"

    INVALID_STATE = "invalid_state"
    NOT_HOMED = "not_homed"
    POSITION_INVALID = "position_invalid"
    SOFT_LIMIT = "soft_limit"
    BUSY = "busy"

    INITIALIZATION_REQUIRED = "initialization_required"
    INITIALIZATION_FAILED = "initialization_failed"

    TIMEOUT = "timeout"
    DEVICE_FAULT = "device_fault"
    COMMUNICATION_ERROR = "communication_error"
    BACKEND_ERROR = "backend_error"
```

### 9.2 命令结果

```python
@dataclass(frozen=True)
class MotionResult:
    ok: bool
    axis: AxisName
    command: str

    accepted: bool
    completed: bool | None

    final_state: AxisState | None

    error_code: MotionErrorCode | None
    message: str
```

### 9.3 `accepted`

表示底层是否接受该命令。

### 9.4 `completed`

- `True`：已可靠确认完成；
- `False`：已确认中止、故障或失败；
- `None`：命令已发送，但当前后端没有可靠的到位确认能力。

禁止将“命令已经下发”表示成“运动已经完成”。

---

## 10. 单轴统一控制接口

```python
from typing import Protocol


class SingleAxisMotionInterface(Protocol):
    def list_axes(self) -> tuple[AxisDescriptor, ...]:
        ...

    def describe_axis(self, axis: AxisName) -> AxisDescriptor:
        ...

    def get_state(self, axis: AxisName) -> AxisState:
        ...

    def move_absolute(
        self,
        axis: AxisName,
        position: float,
        *,
        velocity: float | None = None,
        acceleration: float | None = None,
        wait: bool = False,
        timeout_s: float | None = None,
    ) -> MotionResult:
        ...

    def move_relative(
        self,
        axis: AxisName,
        distance: float,
        *,
        velocity: float | None = None,
        acceleration: float | None = None,
        wait: bool = False,
        timeout_s: float | None = None,
    ) -> MotionResult:
        ...

    def stop(self, axis: AxisName) -> MotionResult:
        ...

    def enable(self, axis: AxisName) -> MotionResult:
        ...

    def disable(self, axis: AxisName) -> MotionResult:
        ...

    def home_reference(
        self,
        axis: AxisName,
        *,
        wait: bool = True,
        timeout_s: float | None = None,
    ) -> MotionResult:
        ...

    def clear_fault(self, axis: AxisName) -> MotionResult:
        ...
```

该接口中明确不包含：

```python
move_to_work_zero(...)
```

原因：

- 工作零点仅属于系统上电初始化；
- 不能作为普通前端命令；
- 不能作为运动学调用；
- 不能被误认为运行中的标准零位操作。

---

## 11. 单轴接口调用规则

### 11.1 `move_absolute()`

所有轴的主要位置控制入口。

```python
move_absolute(
    axis=AxisName.SHOULDER,
    position=20.0,
    velocity=5.0,
)
```

规则：

- `position` 是逻辑工作位置；
- 旋转轴单位为 deg；
- 直线轴单位为 mm；
- 必须检查逻辑软限位；
- 不得绕过现有 joint/axis 层直接写电机原始位置；
- 不得自动使能；
- 不得自动机械归零；
- 不得静默修改目标；
- 不得附加 `startup_position` 偏移。

### 11.2 `move_relative()`

第一版主要用于：

```text
slide
z
```

对于不支持相对运动的轴返回：

```text
UNSUPPORTED_COMMAND
```

运动学模块不得依赖相对运动。

### 11.3 `home_reference()`

只允许：

```text
slide
z
```

肩、肘和末端旋转轴调用时必须失败，且不能使用当前位置重新设置为零。

### 11.4 `stop()`

- `slide`、`z`：映射 STM32 stop；
- `shoulder`、`elbow`：映射 MG4010E software stop；
- `rotation`：第一版返回不支持，除非后续确认可靠的独立停止语义；
- stop 不等价于 emergency stop；
- stop 不自动清除故障。

---

## 12. 系统上电初始化接口

工作零点只由单独的系统初始化接口使用。

```python
from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class SystemInitializationState:
    started: bool
    completed: bool
    failed: bool
    current_step: str | None
    message: str


class SystemInitializationInterface(Protocol):
    def initialize_to_work_ready(
        self,
        config: SystemStartupConfig,
        *,
        timeout_s: float | None = None,
    ) -> SystemInitializationState:
        ...

    def get_initialization_state(self) -> SystemInitializationState:
        ...
```

### 12.1 初始化流程

推荐系统级初始化流程：

```text
1. 创建并连接各底层后端
2. 查询所有轴状态
3. Slide 执行机械归零
4. Z 执行机械归零
5. 确认 Slide/Z homed=True 且 position_valid=True
6. 确认 Shoulder/Elbow/Rotation 的绝对位置有效
7. 初始化协调器读取 SystemStartupConfig
8. 按经过机械验证的安全顺序移动各轴到 startup_position
9. 检查各步骤的接受、故障和可用状态
10. 设置 system_initialization_completed=True
11. 进入正常工作阶段
```

### 12.2 初始化完成后的规则

初始化完成后：

- `SystemStartupConfig` 不参与正常运动命令；
- 运动学模块不读取 `SystemStartupConfig`；
- 前端单轴控制不自动返回 `startup_position`；
- 所有绝对目标继续使用逻辑工作坐标；
- 后续目标不以工作零点为偏移；
- 工作零点不改变 FK/IK 定义；
- 需要重新执行完整初始化时，应由系统状态机显式进入初始化模式。

### 12.3 运行阶段禁止调用

当系统已经处于正常工作阶段时，不提供公共的：

```python
move_to_work_zero(axis)
```

若维护或调试确实需要重新移动到初始位置，应由单独的维护模式或系统复位流程触发，并明确处理：

- 机械安全顺序；
- 当前位置有效性；
- 轴间干涉；
- 故障传播；
- 中途停止。

不得复用前端普通单轴按钮绕过初始化协调器。

---

## 13. 给运动学模块使用的接口

运动学模块不直接调用：

- `MG4010Driver`；
- `CanRotaryJoint`；
- `FeetechRotationAxis`；
- `STM32MotionClient`；
- `SystemStartupConfig`。

运动学只使用正常运行阶段的逻辑角度。

### 13.1 运动学关节状态

```python
@dataclass(frozen=True)
class ArmJointState:
    shoulder_deg: float
    elbow_deg: float
    rotation_deg: float | None

    shoulder_valid: bool
    elbow_valid: bool
    rotation_valid: bool
```

平面二连杆正运动学和逆运动学只依赖：

```text
shoulder_deg
elbow_deg
```

末端旋转角不参与 Planar 2R 的 XY 位置逆解，但可以作为同一次上层姿态命令的一部分。

### 13.2 运动学输出目标

```python
@dataclass(frozen=True)
class ArmJointTarget:
    shoulder_deg: float
    elbow_deg: float
    rotation_deg: float | None = None

    shoulder_velocity_deg_s: float | None = None
    elbow_velocity_deg_s: float | None = None
    rotation_velocity_deg_s: float | None = None
```

运动学输出必须满足：

- 使用统一逻辑角度；
- 与轴配置的正方向一致；
- 使用逻辑零点作为角度零点；
- 位于对应逻辑软限位内；
- 已完成逆运动学可达性检查和候选解筛选；
- 不包含任何 `startup_position` 偏移。

统一控制层仍必须再次检查软限位。

### 13.3 运动学调用接口

```python
class KinematicsMotionInterface(Protocol):
    def get_arm_joint_state(self) -> ArmJointState:
        ...

    def command_arm_joint_target(
        self,
        target: ArmJointTarget,
        *,
        wait: bool = False,
        timeout_s: float | None = None,
    ) -> tuple[MotionResult, ...]:
        ...
```

第一版 `command_arm_joint_target()` 的语义：

1. 验证肩、肘及可选末端旋转角；
2. 检查各轴后端是否可用；
3. 检查系统初始化是否已经完成；
4. 通过统一单轴接口分别下发绝对目标；
5. 返回每个轴独立的 `MotionResult`；
6. 任一轴下发失败时，尽力停止已经下发的肩、肘轴；
7. 不声称严格同步；
8. 不实现连续轨迹；
9. 不读取或使用工作零点。

由于肩、肘和末端旋转轴当前尚未统一实现可靠到位判断，第一版应使用：

```text
wait=False
```

返回：

```text
accepted=True, completed=None
```

只表示命令已接受，不表示已经到位。

### 13.4 运动学模块禁止事项

运动学模块不得：

- 发送 motor degree；
- 发送 raw encoder count；
- 自行应用减速比；
- 自行应用肩肘逻辑零点偏移；
- 自行应用关节方向变换；
- 读取工作零点配置；
- 将工作零点作为关节坐标原点；
- 从运动学角度中减去工作零点；
- 直接访问 CAN 或串口；
- 将机械归零与逻辑角度零点混用；
- 在逆解不可达时发送目标；
- 静默选择违反软限位的逆解。

---

## 14. 后端映射约定

### 14.1 Slide / Z

映射到：

```text
STM32MotionClient
```

统一层负责：

- mm 与 µm 转换；
- 查询状态；
- 绝对运动；
- 相对运动；
- 机械归零；
- 停止；
- 使能和禁用；
- 事件完成状态映射。

Slide/Z 移动到上电工作初始位置之前，必须先完成机械归零。

### 14.2 Shoulder / Elbow

映射到：

```text
CanRotaryJoint
```

统一层负责：

- deg 与 rad 转换；
- 使用现有逻辑零点；
- 使用现有方向配置；
- 使用现有减速比；
- 使用现有软限位；
- 下发绝对位置；
- 映射软件停止。

肩、肘不提供机械归零。

`startup_position` 只在上电初始化时作为一次普通逻辑绝对目标使用。

### 14.3 Rotation

映射到：

```text
FeetechRotationAxis
```

统一层负责：

- deg 与 rad 转换；
- 使用已标定的逻辑零点；
- 使用已标定的方向；
- 使用已标定的软限位；
- 转换为原始编码器目标；
- torque enable/disable。

末端旋转轴不提供机械归零。

`startup_position` 只在上电初始化时作为一次普通逻辑绝对目标使用。

---

## 15. 调用示例

### 15.1 前端查询轴描述

```python
for axis in motion.list_axes():
    print(
        axis.name,
        axis.minimum_position,
        axis.maximum_position,
        axis.capabilities,
    )
```

返回的轴描述不包含工作零点。

### 15.2 Slide 机械归零

```python
result = motion.home_reference(
    AxisName.SLIDE,
    wait=True,
    timeout_s=30.0,
)
```

### 15.3 正常控制肩关节

```python
result = motion.move_absolute(
    AxisName.SHOULDER,
    position=25.0,
    velocity=5.0,
)
```

该目标是相对于逻辑零点的 `25 deg`，与上电工作初始位置无关。

### 15.4 运动学下发肩肘目标

```python
target = ArmJointTarget(
    shoulder_deg=25.0,
    elbow_deg=-65.0,
)

results = kinematics_motion.command_arm_joint_target(
    target,
    wait=False,
)
```

### 15.5 上电初始化

```python
startup_config = SystemStartupConfig(
    targets=(
        StartupAxisTarget(AxisName.SLIDE, position=...),
        StartupAxisTarget(AxisName.Z, position=...),
        StartupAxisTarget(AxisName.SHOULDER, position=...),
        StartupAxisTarget(AxisName.ELBOW, position=...),
        StartupAxisTarget(AxisName.ROTATION, position=...),
    )
)

state = initializer.initialize_to_work_ready(
    startup_config,
    timeout_s=...,
)
```

初始化完成后，`startup_config` 不再进入运动学或普通运动控制链路。

---

## 16. 前端约束

前端应：

- 通过 `list_axes()` 和 `describe_axis()` 获取常规控制能力；
- 根据 `position_unit` 显示 mm 或 deg；
- 根据逻辑软限位限制输入；
- 根据 capability 决定是否显示 Home、Relative、Stop 等按钮；
- 将机械归零与系统上电初始化分开显示；
- 将 `accepted=True, completed=None` 显示为“命令已发送”；
- 在系统初始化未完成时禁止启动运动学任务；
- 只显示系统级的“初始化状态”，不将工作零点暴露为普通单轴目标。

前端不应：

- 为所有轴显示机械 Home 按钮；
- 提供普通的“回工作零点”单轴按钮；
- 将工作零点假定为 `0.0`；
- 将工作零点作为坐标偏移；
- 硬编码 CAN ID、串口轴字母或舵机 ID；
- 显示电机原始编码器值作为工作角度；
- 在不支持 wait 的轴上显示“已经到位”。

---

## 17. 配置要求

### 17.1 常规轴配置

每个轴的常规控制配置至少包含：

```python
@dataclass(frozen=True)
class AxisMotionConfig:
    minimum_position: float
    maximum_position: float

    default_velocity: float | None
    default_acceleration: float | None
```

旋转轴底层配置还负责：

```text
logical zero
direction
gear ratio
counts per turn
raw limits
```

### 17.2 系统初始化配置

上电工作初始位置必须单独保存：

```python
@dataclass(frozen=True)
class SystemStartupConfig:
    targets: tuple[StartupAxisTarget, ...]
```

不能把 `startup_position` 合并到：

- `AxisMotionConfig`；
- `AxisDescriptor`；
- `ArmJointTarget`；
- `ArmJointState`；
- 运动学模型参数；
- 坐标变换参数。

当前尚未确认的初始位置、限位和速度必须保留为显式待配置项，不得从测试脚本猜测。

---

## 18. 当前未覆盖内容

以下内容不属于本版接口：

- 多轴时间同步；
- 速度连续模式；
- 笛卡尔空间轨迹；
- 插补；
- 关节到位稳定窗口；
- 肩、肘和末端旋转轴的统一超时停止；
- 硬件急停；
- 碰撞检测；
- 自动抓取流程；
- 吸盘状态机；
- 视觉坐标到机器人坐标变换；
- 运行阶段一键回到初始姿态；
- 初始化失败后的自动恢复策略。

---

## 19. 交接给运动学开发者的最小约定

运动学开发可以依赖以下稳定约定：

1. 肩、肘输入输出角度统一使用 `deg`；
2. 肩、肘角度均为逻辑工作角度；
3. 角度零点是逻辑零点，不是上电工作初始位置；
4. 正方向、零点、减速比和电机原始位置转换由控制层处理；
5. 运动学只需要输出 `ArmJointTarget`；
6. 运动学必须检查目标可达性和候选解；
7. 控制层会再次检查关节软限位；
8. `rotation_deg` 不参与 Planar 2R 的 XY 逆解；
9. 第一版运动学下发使用 `wait=False`；
10. `accepted=True, completed=None` 只表示命令已接受；
11. 运动学不得直接调用底层驱动；
12. 运动学不得读取、保存或使用各轴工作零点；
13. 工作零点只由系统上电初始化协调器使用；
14. 初始化完成后，所有解算继续使用正常逻辑坐标。

---

## 20. 待双方确认的配置项

| 项目 | 当前状态 |
| --- | --- |
| Slide 逻辑正方向 | 待机械确认 |
| Z 逻辑正方向 | 已知向上为正，需与最终配置核对 |
| Shoulder 逻辑正方向 | 由现有关节配置提供，需运动学方确认 |
| Elbow 逻辑正方向 | 由现有关节配置提供，需运动学方确认 |
| Rotation 逻辑正方向 | 待 Feetech 标定 |
| Slide 软限位 | 待最终机械实测 |
| Z 软限位 | 待最终机械实测 |
| Shoulder 软限位 | 使用现有配置，需机械复验 |
| Elbow 软限位 | 使用现有配置，需机械复验 |
| Rotation 软限位 | 待 Feetech 标定 |
| 各轴 startup position | 待确定，仅用于上电初始化 |
| Planar 2R `L1/L2` | 待写入正式配置 |
| 上电初始化安全顺序 | 待机械安全验证 |

未确认参数必须保留为显式配置或 TODO，不得写死为未经验证的生产默认值。

---

## 21. 实现验收约束

实现完成后必须验证：

1. 普通 `AxisDescriptor` 中不存在 `startup_position`；
2. `AxisCapabilities` 中不存在 `move_to_work_zero`；
3. `SingleAxisMotionInterface` 中不存在 `move_to_work_zero()`；
4. `ArmJointState` 和 `ArmJointTarget` 中不存在工作零点字段；
5. 运动学代码不导入 `SystemStartupConfig`；
6. 运动学计算中不存在工作零点偏移；
7. 电机目标换算中不存在 `startup_position` 加减；
8. 工作零点只由 `SystemInitializationInterface` 使用；
9. Slide/Z 在机械归零前不能执行上电初始位置移动；
10. 系统初始化完成后，正常运动只使用逻辑工作坐标；
11. 对肩、肘和末端旋转轴调用机械归零会明确返回不支持；
12. 未确认的上电初始位置不会被自动猜测或写死。

建议增加静态或单元测试，明确锁定以上边界。

---

## 22. 术语对照表

| 中文术语 | English Term | Abbreviation |
| --- | --- | --- |
| 统一运动控制器 | Unified Motion Controller | 无常用缩写 |
| 单轴运动接口 | Single-Axis Motion Interface | 无常用缩写 |
| 逻辑工作坐标 | Logical Work Coordinate | 无常用缩写 |
| 机械归零 | Reference Homing | 无常用缩写 |
| 上电工作初始位置 | Startup Work Position | 无常用缩写 |
| 工作零点 | Work-Zero Position | 无常用缩写 |
| 逻辑零点 | Logical Zero | 无常用缩写 |
| 系统初始化配置 | System Startup Configuration | 无常用缩写 |
| 系统初始化协调器 | System Initialization Coordinator | 无常用缩写 |
| 软限位 | Software Limit | Soft Limit |
| 绝对位置控制 | Absolute Position Control | 无常用缩写 |
| 相对位置控制 | Relative Position Control | 无常用缩写 |
| 运动学 | Kinematics | 无常用缩写 |
| 逆运动学 | Inverse Kinematics | IK |
| 正运动学 | Forward Kinematics | FK |
| 执行器 | Actuator | 无常用缩写 |
| 适配器 | Adapter | 无常用缩写 |
| 能力描述 | Capability Description | 无常用缩写 |
| 到位确认 | Arrival Confirmation | 无常用缩写 |
| 逻辑关节角 | Logical Joint Angle | 无常用缩写 |
| 任务协调器 | Task Coordinator | 无常用缩写 |
