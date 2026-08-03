# Kinematics Motion Interface Handoff

## 1. Scope

本文是运动学成员可独立使用的同进程执行接口说明。运动学运动接口
（Kinematics Motion Interface）只负责读取逻辑轴位置、提交多轴最终目标、轮询或等待各轴
到位。它复用唯一的 `UnifiedMotionController`，不访问驱动、transport 或硬件配置。

本接口不是轨迹规划器，不提供 stop、home、单轴调试或系统初始化。

## 2. Import Path

```python
from motion import (
    AxisName,
    AxisTarget,
    KinematicsMotionInterface,
    MultiAxisTarget,
)

# 应用入口负责注入，不在算法模块中创建 runtime。
motion: KinematicsMotionInterface = runtime.kinematics_motion
```

## 3. Axis Names

公开轴为 `AxisName.SLIDE`、`Z`、`SHOULDER`、`ELBOW`、`ROTATION`。不要使用设备编号、
STM32 字符、CAN ID 或 Feetech ID。

## 4. Public Units

| Axes | Position | Velocity | Acceleration |
| --- | --- | --- | --- |
| Slide / Z | logical mm | mm/s | mm/s² |
| Shoulder / Elbow / Rotation | logical deg | deg/s | deg/s² |

运动学边界不发送 rad、电机角或 raw encoder count，不应用减速比。

## 5. Coordinate and Angle Semantics

Shoulder、Elbow 和 Rotation 使用已标定方向与逻辑零点下的 deg。控制层负责 deg↔rad、
方向、零点、减速比和编码器换算。

现有 `Planar2RKinematics` 的纯数学 API 内部使用 rad；把其解写入 `AxisTarget` 前应明确使用
`math.degrees()` 转成公共逻辑 deg。不得添加或减去 `startup_position`。

## 6. Public Methods

```text
get_axis_states(axes) -> tuple[AxisState, ...]
submit_positions(target) -> MultiAxisCommandHandle
get_group_result(handle) -> MultiAxisCommandResult
wait_group(handle, timeout_s=None) -> MultiAxisCommandResult
```

接口没有 `list_axes`、`describe_axis`、`submit_absolute`、`get_command_result`、stop、home、
enable、disable 或 clear_fault。

## 7. Reading Current Axis States

```python
states = motion.get_axis_states(
    (
        AxisName.SLIDE,
        AxisName.Z,
        AxisName.SHOULDER,
        AxisName.ELBOW,
        AxisName.ROTATION,
    )
)
```

结果保持输入顺序。只有 `position_valid=True` 且 `current_position is not None` 的状态才可作为
规划输入；不要从无效位置猜测零点。

## 8. Creating MultiAxisTarget

```python
target = MultiAxisTarget(
    targets=(
        AxisTarget(AxisName.SLIDE, position=300.0),
        AxisTarget(AxisName.Z, position=120.0),
        AxisTarget(AxisName.SHOULDER, position=25.0),
        AxisTarget(AxisName.ELBOW, position=-60.0),
        AxisTarget(AxisName.ROTATION, position=30.0),
    )
)
```

目标只表达最终逻辑位置和可选工程单位速度/加速度，不含时间轨迹或硬件参数。

## 9. Submitting a Target

```python
handle = motion.submit_positions(target)
```

控制器在下发前再次检查 DTO、后端可用性、忙状态、软限位和参数能力，然后按 tuple 顺序
背靠背提交。返回 handle 不表示运动完成。

## 10. Waiting for Arrival

```python
result = motion.wait_group(handle, timeout_s=10.0)
```

`wait_group()` 阻塞调用线程，直到每个参与轴分别到位或出现 timeout/fault/abort/通信错误。
需要非阻塞执行器时使用 `get_group_result(handle)` 轮询，不要把阻塞等待放到共享事件线程。

## 11. accepted / completed Semantics

- `accepted=True, completed=None`：命令已接受或仍在移动；
- `status=ARRIVED, accepted=True, completed=True`：所有参与轴分别确认到位；
- `completed=False`：已拒绝、中止、超时、故障或通信失败；
- accepted 不等于 arrived，也不表示规划目标已经物理实现。

## 12. Partial-Axis Targets

`MultiAxisTarget` 可以只包含当前动作涉及的部分轴：

```python
target = MultiAxisTarget(
    (
        AxisTarget(AxisName.SHOULDER, 20.0),
        AxisTarget(AxisName.ELBOW, -45.0),
    )
)
```

未包含的轴不会被自动移动、停止、归零或补成 `0.0`。同一 target 中不能重复轴。

## 13. Soft-Limit Responsibilities

运动学应在候选解选择阶段排除不可达和违反关节约束的解；统一控制层仍会再次按当前
`AxisDescriptor` 检查软限位。不得依赖控制器静默裁剪，超限目标会被拒绝。

最终 Slide/Z 行程、Shoulder/Elbow/Rotation 机械限位和安全速度仍来自正式配置与机械验证，
不能从示例数值推断生产参数。

## 14. Startup Position Isolation

运动学不得读取、保存或使用 `startup_position`。上电工作初始位置只属于未来的系统初始化
协调器，不是 FK/IK 原点，不进入目标换算，也不改变正常逻辑角。

## 15. Rotation and Planar 2R Relationship

平面二旋转关节（Planar Two-Revolute-Joint, Planar 2R）的 XY 正/逆解只使用 Shoulder 和
Elbow。Rotation 是独立末端姿态轴，可与肩肘放在同一 `MultiAxisTarget` 中，但不参与
Planar 2R 的 XY 逆解。

真实连杆长度 `L1/L2` 必须来自正式配置和机械测量，示例不提供生产默认值。

## 16. Unsupported Assumptions

不得假设：

- 多轴严格同步、同时起步或同时到达；
- 背靠背提交等于硬件时钟同步；
- 存在关节或笛卡尔轨迹插补、连续速度或直线轨迹；
- accepted 等于 arrived；
- 控制器会自动 enable、home、回工作零点或恢复故障；
- Rotation 有独立 stop；
- 已完成碰撞检测、完整抓取状态机或真实整机验证。

建议冻结 `AxisName`、目标/状态/handle/result DTO 与 `KinematicsMotionInterface`；底层 adapter、
transport、最终容差/timeout 和严格同步语义暂不冻结。

## 17. Minimal Integration Checklist

- [ ] 构造函数接收注入的 `KinematicsMotionInterface`，算法模块不创建 runtime。
- [ ] 规划前检查所有输入状态 `position_valid`。
- [ ] 将 Planar 2R rad 解显式转换为公共逻辑 deg。
- [ ] 不处理减速比、方向、编码器零点、电机单位或硬件配置。
- [ ] 目标可只包含需要移动的轴，不补写其余轴。
- [ ] 提交后检查 result；accepted 不作为到位依据。
- [ ] 阻塞执行线程才使用 `wait_group()`；事件线程使用 `get_group_result()`。
- [ ] 不读取 `startup_position`，Rotation 不进入 Planar 2R XY 解。
- [ ] 不声称严格同步、轨迹插补或真实整机验证。
