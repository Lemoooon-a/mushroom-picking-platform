# Frontend Motion Interface Handoff

## 1. Scope

本文是前端成员可独立使用的同进程运动接口交接说明。前端运动接口
（Frontend Motion Interface）只负责查询、提交和轮询五轴逻辑运动；外观
（façade）把调用转发给唯一的 `UnifiedMotionController`，不保存第二套命令或轴状态。

接口不是 HTTP、WebSocket、TCP 或 ROS 2 服务，也不包含系统初始化、轨迹插补、严格同步、
碰撞检测或抓取状态机。

## 2. Import Path

```python
from bootstrap import create_upper_motion_runtime
from config.hardware import load_local_hardware_config
from config.motion_runtime import load_local_motion_config
from motion import (
    AxisName,
    AxisTarget,
    FrontendMotionInterface,
    MultiAxisTarget,
    RuntimeMode,
)

runtime = create_upper_motion_runtime(
    load_local_hardware_config(),
    load_local_motion_config(),
    mode=RuntimeMode.READ_ONLY,
)
motion: FrontendMotionInterface = runtime.frontend_motion
```

稳定类型与接口从 `motion` 导入；runtime 装配函数从 `bootstrap` 导入。不得从 façade 的
`_controller` 私有成员取底层对象。

## 3. Public Units

| 轴类型 | position | velocity | acceleration |
| --- | --- | --- | --- |
| `slide` / `z` | mm | mm/s | mm/s² |
| `shoulder` / `elbow` / `rotation` | deg | deg/s | deg/s² |

公开边界不出现 µm、step、rad、电机角、raw encoder count、gear ratio 或设备 ID。

## 4. Available Axes

固定逻辑轴名为：`slide`、`z`、`shoulder`、`elbow`、`rotation`。代码必须使用
`AxisName`，不要依赖 STM32 的 `S/Z` 字符或硬件编号。

## 5. Public Methods

```text
list_axes() -> tuple[AxisDescriptor, ...]
describe_axis(axis) -> AxisDescriptor
get_state(axis) -> AxisState
get_axis_states(axes=None) -> tuple[AxisState, ...]
submit_absolute(target) -> MotionCommandHandle
submit_positions(target) -> MultiAxisCommandHandle
get_command_result(handle) -> MotionCommandResult
get_group_result(handle) -> MultiAxisCommandResult
stop(axis) -> MotionCommandResult
home_reference(axis, timeout_s=None) -> MotionCommandResult
```

接口不提供 `wait()`、enable、disable、clear_fault、transport 或通用
`move_to_work_zero()`。

## 6. Axis Capabilities

前端必须读取 `AxisDescriptor.capabilities` 决定按钮是否显示/启用，不要按轴名重复硬编码。

| Axis | Query | Absolute | Stop | Reference home | Velocity | Acceleration | Arrival |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `slide` | Yes | Yes | Yes | Yes | Yes | Yes | Yes |
| `z` | Yes | Yes | Yes | Yes | Yes | Yes | Yes |
| `shoulder` | Yes | Yes | Yes | No | Yes | No | Yes |
| `elbow` | Yes | Yes | Yes | No | Yes | No | Yes |
| `rotation` | Yes | Yes | No | No | No | No | Yes |

表格反映当前代码能力；运行时仍以 descriptor 为准。

## 7. Single-Axis Command Example

```python
handle = motion.submit_absolute(
    AxisTarget(
        axis=AxisName.SHOULDER,
        position=25.0,
        velocity=5.0,
    )
)
```

`submit_absolute()` 返回 handle 即表示命令已接受；它没有证明机械已经到位。

## 8. Multi-Axis Command Example

```python
handle = motion.submit_positions(
    MultiAxisTarget(
        targets=(
            AxisTarget(AxisName.SLIDE, position=300.0),
            AxisTarget(AxisName.Z, position=120.0),
            AxisTarget(AxisName.SHOULDER, position=25.0),
        )
    )
)
```

目标可只含部分轴。控制器完整验证后按 tuple 顺序背靠背提交，不保证同时起步或同时到达。

## 9. GUI Polling Pattern

图形用户界面（Graphical User Interface, GUI）主线程应使用定时器进行一次性非阻塞轮询：

```python
def on_motion_timer() -> None:
    result = motion.get_command_result(handle)
    render_motion_result(result)
    if result.completed is None:
        schedule_next_timer_tick()
```

多轴 handle 使用 `get_group_result()`。前端接口刻意不暴露阻塞等待方法；不要在 GUI 主线程
中自建循环或 `sleep()`。

## 10. Command Status Semantics

`accepted=True` 只表示后端接受命令，绝不等于 `arrived`。

| Status | 前端显示 | `accepted` / `completed` |
| --- | --- | --- |
| `ACCEPTED` | 命令已接受 | `True / None` |
| `MOVING` | 正在运动 | `True / None` |
| `ARRIVED` | 已到位 | `True / True` |
| `REJECTED` | 命令被拒绝 | `False / False` |
| `ABORTED` | 运动已中止 | `True / False` |
| `TIMEOUT` | 运动超时 | `True / False` |
| `FAULT` | 设备故障 | `True / False` |
| `COMMUNICATION_ERROR` | 通信错误 | `True / False` |

只有 `ARRIVED` 可显示“已到位”。

## 11. Homing Restrictions

`home_reference()` 是机械归零，只允许 `slide` 和 `z`。对 `shoulder`、`elbow` 或
`rotation` 调用会返回 `REJECTED` 与 `UNSUPPORTED_COMMAND`，不会把当前位置改成零。

前端不得提供“工作零点”按钮。`startup position` 也不属于普通运动接口。

## 12. Stop Restrictions

- `slide` / `z`：支持 STM32 软件停止；
- `shoulder` / `elbow`：支持 MG4010E 软件停止；
- `rotation`：当前没有可靠独立 stop，必须隐藏或禁用 stop 按钮；
- software stop 不是硬件急停，也不会自动 torque disable 或清故障。

已接受的 stop 结果使用 `ABORTED` 表示运动生命周期被中止；不支持的操作为 `REJECTED`。

## 13. Error Display Guidance

提交前验证失败可能抛出 `UnifiedMotionError`；多轴部分提交失败可能抛出
`MultiAxisSubmissionError`，其 `result` 含逐轴结果。handle 已返回后，错误通过 result 的
`status`、`error_code` 和 `message` 显示。

界面应保留明确的轴名和可读 message，不把通信错误归类成软限位，也不把软件 stop 描述为
急停成功。

## 14. Runtime Lifecycle

`create_upper_motion_runtime()` 完成 VID/PID（Vendor ID/Product ID，厂商/产品标识符）设备
解析和唯一 controller 的对象组装，但不打开通信、不初始化、enable、home 或发送运动。
应用入口用 `with runtime:` 或显式 `open()/close()` 管理资源；打开顺序为 STM32、CAN、
Feetech，关闭和失败回滚顺序相反。默认 `READ_ONLY` 会拒绝运动和 home，真实运动必须显式
选择 `MOTION`，Rotation 还需要额外风险授权。

前端模块本身不得创建 runtime 或硬件；应用入口创建一次后注入前端。

## 15. Forbidden Dependencies

前端不得依赖：

- controller 私有成员、命令记录或 backend；
- USB port、VID/PID、CAN bus、CAN ID、Feetech ID；
- STM32/Feetech/MG4010 driver 或 transport；
- raw register、编码器 count、gear ratio、逻辑零点换算；
- `startup_position` 或底层轴字母。

## 16. Current Limitations

当前不支持严格多轴同步、轨迹插补、连续速度控制、笛卡尔直线轨迹、自动 enable、自动归零、
完整系统初始化、自动故障恢复、碰撞检测或真实整机验证。最终机械限位、容差和 timeout 仍需
正式配置与机械验证。

建议冻结 `AxisName`、目标/状态/handle/result DTO 与 `FrontendMotionInterface`；底层 adapter、
轮询细节、transport、最终容差和严格同步语义暂不冻结。

## 17. Minimal Integration Checklist

- [ ] 只从 `runtime.frontend_motion` 获取接口。
- [ ] 单位按 descriptor 显示 mm 或 deg。
- [ ] 按 capabilities 显示 Home/Stop/速度/加速度控件。
- [ ] 单轴用 `get_command_result()` 定时轮询，多轴用 `get_group_result()`。
- [ ] 只有 `ARRIVED` 显示“已到位”。
- [ ] 只为 Slide/Z 提供机械归零。
- [ ] 不为 Rotation 提供可用 stop。
- [ ] 不提供工作零点按钮，不读取硬件端口或私有 controller。
- [ ] 在 fake 环境完成异常、超时和拒绝状态的界面测试。
