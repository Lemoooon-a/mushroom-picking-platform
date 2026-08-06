# Robot Service Runtime

状态：implemented、offline-tested；execute 接口保留但本轮未运行真实硬件。

`MushroomRobotService` 是进程级唯一应用入口，通过私有 axis-motion port 使用内部统一控制器，并持有 `MushroomRobotController`、Vision Gateway、`VisionPickWorkflow`、应用状态和可选 JSON Lines recorder。它不公开 runtime/controller，也不重新实现正运动学（Forward Kinematics, FK）、逆运动学（Inverse Kinematics, IK）或执行器协议。

## API

```python
startup()
shutdown()
status() -> RobotServiceStatus
list_axes() -> tuple[AxisDescriptor, ...]
get_axis_state(axis) -> AxisState
get_axis_states(axes=None) -> tuple[AxisState, ...]
move_axis_absolute(axis, position, velocity=None, acceleration=None, timeout_s=None)
move_axis_relative(axis, delta, velocity=None, acceleration=None, timeout_s=None)
plan_base_target(target: BaseToolTarget)
move_base_target(target: BaseToolTarget) -> MotionResult
request_observation() -> VisionTargetObservation
plan_observation(observation, grasp_profile=None) -> PickPlan
execute_pick_plan(plan) -> PickResult
pick(grasp_profile=None) -> PickResult
return_to_startup()
stop()
enable_joints() / disable_joints()
suction("grip" | "release" | "idle")
```

状态为 `CREATED → STARTING → READY`，观察使用 `OBSERVING`，规划使用 `PLANNING`，真实动作使用 `EXECUTING`，关节失能后为 `DISABLED`，不可恢复执行错误为 `FAULT`，关闭后为 `SHUTDOWN`。Service 用一个私有 active-operation token 串行化 startup、运动、pick、holding 和 suction；状态检查与 token 注册在短临界区内原子完成，阻塞 backend 调用和到位等待不持有状态锁。单轴提交前的参数、busy、位置有效性、Homing、holding 和软限位拒绝保持 READY；获得 command handle 后的 timeout、BUSY、通信或设备故障会 best-effort stop 后进入 FAULT。

`stop()` 和 `shutdown()` 会使当前 token 失效，因此迟到的运动线程不能覆盖较新的 `FAULT` 或 `SHUTDOWN`。没有活动操作时，`stop()` 保持 `CREATED/READY/DISABLED/FAULT/SHUTDOWN` 原状态；它不再充当 lifecycle 或 holding 恢复入口。`DISABLED` 可直接调用 `enable_joints()`，只有重新确认 Shoulder、Elbow、Rotation holding 以及五轴连接、静止、无 fault、位置有效后才进入 `READY`。

## 两种运动入口

- Base-frame task movement：使用 `move_base_target()`，经过 TrayWorkspace、IK、OffsetWorkspace、transition planning 和 motion envelope。
- Raw/manual axis movement：使用 `move_axis_absolute()` / `move_axis_relative()`，只检查所选轴自身状态、holding/Homing 和软限位，不进行 Base-frame 工作区、IK、side-switch、碰撞或 TCP 路径检查。

相对运动在 `UnifiedMotionController` 的同一提交锁内读取调用时当前有效逻辑位置，计算 `current + delta`，校验并走现有绝对提交/到位通路。增量位于轴到位容差内时立即返回 ARRIVED，不发送硬件命令；result 中 `target_position` 仍是解析后的绝对逻辑位置。

## 模式

- `read-only`：加载并检查配置、status/capabilities/workspace；不构造硬件 runtime，不打开硬件。
- `dry-run`：使用纯配置 `OfflinePlanningBackend`、FakeVisionGateway、真实 FK/IK/工作区/transition planner；后端没有硬件 submit API。
- `execute`：使用现有 `DemoMotionFlow`、`UnifiedMotionController` 和 `MotionAuthorization`，CLI 还要求 `--confirm-motion --confirm-rotation-no-stop`。本轮未执行。

入口：

```bash
cd host
.venv/bin/python scripts/robot_service.py --mode read-only
.venv/bin/python scripts/robot_service.py --mode dry-run --fake-position X Y Z
.venv/bin/python scripts/robot_service.py --mode execute \
  --confirm-motion --confirm-rotation-no-stop
```

支持原有命令，并新增 `axes`、`axis state`、`axis states`、`axis move-abs` 和 `axis move-rel`。单轴移动只允许 execute + READY；状态查询允许 execute 的 READY/EXECUTING/DISABLED/FAULT，也允许已有明确模拟状态的 dry-run。

`--record-jsonl PATH` 才会写记录；默认不写仓库。记录包含版本、状态、输入、计划、阶段结果和错误，不记录 `tool_T_camera` 标定矩阵。
