# MushroomRobotController 应用层能力接口

> 顶层进程入口现为 `MushroomRobotService`；本控制器只负责单次机器人能力、Tray 门禁与 Base 计划/执行，不拥有完整采摘流程。

## 1. 职责

`host/application/controller.py` 定义整机上层边界。它把当前正式 Base-frame 能力与未来视觉能力
分开，并保证视觉目标最终只复用现有 Base 运动入口。

```text
Manual Base Target ───────────────────────────────┐
                                                  ├─> plan/move_to_base_pose
Vision Observation → VisionTargetResolver → Base TCP Goal ┘
                                                     ↓
                       TrayWorkspace(Base) → IK → stages → execution
```

控制器只维护应用层培养槽任务门限，不复制机械臂局部正负偏置区、IK、Slide 选择、跨区规划或
轴执行。

装配层明确区分三份配置：`TrayWorkspaceConfig` 检查 Base 最终任务 TCP，
`OffsetWorkspaceConfig` 注入 solver 的 arm-local 策略，`RobotMotionEnvelopeConfig` 注入 planner
和 startup/return 专用流程。

## 2. 正式可用接口

以下接口通过 `BaseFrameRobotBackend` 转发现有 `DemoMotionFlow` 和统一运动链：

```python
robot.startup()
robot.plan_to_base_pose(x_mm, y_mm, z_mm, yaw_deg)
robot.move_to_base_pose(x_mm, y_mm, z_mm, yaw_deg)
robot.return_to_startup()
robot.stop()
robot.enable_joints()
robot.disable_joints()
robot.suction_grip()
robot.suction_release()
robot.suction_idle()
robot.get_status()
robot.shutdown()
```

这里的 `x_mm/y_mm/z_mm/yaw_deg` 只表示目标 TCP 在 Base frame 中的 xyz+yaw。当前实现不接收
Camera frame 坐标，也不支持任意 roll/pitch。

`plan_to_base_pose()` 在调用 `BaseFrameRobotBackend` 前先调用
`TrayWorkspace.require_xyz_allowed()`。越界时不会读取规划状态、调用 IK、调用 transition planner
或提交硬件命令。`move_to_base_pose()` 必须先调用同一个 `plan_to_base_pose()`，再把得到的计划交给
`execute_base_plan()`；执行入口不重新解算。后端返回字面值 `False` 表示计划未成功执行，
应用控制器会将其转换为 `BaseMotionExecutionError`，调用方不得继续报告成功或执行后续抓取阶段。
到位和静止稳定窗由统一控制器确认，阶段后的状态快照不再用单次 `busy` 样本推翻已确认的
`ARRIVED`；后续阶段提交仍执行忙碌保护。

`plan_base_target(BaseToolTarget, enforce_tray_workspace=...)` 是 Robot Service 和 PickPlanner 的统一值对象入口。只有已知的 pre-grasp/retreat 中间高位阶段可设置 `False`；这不会绕过 Base solver、OffsetWorkspace、轴/关节限位或 RobotMotionEnvelope。

`startup()` 与 `return_to_startup()` 直接使用专用启动流程，因此允许访问培养槽任务区外的
`STARTUP_SAFE_POSE`。普通 `move_to_base_pose(startup_x, startup_y, startup_z)` 不享有该例外。
阶段规划产生的 `LIFT`/`TRANSIT` 只受现有轴限位、偏置区阶段规则和完整 FK 验证约束，不对每个
阶段重复应用培养槽正常 Z 范围；最终 `DIRECT`/`LOWER` 落点来自已经通过任务门限的目标。

## 3. 预留且受门禁的接口

```python
robot.plan_to_observation(observation, grasp_offset)
robot.move_to_observation(observation, grasp_offset)
```

顺序固定为：

```text
独立手眼 validated 门禁
→ 检查 observation frame 和静止采集状态
→ q_capture 的 Base FK
→ Camera target 到 Base object
→ 应用 grasp_offset
→ 检查最终目标仅含当前模型支持的 xyz+yaw
→ 调用同一 plan_to_base_pose() 或 move_to_base_pose()
→ TrayWorkspace 在 IK 前检查最终 Base TCP 目标
```

当前本机 `tool_T_camera=null`，因此这两个接口只会明确拒绝，不会调用 FK、planner 或 submit。
类中存在方法不等于 capability 可用。

## 4. Capability 状态

`RobotCapabilities` 字段：

```python
base_frame_motion: bool
suction_control: bool
rotary_joint_enable_control: bool
hand_eye_calibration: HandEyeCalibrationStatus
vision_target_resolution: bool
vision_target_motion: bool
```

当前预期：

```text
base_frame_motion = true
suction_control = true
rotary_joint_enable_control = true
hand_eye_calibration = missing
vision_target_resolution = false
vision_target_motion = false
```

使用只读命令查看配置对应能力：

```bash
cd host
.venv/bin/python scripts/robot_capabilities.py
```

该命令只读取配置，不打开串口/CAN/Feetech，不执行 Homing 或运动。若未来同时加载经过验证的
Base 与手眼标定，输出会按独立 validation 字段更新。

## 5. 构造与生命周期

应用工厂位于 `host/application/demo_backend.py`：

```python
robot = create_mushroom_robot_controller(
    execute=False,
    frame_config=Path("config/local/frame_transforms.json"),
    tray_workspace_config=Path("config/local/tray_workspace.json"),
)
```

工厂使用 tracked `DEFAULT_OFFSET_WORKSPACE_CONFIG` 与
`DEFAULT_ROBOT_MOTION_ENVELOPE_CONFIG`，并只从显式路径加载 Tray local 配置。相同 envelope 实例
同时提供 startup pose 和 planner side-switch clearance，CLI、controller、solver 与 planner
不会各自创建第二份数值。

仓库不提供推测边界或可执行默认值。本机已按用户 2026-08-05 的明确输入配置 Base X
`[20, 480] mm`、Base Y `[20, 700] mm`、Base Z `[0, 180] mm`，并设置
`metadata.validated=true`。其他机器应以 `config/examples/tray_workspace.json` 为字段参考，修改
Git 跟踪的 `config/local/tray_workspace.json`，填入经确认的 Base-frame 边界并在验收后提交。
配置缺失、仍为 `null` 或未确认时，工厂和真实 CLI 在创建 Runtime、打开硬件前失败关闭。

Base Z `[0, 180] mm` 是最终 TCP 的绝对任务许可高度。上限 `180 mm` 是当前 Base 标定和 TCP
几何的静态计算结果，不会在运行时自动更新；若二者变化，必须重新计算并人工确认。

构造本身不打开硬件。显式 `startup()` 才由原流程打开 Runtime 并执行对应 READ_ONLY 或 MOTION
模式；`shutdown()` 关闭通信资源。真实执行仍必须由调用者显式传入 `execute=True`，沿用现有运动
授权规则。正常 CLI 的 `move` 命令调用 `MushroomRobotController.move_to_base_pose()`；`workspace`
命令只读显示 Base-frame 培养槽边界、arm-local 正负偏置区、Robot motion envelope、startup、
side-switch clearance 和轴/关节限位，并明确 envelope 不是碰撞模型。

## 6. 明确不提供的接口

当前不提供或不声称已真实可执行：

- `move_to_camera_target(...)`；
- `move_to_detection(...)`；
- 经硬件验证的 `pick_detected_object(...)`；软件侧等价编排位于 `VisionPickWorkflow`；
- `move_to_mushroom(...)`；
- 物理吸附确认、自动放置/释放、视觉闭环或运动中图像时间同步。
