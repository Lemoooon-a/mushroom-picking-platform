# 培养槽任务工作区门限

## 1. 结论

普通 Base-frame 最终 TCP 目标现在必须先通过 `TrayWorkspace`，然后才能进入五轴逆运动学
（Inverse Kinematics, IK）和阶段规划。门限拒绝时不裁剪坐标、不调用 solver、不调用 planner，
也不提交硬件命令。

用户已于 2026-08-05 明确确认培养槽 Base X/Y/Z 边界，本机配置保存在被 Git 忽略的
`host/config/tray_workspace.local.json`。实现仍不提供推测默认值；机械最大可达范围、测试点、
arm-local 正负偏置区与 150 mm 跨区安全高度都不是培养槽边界来源。

## 2. Workspace Layers

| 约束 | 坐标系 | 回答的问题 | 实现位置 |
| --- | --- | --- | --- |
| Cultivation-tray workspace permission | Base frame | 最终 TCP 目标是否属于允许操作的培养槽区域 | `host/config/tray_workspace.py`、`host/application/tray_workspace.py` |
| Kinematic reachability | Slide-zero/五轴模型 | 机器人是否能在几何、关节和轴限位内到达 | `host/kinematics/base_frame_solver.py` |
| Offset workspace validity | arm-local frame | 当前 Slide 下局部解是否位于正/负偏置矩形，是否需要换侧 | `host/config/workspace_planning.py` |
| Robot motion envelope policy | Base Z + 轴逻辑位置 | startup/return 与跨区中间阶段采用什么固定策略 | `host/config/robot_motion_envelope.py` |

检查结果不能相互替代。机械可达但位于培养槽外的目标必须报告
`TargetOutsideTrayWorkspace`，而不是 `IK failed`。

### Tray Workspace

只检查普通最终 TCP 的 Base 绝对坐标。它不是机器人机械范围，也不检查中间阶段。

### Arm-Local Offset Workspace

只约束移除 Slide 平移后的局部平面 IK 与 Slide 候选。它不使用 Base 全局坐标表达。

### Robot Motion Envelope

集中 startup/return pose 和 side-switch clearance。当前仅是软件阶段策略，不是完整碰撞模型或
经过认证的机械安全包络。

## 3. Configuration Sources

提交模板：`host/config/tray_workspace.example.json`。

本机文件：`host/config/tray_workspace.local.json`（被 Git 忽略）。

当前用户确认值：

```text
X: [20, 480] mm
Y: [20, 700] mm
Z: [0, 180] mm
frame: base
```

数值来源：用户在 2026-08-05 对培养槽任务区域的明确输入。Z 是最终 TCP 在 Base frame 中的绝对
高度，不是 Z 轴逻辑位置，也不是相对 Home 的位移。当前上限是静态快照：已验证的
`base_T_slide_zero.z=420 mm` 加上 `rotation_output_T_tool.z=-240 mm`，得到 Z 轴回零时
`TCP Base Z=180 mm`。若 Base 标定或 TCP 几何变化，必须重新计算并由用户确认配置；运行时不会
自动推导。当前只完成配置加载与离线边界验证，没有据此执行真实硬件运动。

边界为闭区间，并使用默认 `1e-6 mm` 数值容差。配置不会把允许容差内的请求坐标改写到边界。
JSON 的 `metadata.validated` 必须明确为 `true`；模板中的 `null` 和 `false` 不能用于运动入口。

## 4. 调用链

```text
应用状态/关节状态门禁
→ 参数检查
→ TrayWorkspace.require_xyz_allowed(final x/y/z)
→ backend 读取当前状态
→ BaseFrameFiveAxisSolver
→ arm-local 正负偏置区约束
→ BaseMoveTransitionPlanner
→ execute_base_plan
→ validate/submit/wait/FK verification
```

`MushroomRobotController.move_to_base_pose()` 先调用自身的 `plan_to_base_pose()`，再执行返回计划。
未来视觉链将 `base_T_tool_goal` 交给相同入口，因此不会维护第二套工作区判断。

## 5. Startup Safe Pose and Side-Switch Clearance

### Startup Safe Pose

- `startup()` 与 `return_to_startup()` 是唯一允许直接使用 `STARTUP_SAFE_POSE` 的专用流程；
- 普通 Base 目标即使等于启动坐标，也必须通过培养槽门限；

### Side-Switch Clearance

- `DIRECT` 与 `LOWER` 的最终落点等于已通过培养槽检查的请求目标；
- `LIFT` 与 `TRANSIT` 可以高于培养槽正常 Z 上限，不逐阶段调用 `TrayWorkspace`；
- clearance 为 `RobotMotionEnvelopeConfig.side_switch.clearance_base_z_mm=150 mm` 的绝对 Base TCP
  Z 最低高度，不是相对增量、Z Home、Tray `z_max` 或 target `+150 mm`；
- 所有中间阶段仍受轴软限位、运动包络策略、正负偏置阶段规则及完整 FK 验证约束。

## 6. CLI

正常真实运动 CLI 要求显式加载用户确认的配置：

```bash
cd host
.venv/bin/python scripts/run_motion_demo.py \
  --tray-workspace-config config/tray_workspace.local.json
```

省略 `--execute` 仍是只读预览。`workspace` 命令显示 Tray、arm-local offset、Robot motion
envelope、startup、clearance 和各轴软限位。越界 `move`
输出 `REJECTED: target outside cultivation-tray workspace.`，不会自动 stop 或发送运动命令。

## 7. Known Safety Limitations

当前 Robot motion envelope 不包含 Base XYZ 完整机械包络、培养槽围栏几何、link sweep、
self-collision、path collision、emergency stop 或 dynamic obstacle。离线门限、IK、FK 和阶段测试
不能替代真实机构验证。
