# 视觉到机器人坐标链审计与冻结边界

## 1. 结论

当前正式能力止于：

```text
Base frame 中的 TCP xyz+yaw 目标
→ 培养槽工作区检查
→ 五轴 IK
→ DIRECT 或 LIFT/TRANSIT/LOWER
→ 统一执行
```

Camera frame 到 Base frame 的视觉运动不可用。原因不是缺少一个 API 名称，而是生产链中同时
缺少经过验证的 `tool_T_camera`、视觉观察生产者、相机内参/深度证据和抓取偏移配置。实现不会用
单位矩阵、全零外参、猜测值或“当前最新轴位置”补齐这些缺口。

## 2. 记号与实际坐标系

仓库的 `RigidTransform` 明确定义 `A_T_B`：把 B 中的量转换到 A；组合方向为：

```text
A_T_C = A_T_B @ B_T_C
```

当前实际存在的 frame：

- `Base (B)`：公开工作坐标根；
- `Slide-zero (S)`：Slide/Z 机械零位下的内部运动学根；
- `planar origin`：五轴模型内部肩关节平面原点；
- `rotation output`：第二连杆末端的 Rotation 输出 frame；
- `Tool/TCP (T)`：吸盘工具中心点（Tool Center Point, TCP）；
- `Camera (C)`：仅有 frame/外参配置边界，没有经过验证的实际外参；
- `Object/target (O)`：仅作为本轮冻结的数据契约，没有当前视觉生产者。

## 3. 变换审计表

| Transform | 含义 | 实际来源 | 当前是否存在 | 验证状态 | 使用位置 |
| --- | --- | --- | --- | --- | --- |
| `base_T_slide_zero` | Slide-zero 在 Base 中的位姿 | `frame_transforms.local.json`；Base 标定流程 | 是 | 本机 metadata 为 `validated=true`；本轮未重做硬件验证 | `BaseFrameFiveAxisSolver`、`RobotFrameChain` |
| `slide_zero_T_tool(q)` | 当前五轴状态下 TCP 在 Slide-zero 中的位姿 | `FiveAxisKinematics.forward_kinematics()` | 是 | 本机几何 `geometry_confirmed=true`；有离线 FK 测试 | FK、Base 求解器残差检查 |
| `base_T_tool(q)` | 当前 TCP 在 Base 中的位姿 | 前两项组合 | 是 | 随前两项；有离线组合测试 | Base-root FK、规划当前状态 |
| `tool_T_camera` | Camera 坐标转换到 Tool | 配置槽位、人工录入脚本 | 配置结构存在；本机值为 `null` | 缺失；不存在独立 `tool_camera_validated=true` 记录 | 旧 `RobotFrameChain` Camera helper；新 resolver 门禁 |
| `camera_T_target` | 目标在 Camera 中的完整位姿 | 本轮 `VisionTargetObservation` 契约 | 只有契约 | 无生产者、无真实数据验证 | `VisionTargetResolver` 输入 |
| `object_T_tool_grasp` | 目标到期望 TCP 抓取位姿的偏移 | 每次调用显式 `grasp_offset` | 无项目配置 | 未验证 | `resolve_tool_goal_in_base()` 参数 |
| `base_T_target` | 目标在 Base 中的位姿 | 预期矩阵链组合 | 有受门禁的纯计算实现 | 只有合成测试；真实能力不可用 | `resolve_object_in_base()` |
| `base_T_tool_goal` | 最终 TCP 抓取目标 | `base_T_target @ object_T_tool_grasp` | 有受门禁的纯计算实现 | 只有合成测试；真实能力不可用 | `resolve_tool_goal_in_base()` |

注意：`frame_transforms.local.json.metadata.validated` 只属于 Base–Slide-zero 标定。它绝不验证
`tool_T_camera`。手眼状态只读取独立字段 `tool_camera_validated`。

培养槽边界另由 `tray_workspace.local.json` 提供，frame 固定为 `base`。它既不是标定变换，也不
从机械可达范围、正负偏置区或跨区安全高度推导。本机已按用户 2026-08-05 的明确输入配置
X `[20, 480] mm`、Y `[20, 700] mm`、Z `[0, 180] mm`，并显式标记
`metadata.validated=true`。Z 是最终 TCP 的绝对 Base 高度，`180 mm` 是当前 Z 回零 TCP 高度的
静态快照；该确认不改变手眼标定仍缺失的状态。

## 4. 冻结的 Eye-in-Hand 链

末端相机采用眼在手上（Eye-in-Hand）结构。机器人静止采集时保存 `q_capture`：

```text
base_T_object
  = base_T_tool(q_capture)
  @ tool_T_camera
  @ camera_T_object

base_T_tool_goal
  = base_T_object
  @ object_T_tool_grasp
```

`VisionTargetResolver` 严格按此顺序组合。它不做轴下发、Homing、吸盘、工作区选择、阶段规划
或真实执行。

## 5. 当前视觉输出审计

仓库中没有视觉检测模块或 detection 数据文件，因此当前真实输出类型是“未实现”，不能描述为
2D pixel、3D point、position+yaw 或 6D pose 中的任何一种。仓库也没有找到：

- 相机内参；
- 深度图或深度相机 SDK 接入；
- 像素反投影；
- `camera_T_object` 生产者；
- 目标 yaw 估计策略；
- 视觉置信度与时间戳生产逻辑。

本轮定义的 `VisionTargetObservation` 是未来完整 6D 输出的消费契约，不代表当前视觉算法已能
产生 orientation。若未来视觉只输出 3D 点，应另增与真实能力匹配的 point 契约，不得伪造旋转。

## 6. 时间对应与静止门禁

最小安全规则冻结为：

```text
全部轴已到位且机器人静止
→ 读取稳定五轴逻辑状态
→ 采集图像
→ detection 与该状态组成同一 observation
→ 使用 observation.capture_axis_state 作为 q_capture
```

`capture_motion_state` 只有 `STATIONARY` 可解析；`MOVING` 和 `UNKNOWN` 均拒绝。解析器从不查询
硬件，也不会用解析时的最新轴状态替换采集快照。本阶段不实现运动中时间同步。

## 7. 受门禁的边界

缺少或未验证手眼标定时，解析器抛出 `HandEyeCalibrationUnavailable`，错误明确包含：

```text
Hand-eye calibration is missing or not validated.
Base-frame manual motion remains available.
Camera-target motion is disabled.
```

此外还会拒绝 frame 不匹配、运动中/未知采集状态，以及当前 xyz+yaw 运动模型无法表达的非零
roll/pitch 最终 TCP 目标。合成 validated 手眼数据解析出的 `base_T_tool_goal` 仍必须调用
`MushroomRobotController.move_to_base_pose()`；若目标位于培养槽外，`TargetOutsideTrayWorkspace`
会在 IK、planner 和 submit 前拒绝。视觉入口不得直接访问 solver、planner 或统一运动控制器。
