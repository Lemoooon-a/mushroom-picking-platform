# Base-frame 五轴目标解算

## 1. Scope

本模块把 Base frame 中的 Tool/工具中心点（Tool Center Point, TCP）目标位姿转换为五轴逻辑
目标，并生成统一 `MultiAxisTarget`。当前只求解 `x/y/z/yaw`，不执行运动、轨迹插补、碰撞检测、
严格同步、速度规划或自动抓取。

## 2. Frame conventions

变换 `A_T_B` 把 B 中表达的坐标转换到 A。笛卡尔目标和
`FiveAxisKinematics` 的根都固定为机械 Base（B）：

```text
Base (B) -> Slide Y + Z + planar Shoulder/Elbow -> TCP/Rotation center (T)
                                                 -> Camera (C)
```

Slide=0 时 Base XY 与肩关节平面原点 XY 重合，Base/平面 roll、pitch、yaw 都为 0。
Rotation 输出轴中心与 TCP 位置同心。

## 3. FK chain

五轴逻辑状态为：

```text
q = [slide_mm, z_mm, shoulder_deg, elbow_deg, rotation_deg]
```

正运动学（Forward Kinematics, FK）为：

```text
x = planar_2r_fk_x(shoulder, elbow)
y = slide_mm + planar_2r_fk_y(shoulder, elbow)
z = tcp_height_at_z_zero_mm + z_mm
yaw = shoulder_deg + elbow_deg + rotation_deg
```

Rotation 只改变 yaw，不改变 TCP XYZ。两段连杆长度和
`tcp_height_at_z_zero_mm` 来自正式几何配置；Base XY/yaw、Slide +Y 与 Z +Z 由机械定义固定。

## 4. Base target input

Base 目标直接进入求解器，不再经过外部单点 Base 标定：

```text
base_T_tool_target = requested_base_T_tool_target
```

历史 `base_T_slide_zero` 可继续留在 frame 文档供审计，但正常 FK、IK 和视觉规划不读取其数值。

## 5. Five-axis variables

固定 Slide 候选后，求解器直接使用 `local_x=target_x`、`local_y=target_y-slide`，并计算
`z_mm=target_z-tcp_height_at_z_zero_mm`。平面 2R IK 求 Shoulder/Elbow，Rotation 继续复用既有
角度关系维持目标 yaw。超出模型支持容差的 roll/pitch 会明确拒绝，不会被静默忽略。

当前状态由调用方传入，只用于冗余选择和连续性评分。纯数学模块不查询硬件、本机文件或
startup position。

## 6. Offset Workspace Constraints

偏置矩形是运动学强约束，不只是可视化提示。分类使用移除 Slide 平移后的机械臂平面局部坐标，
而不是 Base 全局 Y：

```text
Positive: local_x in [50, 450] mm, local_y in [150, 350] mm, center_y=+250 mm
Negative: local_x in [50, 450] mm, local_y in [-350, -150] mm, center_y=-250 mm
```

边界包含在内；中心空白区和矩形外部均为 `OUTSIDE`。最终普通五轴解必须位于正偏置区或负偏置
区。局部边界、容差和 10 mm fallback 步长集中在 `config/project/workspace_planning.py`；该配置不再
包含 startup 或 Base-frame clearance。

该 arm-local 偏置区只回答当前 Slide 下肩肘解是否有效以及是否需要重新分配 Slide；它不是
Base frame 中的培养槽任务许可。最终用户目标另由应用层 `TrayWorkspace` 在调用本求解器前检查，
两类配置不得合并。

### Positive and Negative Offset Regions

`compute_arm_local_target()` 是 solver、当前状态侧别判断和诊断共用的唯一局部目标换算入口，
不会在多个模块重复手写 `tool_y - slide`。

### Keep-Current-Slide Policy

当前 Slide 下只要存在通过工作区、完整五轴闭区间限位、平面 IK 和 FK 残差的合法解，就立即使用
`KEEP_CURRENT_SLIDE`。此时不得为了更靠近偏置区中心而重新移动 Slide，也不生成中心或 fallback
候选。

### Slide Candidate Selection Priority

固定优先级为 `KEEP_CURRENT_SLIDE > OFFSET_CENTER > OFFSET_FALLBACK`。当前 Slide 失败后同时验证
正负中心候选，并只在两个中心均失败后，以每侧最多 64 个、默认 10 mm 步长的有限候选搜索矩形
内部。同一优先级内按 Slide、Shoulder、Elbow、Rotation 变化量、距该侧中心距离、FK 残差和稳定
枚举顺序作字典序选择。

## 7. Shoulder/Elbow branches

每个 Slide 候选都复用现有 `Planar2RKinematics.inverse()`，保留
`elbow-positive` 和 `elbow-negative` 两支主值解。伸直或完全折叠奇异点由平面 IK 去重为确定的
单一候选。每支解都检查数值、Shoulder/Elbow 软限位和整体 FK；不会维护第二套余弦定理实现。

## 8. Rotation yaw

FK 与 IK 共用 `rotation_output_yaw_deg()` / `rotation_deg_for_output_yaw()`。当前模型中 TCP
yaw 为 Shoulder、Elbow 与 Rotation 逻辑角之和。反解后枚举相差 360° 的周期等价角，只保留
Rotation 软限位内的值，并优先选择最接近当前 Rotation 的等价值。

## 9. Workspace-Side Classification

当前侧别由当前实际五轴状态经 FK 得到 `base_T_tool(current_q)`，再由统一 helper 计算
`local_x/local_y`；不得由历史标签、Slide 正负或 Base 全局 Y 推断。目标侧别来自最终选中的
`FiveAxisSolution.workspace_side`。

## 10. FK residual verification

每个候选都会重新调用现有五轴 FK，得到 `base_T_tool_reconstructed`，并与 Base 目标比较：

- `position_error_xyz_mm`：XYZ 分量误差；
- `position_residual_mm`：三维位置误差范数；
- `yaw_residual_deg`：考虑角度回绕后的 yaw 误差。

任一残差超过配置阈值，候选立即被拒绝。

## 11. MultiAxisTarget output

`FiveAxisSolution` 保存五轴位置、XYZ/位置/yaw 残差、评分、肩肘分支、Slide 选择原因和各轴限位
余量。`solution_to_multi_axis_target()` 将其转换成完整五轴目标：

```text
slide, z                 -> mm
shoulder, elbow, rotation -> deg
velocity, acceleration   -> None（默认）
```

`MultiAxisTarget` 不表达笛卡尔 frame；frame 只属于 IK 输入 `base_T_tool_target`。输出中没有
`frame_id`、Base offset、startup position、rad、原始编码器计数或生产速度猜测。

## 12. Historical Base calibration compatibility

本机 `host/config/robot_runtime.json` 的 `frame_transforms` 区块仍可保存历史
`base_T_slide_zero` 及其 validation
metadata，既有标定/审计脚本也保留；这些字段不再是 Base-frame 规划门禁。`base_T_slide_zero`
缺失时 loader 以 identity 兼容相机配置。正式运动学门禁改为几何配置中的连杆长度和现场测得的
`tcp_height_at_z_zero_mm` 都存在，且 `geometry_confirmed=true`。

## 13. Safe Side-Switch Transition

同一合法侧输出一个 `DIRECT`。`POSITIVE <-> NEGATIVE`，以及 `OUTSIDE` 当前状态需要改变任意
平面轴时，固定输出 `LIFT`、`TRANSIT`、`LOWER`。跨正负偏置区必须先抬升，再横向过渡，最后
下降。

### LIFT, TRANSIT, and LOWER Invariants

- `LIFT` 保持当前 Base X/Y/yaw 与 Slide/Shoulder/Elbow/Rotation，只由正式 IK 换算 Z；
- `TRANSIT` 使用最终解的四个平面轴，并在 clearance Base Z 完成侧别切换；
- `LOWER` 使用最终完整解，与 `TRANSIT` 的四个平面轴完全相同，因此只改变 Z。

每阶段都形成完整 `MultiAxisTarget`，检查五轴限位、工作区语义和 FK 平移/yaw 残差；任一阶段
失败即拒绝整个计划，不返回部分计划。

### Clearance Height Calculation

```text
clearance_base_z = max(current_tcp_base_z, target_tcp_base_z, 150 mm)
```

150 mm 来自 `config/project/robot_motion_envelope.py` 的当前项目软件安全阶段策略，是 Base 绝对最低高度，
不是相对当前位置再抬升 150 mm。先在 Base frame
构造该高度，再通过正式几何 helper 求 Z 逻辑位置，不使用固定轴增量。若当前或目标已经高于
150 mm，则保持两者中较高高度；当前 TCP 已在最高点且高于 150 mm 时，`LIFT` 是零位移验证，
随后可直接 `TRANSIT`。要求的安全高度超出 Z 逻辑范围时拒绝整个计划。

150 mm 不是培养槽正常作业 Z 上限。应用层只检查最终任务目标；`LIFT`/`TRANSIT` 可高于培养槽
Z 上限，但仍必须通过本节已有的轴软限位、阶段约束与完整 FK 验证。

`BaseMoveTransitionPlanner` 由装配处显式注入 `RobotMotionEnvelopeConfig`；solver 只持有
`OffsetWorkspaceConfig`。planner 不加载 local 文件、不访问硬件，也不从 Demo 脚本读取常量。

### OUTSIDE Conservative Policy

当前 `OUTSIDE` 时，只有能以当前 Slide/Shoulder/Elbow/Rotation 完整 FK 证明 Base X/Y/yaw 不变、
仅 Z 改变，才允许单阶段 `DIRECT`；其余到合法侧的平面运动一律使用三阶段。

### Planning-Only CLI

`plan-base` 只打开 `READ_ONLY` runtime，读取状态和机械几何、规划并逐阶段调用
`validate_positions()`。它没有 `--execute`，不调用 submit、wait、home、stop 或 torque enable。

## 14. Current limitations

- 本机 `tcp_height_at_z_zero_mm` 尚未现场填写并确认时，正式 Base 规划保持 fail-closed；
- fallback 使用有限 10 mm 离散搜索，可能漏掉步长之间很窄的可行区间；
- 只支持当前五轴模型允许的 `x/y/z/yaw`；
- 不检查自碰撞、环境碰撞、线缆、奇异邻域速度、路径连续性或动态可达性；
- 候选优选不是数学唯一解，也不是碰撞或路径意义下的全局最优解；
- 离线 FK/IK 和软限位通过不等于真实机构已安全验证。
- `RobotMotionEnvelopeConfig` 仅集中 startup 和 side-switch 阶段策略，不包含完整机械包络、
  自碰撞、环境碰撞或动态障碍。

## 15. Future real-motion integration

真实执行前必须先现场测量并复核 `tcp_height_at_z_zero_mm`，再用已知机械姿态验证 Slide、Z 和
Rotation 的 FK 验收关系。低速执行继续复用现有 runtime/controller 授权、状态门禁、Rotation
torque 风险确认、最大单次位移限制、提交/等待和失败停止语义，并从远离奇异点与软限位的小位移
开始。轨迹、碰撞和速度安全仍需另行设计与验收。
