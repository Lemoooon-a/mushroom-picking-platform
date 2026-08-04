# Base-frame 五轴目标解算

## 1. Scope

本模块把 Base frame 中的 Tool/工具中心点（Tool Center Point, TCP）目标位姿转换为五轴逻辑
目标，并生成统一 `MultiAxisTarget`。当前只求解 `x/y/z/yaw`，不执行运动、轨迹插补、碰撞检测、
严格同步、速度规划或自动抓取。

## 2. Frame conventions

变换 `A_T_B` 把 B 中表达的坐标转换到 A。外部笛卡尔目标根固定为 Base（B），
`FiveAxisKinematics` 的内部根固定为 Slide-zero（S）：

```text
Base (B) -> Slide-zero (S) -> five-axis chain -> Tool/TCP (T)
```

Slide-zero 是 Slide 与 Z 机械归零、逻辑位置都为 0 时的几何参考；它不是 startup position。

## 3. FK chain

五轴逻辑状态为：

```text
q = [slide_mm, z_mm, shoulder_deg, elbow_deg, rotation_deg]
```

内部和 Base 根正运动学（Forward Kinematics, FK）分别为：

```text
slide_zero_T_tool(q)
base_T_tool(q) = base_T_slide_zero @ slide_zero_T_tool(q)
```

Slide/Z 方向、平面安装、两段连杆和 `rotation_output_T_tool` 全部来自正式几何配置，求解器不
硬编码本机尺寸或标定值。

## 4. Base target conversion

Base 目标只在求解器输入边界转换一次：

```text
slide_zero_T_base = inverse(base_T_slide_zero)
slide_zero_T_tool_target = slide_zero_T_base @ base_T_tool_target
```

转换后所有逆运动学（Inverse Kinematics, IK）计算和 FK 复核都在 Slide-zero 根下完成。

## 5. Five-axis variables

求解器从目标 Tool 位姿移除配置的 `rotation_output_T_tool`，再根据配置的平面安装和轴方向反解
Z、平面 XY、Shoulder、Elbow 与 Rotation。Z 不是直接等于目标高度；固定 TCP Z 偏移、平面安装
高度、Z 正方向以及可能的轴方向分量都会进入反解。超出模型支持容差的 roll/pitch 会明确拒绝，
不会被静默忽略。

当前状态由调用方传入，只用于冗余选择和连续性评分。纯数学模块不查询硬件、本机文件或
startup position。

## 6. Slide redundancy

同一个 TCP 目标可能对应多个 Slide 位置。默认策略为
`KEEP_CURRENT_SLIDE_THEN_NEAREST`：

1. 搜索范围严格来自 Slide 软限位；
2. 当前 Slide、两端软限位和按配置步长生成的有限离散点都会成为候选；
3. 当前 Slide 可达时优先保持；否则优先选择与当前位置最近的合法离散候选；
4. `solve_with_fixed_slide()` 可在调用方明确给出 Slide 时只求该位置。

这是有限、确定性的离散搜索，不是解析连续搜索，也不表示找到数学上唯一或全局最优的解。
默认步长为 5 mm，最终候选仍必须通过完整 FK 残差复核。

## 7. Shoulder/Elbow branches

每个 Slide 候选都复用现有 `Planar2RKinematics.inverse()`，保留
`elbow-positive` 和 `elbow-negative` 两支主值解。伸直或完全折叠奇异点由平面 IK 去重为确定的
单一候选。每支解都检查数值、Shoulder/Elbow 软限位和整体 FK；不会维护第二套余弦定理实现。

## 8. Rotation yaw

FK 与 IK 共用 `rotation_output_yaw_deg()` / `rotation_deg_for_output_yaw()`。当前模型中 Rotation
输出 yaw 为 Shoulder、Elbow 与 Rotation 逻辑角之和；固定 Tool yaw 通过
`rotation_output_T_tool` 的刚性变换统一处理。反解后枚举相差 360° 的周期等价角，只保留
Rotation 软限位内的值，并优先选择最接近当前 Rotation 的等价值。

## 9. Candidate scoring

候选首先必须满足五轴软限位和 FK 残差阈值。合法候选先按 Slide 与当前位置的归一化变化量
排序，再使用集中配置的加权评分：

```text
w_slide * normalized_slide_change
+ w_shoulder * normalized_shoulder_change
+ w_elbow * normalized_elbow_change
+ w_rotation * normalized_rotation_change
+ w_margin * soft_limit_margin_penalty
```

相同排序值使用 `slide, shoulder, elbow, rotation` 数值作为稳定 tie-break，保证结果可重复。

## 10. FK residual verification

每个候选都会重新调用现有五轴 FK，得到 `slide_zero_T_tool_reconstructed`，并与转换后的目标比较：

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

## 12. Provisional calibration gate

本机 `frame_transforms.local.json` 中 `validated=false` 时，构造求解器默认报错：

```text
The Base–Slide-zero transform is provisional and has not passed an independent pose validation.
```

只读调试可显式传入 `allow_unvalidated_base_transform=True`，CLI 对应
`--allow-unvalidated-frame-transform`。该许可只作用于本次预览，不写配置，不把验证状态改成 true。

## 13. Current limitations

- Base–Slide-zero 当前单姿态标定仍是 provisional；
- Slide 使用有限离散搜索，可能漏掉步长之间很窄的可行区间；
- 只支持当前五轴模型允许的 `x/y/z/yaw`；
- 不检查自碰撞、环境碰撞、线缆、奇异邻域速度、路径连续性或动态可达性；
- 候选优选不是数学唯一解，也不是碰撞或路径意义下的全局最优解；
- 离线 FK/IK 和软限位通过不等于真实机构已安全验证。

## 14. Future real-motion integration

真实执行应作为独立任务增加，不能把 `plan-base` 直接改成隐式运动入口。建议顺序是：先完成第二
独立姿态的 Base–Slide-zero 验证并把验证状态通过正式流程更新；再增加专门的低速执行命令，复用
现有 runtime/controller 授权、状态门禁、Rotation torque 风险确认、最大单次位移限制、提交/等待
和失败停止语义；最后从远离奇异点与软限位的小位移开始实机验证。轨迹、碰撞和速度安全仍需另行
设计与验收。
