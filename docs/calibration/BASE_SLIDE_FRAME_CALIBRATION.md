# Base–Slide-zero 坐标标定

## 1. 安全范围

三个工具均默认只读或预览：

- 不自动 home；
- 不自动移动或 stop；
- 不自动 enable；
- 不自动 torque enable；
- 不修改逻辑零点、软件限位或电机寄存器。

运行采集工具前，操作者必须通过既有安全界面把机器人移动到参考姿态，确认机械空间、急停和
人员安全，然后停止所有轴。脚本打开通信、对肩/肘执行现有只读绝对位置初始化、连续读取状态，
最后可靠关闭 Runtime。

## 2. 当前必须先补齐的 FK provider

仓库当前缺少正式五轴 FK、真实连杆长度和已冻结的 Tool 轴方向，因此脚本要求显式：

```bash
--fk-provider your_module:SLIDE_ZERO_KINEMATICS
```

该对象必须实现纯运动学 Protocol：

```python
class SlideZeroKinematics(Protocol):
    def forward_kinematics(
        self,
        axis_state: RobotAxisState,
    ) -> RigidTransform:
        ...  # returns slide_zero_T_tool
```

provider 不得查询硬件或读取 startup position。请先确认 Tool 原点/轴方向、实际连杆参数、
Slide/Z 方向和 Rotation 几何，再实现并审查 provider。当前 Planar 2R 只给出肩肘 XY 点，
不足以安全代替完整五轴 `slide_zero_T_tool`。

## 3. 标定前机械与状态条件

1. Slide 和 Z 已完成机械归零；
2. Slide/Z 逻辑位置接近 0，默认容差均为 `±0.5 mm`；
3. 肩、肘、Rotation 当前逻辑角可读且有效；
4. 五轴均无 fault，`busy` 不为 `True`；
5. 若后端 `busy=None`，至少三次连续样本必须在稳定阈值内；
6. TCP 在 Base 中的已知参考位姿至少包含 `x/y/z/yaw`；
7. 参考 yaw 和 FK 使用同一 Tool frame 定义。

默认读取 20 个样本，间隔 `0.05 s`。直线轴和旋转轴默认最大漂移分别为 `0.1 mm` 和
`0.1 deg`。角度使用圆周平均，`179 deg` 与 `-179 deg` 不会错误平均为 0。

## 4. 计算公式

标定使用实际五轴逻辑状态，不使用配置初始角：

```text
base_T_slide_zero
  = base_T_tool_reference
  @ inverse(slide_zero_T_tool(q_calibration))
```

并计算：

```text
slide_zero_T_base = inverse(base_T_slide_zero)
```

程序重建参考 Tool 位姿、报告位置/yaw 数值残差、实际 roll/pitch，以及 Slide-zero yaw 与
期望 Base `+y` 对齐方向的误差。单点代数残差天然接近零，不能据此声明机械验证完成。

Base–Slide-zero 当前只接受平移加 yaw 假设。明显 roll/pitch 不会被静默投影掉；yaw 对齐或
roll/pitch 超限会使 `valid=false`，默认禁止保存。轴方向相反时可把期望 yaw 配置为 180°。

## 5. 预览标定

在 `host/` 中执行：

```bash
.venv/bin/python scripts/calibrate_base_slide_frame.py \
  --tcp-x-mm 0 \
  --tcp-y-mm 0 \
  --tcp-z-mm 200 \
  --tcp-yaw-deg 0 \
  --fk-provider your_module:SLIDE_ZERO_KINEMATICS
```

常用可选参数：

```text
--expected-slide-yaw-deg 0
--slide-zero-tolerance-mm 0.5
--z-zero-tolerance-mm 0.5
--max-yaw-error-deg 5
--max-roll-pitch-deg 1
--samples 20
--sample-interval-s 0.05
--max-linear-drift-mm 0.1
--max-rotary-drift-deg 0.1
```

输出包含捕获的五轴逻辑状态、参考 `base_T_tool`、计算的正逆固定变换、残差、对齐误差、
roll/pitch、有效状态和 warnings。默认不修改文件。

## 6. 保存和备份

首次写入必须显式添加：

```bash
--write-local
```

目标文件为：

```text
host/config/frame_transforms.local.json
```

该文件被 Git 忽略。保存采用同目录临时文件和原子替换；已有文件默认拒绝覆盖，复核预览并
完成外部备份后才可添加 `--force`。无效标定同样只有 `--force` 才允许保存，并会在 metadata
保留 `valid=false` 和 warnings。更新 Base 变换时保留已有 `tool_T_camera`。

metadata 包含 UTC 时间、Git commit、五轴位置、Base TCP 参考、正逆固定变换、残差、对齐
检查、操作者 notes。首次单点结果固定记录：

```json
"validated": false
```

不要把本机标定文件提交到仓库；应按设备序列号和日期在受控外部位置备份。

## 7. 第二独立参考位姿验证

把 TCP 移动到另一个已知 Base 位姿并停止，执行：

```bash
.venv/bin/python scripts/verify_base_slide_frame.py \
  --tcp-x-mm ... \
  --tcp-y-mm ... \
  --tcp-z-mm ... \
  --tcp-yaw-deg ... \
  --fk-provider your_module:SLIDE_ZERO_KINEMATICS \
  --max-position-error-mm 2 \
  --max-yaw-error-deg 2
```

该步骤不要求 Slide/Z 位于零位，但仍要求它们 `homed=True`、位置有效、静止且无 fault。
程序计算：

```text
predicted_base_T_tool
  = saved_base_T_slide_zero @ current_slide_zero_T_tool
```

并输出 x/y/z 分量误差、三维位置误差、yaw 误差和通过/失败。退出码：

- `0`：阈值内；
- `1`：位置或 yaw 超限；
- `2`：配置、通信、FK provider 或状态错误。

默认不改 metadata。验证通过并人工复核后，可显式使用：

```bash
--write-validation --force
```

此时才写入 `validated=true` 及第二姿态证据。

## 8. Tool–Camera 固定外参

`tool_T_camera` 把 Camera 中的坐标转换到 Tool，不由 Base–Slide-zero 标定求解。它可来自
CAD（Computer-Aided Design，计算机辅助设计）、人工测量或后续独立外参标定，并允许完整
六自由度：

```bash
.venv/bin/python scripts/set_tool_camera_transform.py \
  --x-mm ... --y-mm ... --z-mm ... \
  --roll-deg ... --pitch-deg ... --yaw-deg ... \
  --config config/frame_transforms.local.json
```

默认仅预览并检查 `tool_T_camera @ camera_T_tool` 的 round trip。确认后使用
`--write-local --force`；脚本保留 `base_T_slide_zero` 和既有 metadata。

## 9. 常见失败

- `FK provider ...`：完整 FK 尚未配置或返回类型不正确；
- `not homed` / `position is not valid`：先通过安全界面完成机械归零或位置初始化；
- `outside zero tolerance`：单点标定要求 Slide/Z 停在机械零位；
- `busy` / `unstable`：等待机构完全静止并排除反馈抖动；
- `faulted`：先按现有维护流程人工处理 fault；脚本不会清错；
- yaw alignment 超限：检查 Slide 正方向、Base 定义、参考 yaw 和 Tool/FK 约定；
- roll/pitch 超限：检查参考姿态、机械安装和平面假设；程序不会用投影掩盖误差；
- existing file：备份并复核后再显式使用 `--force`；
- 第二姿态失败：不要把单点结果标记为已验证，先排查 FK 参数、TCP 定义和测量误差。
