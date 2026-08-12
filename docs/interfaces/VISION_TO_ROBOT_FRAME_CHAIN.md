# 视觉到机器人坐标链

## 1. 当前链路

视觉观察使用拍照时保存的五轴状态，不使用解析时的“最新状态”：

```text
camera_color_optical_frame 下目标 XYZ
→ target_compensation_camera_mm（一次）
→ tool_T_camera
→ 当前 TCP 下目标位移
→ base_T_tool(q_capture)
→ raw Base target
→ target_compensation_base_mm（一次）
→ final Base target
→ Base TCP workspace
→ 五轴 IK / transition planner
```

矩阵形式：

```text
base_T_target_raw
  = base_T_tool(q_capture)
  @ tool_T_camera
  @ camera_T_target
```

其中 `tool` 明确表示 TCP。现有 `tool_T_camera` 数值、Camera frame 名称、视觉协议、8 点扫描、
observe/plan/pick/place 流程都保持不变。

## 2. 与机械 Base 简化的关系

`base_T_tool(q_capture)` 直接来自机械 Base FK：

```text
x = planar_2r_fk_x
y = slide + planar_2r_fk_y
z = tcp_height_at_z_zero_mm + z_axis_mm
yaw = shoulder + elbow + rotation
```

视觉链不使用历史 `base_T_slide_zero`，也不使用 `rotation_output_T_tool` 位置平移。Rotation 改变
只影响当前 TCP yaw；不会引入额外 TCP XYZ 偏移。

## 3. 标定与补偿门禁

- `tool_T_camera` 缺失时，Camera 目标解析明确拒绝；
- 非 dry-run 的视觉规划要求 `tool_camera_validated=true`；
- `target_compensation_base_mm` 在 raw Base target 后只应用一次；
- `target_compensation_camera_mm` 在 Camera 检测点上只应用一次；
- Camera 与 Base 补偿不能同时为非零；
- Base TCP workspace 对最终补偿后的目标执行门限；
- arm-local offset workspace 随后只负责 IK 构型与 Slide 选择。

`frame_transforms.json` 中历史 `base_T_slide_zero` 及 `metadata.validated` 不再验证或门禁视觉
Base 坐标。`tool_camera_validated` 仍只表示 TCP→Camera 安装关系的验证状态。

## 4. 时间与静止门禁

```text
全部轴到位且静止
→ 保存五轴 capture state
→ 发出 capture request
→ 收到同 request_id 的 detection
→ 确认状态未变化
→ 用 capture_axis_state 解析 Camera 目标
```

`capture_motion_state` 只有 `STATIONARY` 可解析；`MOVING` 和 `UNKNOWN` 均拒绝。frame_id 必须为
配置的 Camera frame，Camera Z 必须为正且所有坐标有限。

## 5. API 兼容

`/api/vision/plan` 的 Web API（Application Programming Interface，应用程序编程接口）结构不变。
响应继续区分 Camera 输入、raw Base position、Base 补偿和 final Base position；规划结果仍走统一
Base solver。坐标简化不绕过工作区、轴/关节限位、状态门禁或 FK 重建检查。

## 6. 验证边界

离线合成测试验证 Camera→TCP→Base 矩阵组合、补偿只应用一次和 API 规划回归。它不代表现场
相机、深度、TCP 零位高度或抓取精度已经完成硬件验证。
