# 机器人坐标约定

## 1. 坐标链

公开根和运动学根统一为机械 Base：

```text
Base
├── Slide +Y
├── Z +Z（逻辑向下为负）
├── Shoulder/Elbow planar 2R
└── TCP / Rotation axis center
    └── Camera
```

Slide=0 时，Base XY、肩关节平面原点 XY 和 Slide 零位机械 XY 重合；Base 相对肩肘平面的
roll、pitch、yaw 都为 0。Rotation 输出轴中心和 TCP 位置同心。

## 2. 变换记号

`A_T_B` 把 B 中表达的点或位姿转换到 A，组合顺序为：

```text
A_T_C = A_T_B @ B_T_C
```

`tool_T_camera` 中的 `tool` 就是 TCP frame。该名称为兼容现有配置保留。

## 3. 五轴 FK

逻辑状态：

```text
q = [slide_mm, z_mm, shoulder_deg, elbow_deg, rotation_deg]
```

机械公式：

```text
tcp_base_x = planar_2r_fk_x(shoulder, elbow)
tcp_base_y = slide_mm + planar_2r_fk_y(shoulder, elbow)
tcp_base_z = tcp_height_at_z_zero_mm + z_mm
tcp_base_yaw = shoulder_deg + elbow_deg + rotation_deg
```

Rotation 只改变 yaw，不改变 TCP XYZ。`z_mm=0` 时 TCP Base Z 必须严格等于
`tcp_height_at_z_zero_mm`；`z_mm=-50` 时 TCP 下降 50 mm。

## 4. 五轴 IK

对 Base TCP 目标和固定 Slide 候选：

```text
arm_local_x = target_base_x
arm_local_y = target_base_y - slide_mm
z_mm = target_base_z - tcp_height_at_z_zero_mm
```

`arm_local_x/y` 进入既有 Planar 2R IK；Rotation 复用
`rotation_deg = target_yaw - shoulder_deg - elbow_deg`，再枚举软限位内的 360° 周期等价值。
每个候选都必须通过轴/关节限位和完整 FK 重建。

## 5. 视觉链

机器人静止采集并保存 `q_capture` 后：

```text
base_T_target_raw
  = base_T_tool(q_capture)
  @ tool_T_camera
  @ compensated_camera_T_target

base_target_final.xyz
  = base_target_raw.xyz
  + target_compensation_base_mm
```

补偿只应用一次。相机安装变换和视觉协议不因机械 Base 简化而改变。

## 6. 配置与历史兼容

正式坐标核心参数：

- `tcp_height_at_z_zero_mm`；
- `tool_T_camera`；
- Base TCP workspace 的 XYZ min/max；
- `target_compensation_base_mm`。
- `target_compensation_camera_mm`（与 Base 补偿不能同时非零）。

连杆长度、关节零位、轴/关节软限位继续保留。历史 `base_T_slide_zero`、其 validation metadata
和 Base 标定脚本可供审计，但正常 FK、IK、视觉目标转换和规划不读取其数值。
`rotation_output_T_tool` 已从正式五轴几何配置移除。

## 7. 安全边界

Base TCP workspace 是最终普通任务目标的绝对 Base XYZ 门限；arm-local workspace 只用于
Shoulder/Elbow IK 构型和 Slide 候选，两者不能互换。离线 FK/IK、工作区和软限位通过不等于
完成真实机构验证。
