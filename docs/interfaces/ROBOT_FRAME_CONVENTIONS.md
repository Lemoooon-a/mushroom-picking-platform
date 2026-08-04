# 机器人坐标系约定

## 1. 公开根与坐标链

当前系统不引入 World frame。Base frame 是视觉、前端、标定、公开正运动学
（Forward Kinematics, FK）和后续逆运动学（Inverse Kinematics, IK）目标的根：

```text
Base (B)
  -> Slide-zero (S)
  -> current five-axis kinematics
  -> Tool / TCP (T)
  -> Camera (C)
```

- **Base**：机器人对外的工作坐标根。
- **Slide-zero**：Slide 和 Z 完成机械归零、逻辑位置均为 0 时，内部几何模型的参考坐标；
  它不是 World、startup position 或视觉参考坐标。
- **Tool**：原点位于吸盘工具中心点（Tool Center Point, TCP），安装在 Rotation 输出侧，
  因此随 Rotation 转动。
- **Camera**：刚性安装在 Rotation 输出侧，通过固定六自由度外参关联 Tool。

仓库现在提供参数化五轴 FK，但仍没有 URDF（Unified Robot Description Format，统一机器人
描述格式）或当前机器的实际连杆长度。`five_axis_geometry.local.json` 必须由机械负责人填写并
确认 Tool 右手轴方向、Rotation 零角语义、TCP 相对 Rotation 输出的固定几何，然后显式设置
`geometry_confirmed=true`；实现不会猜测这些参数。

内置五轴模型把 Rotation output frame 定义为：原点在第二连杆末端，Shoulder/Elbow/Rotation
均为 0 时，其 x/y/z 与平面运动学 frame 一致；运行时绕局部 `+z` 旋转
`shoulder + elbow + rotation`。Tool frame 通过配置的 `rotation_output_T_tool` 接到该输出。

## 2. 变换记号和组合方向

统一使用：

```text
A_T_B
```

它把 B 中表达的点或位姿转换到 A。组合从右向左应用：

```python
A_T_C = A_T_B @ B_T_C
```

固定变换：

```text
base_T_slide_zero
tool_T_camera
```

运行时变换：

```text
slide_zero_T_tool(q)
base_T_tool(q)
base_T_camera(q)
```

其中：

```text
q = [slide, z, shoulder, elbow, rotation]
```

## 3. 单位和旋转约定

- 平移和点：mm；
- 对外角度：deg；
- 内部三角函数：rad；
- yaw：绕 `+z` 的右手正旋转；
- RPY（Roll-Pitch-Yaw，横滚-俯仰-偏航）输入顺序为 roll、pitch、yaw；
- 旋转矩阵统一为固定轴组合：

  ```text
  R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
  ```

这等价于对列向量先应用 roll，再应用 pitch，最后应用 yaw。

`RigidTransform` 构造时拒绝非有限值、非法齐次末行、非正交旋转和 determinant 为负的
reflection；不会静默修复严重非法矩阵。

## 4. Base-root FK 与 Camera 点

内部 FK 仍以 Slide-zero 为根：

```text
base_T_tool(q)
  = base_T_slide_zero @ slide_zero_T_tool(q)
```

Camera 与 Tool 刚性连接：

```text
base_T_camera(q)
  = base_T_tool(q) @ tool_T_camera
```

Camera 点转换到 Base：

```text
point_base
  = base_T_camera(q) @ point_camera
```

代码中由 `RobotFrameChain.transform_camera_point_to_base()` 集中完成；前端和视觉模块不应
自行拼固定变换。

## 5. Base IK 目标转换

Base 中的目标先转换到内部 Slide-zero：

```text
slide_zero_T_base = inverse(base_T_slide_zero)

target_slide_zero_T_tool
  = slide_zero_T_base @ target_base_T_tool
```

`RobotFrameChain.transform_base_target_to_slide_zero()` 提供该薄包装。仓库当前只有 Planar 2R
的 XY IK，没有完整五轴 IK，因此本轮不虚构全机构逆解。

## 6. startup position 隔离

startup position 只属于系统上电初始化；它不是坐标系，也不进入：

- `base_T_slide_zero`；
- FK/IK；
- Tool/Camera 位姿；
- Base 目标转换；
- 视觉目标位置。

禁止对运动学目标加减 startup position。肩、肘、Rotation 的逻辑零点、方向、减速比和
raw count 转换继续由现有关节/驱动层负责。

## 7. Rotation 职责边界

Frame chain 只计算当前位姿。若未来需要保持 Tool 或 Camera 在 Base 中的指定 yaw，应由
运动学层根据 shoulder、elbow、rotation 求解目标逻辑角；该逻辑不进入统一控制器、驱动或
Base–Slide-zero 标定。
