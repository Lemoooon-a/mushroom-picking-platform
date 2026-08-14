# 平面二连杆运动学

`Planar2RKinematics` 提供肩关节、肘关节组成的平面二旋转关节（2R）正逆运动学。
模块是纯数学计算，不导入 CAN 驱动，也不会访问电机。

坐标和关节约定：

- x 向前、y 向左、z 向上；
- 从 +z 方向俯视，正角按右手定则从 +x 转向 +y；
- 肩角相对全局 +x；
- 肘角相对 link1，肘角为 0 时 link2 与 link1 同向；
- 肩、肘均为 0 时，两根连杆都沿 +x。

连杆长度在构造时传入，可以使用 mm、m 或其他单位；输出坐标沿用相同单位，
角度统一使用 rad。

```python
from kinematics import Planar2RKinematics

kinematics = Planar2RKinematics(link1_length=300.0, link2_length=250.0)
point = kinematics.forward(shoulder_rad=0.0, elbow_rad=0.0)
solutions = kinematics.inverse(x=point.x, y=point.y)
```

逆运动学返回全部主值数学解，不应用当前肩、肘软件限位。调用层应根据
`JointConfig.min_position_rad` 和 `JointConfig.max_position_rad` 过滤结果。

## 参数化五轴正运动学

`five_axis.FiveAxisKinematics` 在上述 Planar 2R 外组合 Slide、Z 和 Rotation，直接返回
`base_T_tool`：

```text
x = planar_2r_fk_x(shoulder, elbow)
y = slide_mm + planar_2r_fk_y(shoulder, elbow)
z = tcp_height_at_z_zero_mm + z_mm
yaw = shoulder_deg + elbow_deg + rotation_deg
```

Rotation 只影响 yaw，不影响 TCP XYZ；不再使用外部 Base→Slide-zero XY/yaw 或
Rotation→TCP 位置偏移。当前机械臂正式几何从 `config/robot_geometry.json` 加载；仓库不提供
local 或 example 几何模板。
该 example 故意把未知尺寸设为 `null` 且保持
`geometry_confirmed=false`，避免示例数值进入真实标定。

默认标定 provider 为：

```text
kinematics.five_axis:load_robot_five_axis_kinematics
```
