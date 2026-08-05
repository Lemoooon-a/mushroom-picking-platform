"""肩关节和肘关节上机标定配置。

当前零点、方向和软限位来自上机测量：

- 肩关节输出轴绝对角 100 度为逻辑 0 度，逻辑范围 -65 到 +65 度；
- 肘关节输出轴绝对角 158 度为逻辑 0 度，方向反转，逻辑范围 -160 到 +160 度；
- 软件限位始终使用逻辑关节语义，不直接使用输出轴绝对角；
- 肩关节和肘关节各自保存完整参数，互不共享可调配置。

机械安装、联轴器位置或编码器对应关系变化后，必须重新标定这些值。
"""

from __future__ import annotations

import math

from robot.joint import JointConfig


SHOULDER_JOINT_CONFIG = JointConfig(
    name="shoulder",  # 关节名称，用于日志、命令行选择和错误定位。
    motor_id=1,  # 肩关节电机的 CAN 节点 ID。
    gear_ratio=36.0,  # 电机转角到输出轴转角的减速比。
    direction_sign=1,  # 输出轴绝对角增大时逻辑关节角也增大。
    encoder_zero_output_deg=100.0,  # 输出轴绝对角 100 度标定为逻辑关节 0 度。
    min_position_rad=math.radians(-65.0),  # OA=35 度，对应最小逻辑角 -65 度。
    max_position_rad=math.radians(65.0),  # OA=165 度，对应最大逻辑角 +65 度。
    max_velocity_rad_s=math.radians(50.0),  # 最大逻辑关节速度为每秒 50 度。
    position_tolerance_rad=math.radians(0.1),  # 误差不超过 0.1 度时不重复发位置命令。
    moving_velocity_threshold_rad_s=math.radians(0.05),  # 超过每秒 0.05 度视为仍在运动。
)


ELBOW_JOINT_CONFIG = JointConfig(
    name="elbow",  # 关节名称，用于日志、命令行选择和错误定位。
    motor_id=2,  # 肘关节电机的 CAN 节点 ID。
    gear_ratio=36.0,  # 电机转角到输出轴转角的减速比。
    direction_sign=-1,  # 输出轴绝对角减小时逻辑关节角增大。
    encoder_zero_output_deg=158.0,  # 输出轴绝对角 158 度标定为逻辑关节 0 度。
    min_position_rad=math.radians(-160.0),  # OA=318 度，对应最小逻辑角 -160 度。
    max_position_rad=math.radians(160.0),  # OA=358 度，对应最大逻辑角 +160 度。
    max_velocity_rad_s=math.radians(50.0),  # 最大逻辑关节速度为每秒 50 度。
    position_tolerance_rad=math.radians(0.1),  # 误差不超过 0.1 度时不重复发位置命令。
    moving_velocity_threshold_rad_s=math.radians(0.05),  # 超过每秒 0.05 度视为仍在运动。
)

JOINT_CONFIGS = {
    SHOULDER_JOINT_CONFIG.name: SHOULDER_JOINT_CONFIG,
    ELBOW_JOINT_CONFIG.name: ELBOW_JOINT_CONFIG,
}
