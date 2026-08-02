# Motion

当前 `robot.planar_arm.Planar2RArmController` 已提供最小的肩肘双关节调用：
逆运动学解先通过两关节逻辑软限位筛选，两个命令参数全部验证后，
再依次快速发送肩、肘 `0xA4`。任一下发失败时会尽力对两关节发送
`0x81`。

这是背靠背下发，不是硬件时钟同步或轨迹插补。
`CanRotaryJoint.command_position()` 只确认通信应答，不等待机械到位。
到位等待、运动超时、严格同步和轨迹执行仍未实现。

`capabilities.py` 仅记录 Slide、Z、shoulder、elbow、rotation 和 vacuum 当前可证明的
能力，并提供零动作的 `UpperMotionBackends` 聚合；它不是 coordinator，也不会在构造时
初始化或控制硬件。后续协调层必须显式处理各后端不对称的 homing、position valid、
arrival event、timeout 和 disable 语义。
