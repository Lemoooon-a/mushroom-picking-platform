# Host Python control library

本目录是蘑菇采摘平台的上位机 Python 代码。当前正式实现瓴控
MG4010E-i36 的只读状态查询、有限行程旋转关节位置控制和软件停止。

首版只发送以下命令：

- `0x94`：读取单圈绝对位置；
- `0x92`：读取当前上电周期多圈坐标；
- `0x9A`：读取电机状态和故障；
- `0x9C`：读取速度、温度、电流和编码器状态；
- `0xA4`：多圈绝对位置闭环控制命令 2；
- `0x81`：软件停止。

首版不实现转矩、连续速度、其他位置模式、参数写入、ROM 写入、自动使能、
自动清错、自动回零、后台监控线程或多关节轨迹规划。

## 分层

```text
host/
├── config/       关节标定配置模板
├── drivers/      CAN 总线、MG4010 协议和电机驱动
├── robot/        弧度制有限行程关节
├── motion/       后续到位等待和运动协调
├── kinematics/   平面二连杆正逆运动学与后续坐标变换
├── tasks/        后续采摘任务状态机
├── scripts/      只读诊断和显式人工测试工具
└── tests/        不连接硬件的离线单元测试
```

各层职责如下：

- `CanMotorBus` 打开和关闭 `python-can` 设备，用一把可重入锁串行化完整请求/
  应答事务，并负责清理旧帧、超时、重试、帧格式和应答匹配。肩关节和肘关节
  必须共享同一个实例，否则两个接收者可能取走对方的回复。
- `mg4010_protocol` 只处理命令字、CAN 标识符、8 字节数据、小端序和协议原生
  数据类型，不包含减速比、机械方向、软件零点或软限位。
- `MG4010Driver` 表示一个电机 ID，对外使用电机侧 degree 和 degree/s，隐藏
  `DATA[0]` 到 `DATA[7]`。它不自行创建 CAN 总线。
- `CanRotaryJoint` 使用输出轴 rad 和 rad/s，负责减速比、方向、软件零点、
  有限行程角度解析和软限位。
- `Planar2RKinematics` 是不依赖 CAN 的纯计算层，使用 x 前、y 左、z 上右手
  坐标系，提供肩肘二连杆的 XY 平面正逆运动学。

## 四层角度

`0x94` 返回 `circle_angle_raw`，协议单位是 `0.01°/LSB`。对于 36:1 型号：

```text
motor_cycle_deg = circle_angle_raw / 100
output_abs_deg = wrap_360(motor_cycle_deg / gear_ratio)
```

`output_abs_deg` 是 `[0°, 360°)` 内的输出轴绝对角。业务层使用的
`joint_position_rad` 由已标定的 `encoder_zero_output_deg`、方向和有限行程唯一
解析：

```text
output_delta_deg = output_abs_deg - encoder_zero_output_deg + 360 * k
candidate_rad = direction_sign * radians(output_delta_deg)
```

实现枚举 `k ∈ {-2, -1, 0, 1, 2}`，只接受位于
`[min_position_rad, max_position_rad]` 的候选值。合法候选必须恰好有一个：没有
候选说明当前绝对位置越界，多个候选说明配置有歧义。该算法不能替换为固定的
`wrap_to_pi`，因为关节有限行程可能跨越编码器 `0°/360°`，也可能宽于 π，
但配置宽度必须严格小于 `2π`。

反向转换为：

```text
output_abs_deg = wrap_360(
    encoder_zero_output_deg + direction_sign * degrees(joint_position_rad)
)
```

`direction_sign=+1` 表示输出轴绝对角增加时逻辑关节角增加，`-1` 表示逻辑方向
相反。

## 为什么绝对位置使用 0x94

实测表明，重新上电后 `0x92` 可能根据当前 `0x94` 初始化到距离零点最近的等效
有符号角。同一个机械位置在不同上电周期的 `0x92` 可能相差：

```text
360° * 36 = 12960°
```

因此有限行程关节以 `0x94` 为跨重启位置真值。`0x92` 只表示当前上电周期中
`0xA4` 使用的多圈控制坐标，不能把固定的 `0x92` 值持久化为机械零点。

`encoder_zero_output_deg` 是项目软件零点，不执行 `0x19`，也不写入电机 ROM。

## 动态构造 0xA4 目标

`CanRotaryJoint.command_position(position_rad, velocity_rad_s)` 按以下顺序执行：

1. 在任何 CAN 请求前检查参数有限、目标在软限位内且速度为有限正值；
2. 读取当前 `0x94`，通过有限行程候选解析得到当前逻辑关节角；
3. 读取 `0x9A` 和 `0x9C`，发现故障、协议明确关闭状态或未知运行状态时拒绝运动；
4. 若目标与当前位置之差在容差内，不发送 `0xA4`；
5. 若电机仍在运动则拒绝重提交；读取当前 `0x92` 后再次读取 `0x94`，只有
   两次绝对位置样本稳定时才建立本次命令快照；
6. 计算逻辑位移和电机侧增量：

   ```text
   delta_joint_rad = target_position_rad - current_position_rad
   motor_delta_deg = direction_sign * degrees(delta_joint_rad) * gear_ratio
   target_motor_multi_turn_deg = current_motor_multi_turn_deg + motor_delta_deg
   ```

7. 按当前协议解释和实测结果换算速度：

   ```text
   max_motor_speed_deg_s = degrees(velocity_rad_s) * gear_ratio
   ```

8. 构造并发送 `0xA4`，确认正确通信应答后立即返回。

`command_position()` 不等待机械到位。后续 `motion/` 层负责到位等待、运动超时、
故障停止、多关节协调和轨迹执行。

如果 `0xA4` 已尝试发送但最终无法确认应答，驱动会尽力发送一次 `0x81`，再抛出
`MotorCommandResultUnknownError`。异常明确提示原命令可能已经被电机接收，机械
状态未知。

## 首次标定

`config/joints.py` 中的肩、肘配置已经写入当前上机测量值。机械安装、联轴器
位置或编码器对应关系变化后，必须重新执行以下标定流程：

1. 只读连接电机；
2. 手动将关节放到逻辑零位；
3. 读取 `0x94` 换算后的 `output_abs_deg`，填入 `encoder_zero_output_deg`；
4. 小角度手动转动，确定 `direction_sign`；
5. 测量并填写最小、最大软限位；
6. 先做离线 dry-run；
7. 再做低速、小角度、空载测试；
8. 确认机械方向、限位和急停后才进入正常控制。

## 命令示例

只读查询；实机固件若回复在请求 ID 上，必须显式开启兼容：

```bash
cd host
python scripts/read_motor_basic_params.py \
  --motor-id 1 \
  --allow-same-id-response
```

完全离线 dry-run；示例数字仅演示接口，不是肩关节或肘关节标定值：

```bash
python scripts/test_joint_position.py \
  --motor-id 1 \
  --target-rad 0.10 \
  --velocity-rad-s 0.05 \
  --gear-ratio 36 \
  --encoder-zero-output-deg 350 \
  --direction-sign 1 \
  --min-position-rad -0.35 \
  --max-position-rad 0.70 \
  --current-circle-angle-raw 1260000 \
  --current-multi-turn-deg -3562.5 \
  --dry-run
```

显式允许运动的形式如下。执行前必须用真实标定值替换示例，并确认机械区域安全；
仅阅读文档或运行自动测试不会执行该命令：

```bash
python scripts/test_joint_position.py \
  --motor-id 1 \
  --target-rad 0.10 \
  --velocity-rad-s 0.05 \
  --gear-ratio 36 \
  --encoder-zero-output-deg YOUR_CALIBRATED_ZERO \
  --direction-sign YOUR_CALIBRATED_DIRECTION \
  --min-position-rad YOUR_CALIBRATED_MIN \
  --max-position-rad YOUR_CALIBRATED_MAX \
  --allow-same-id-response \
  --enable-motion
```

上机配置也可以直接按名称选择。当前 `shoulder` 为 ID 1、`elbow` 为 ID 2，
两者均使用 36:1 减速比。肩关节以 OA=100 度为逻辑零点，逻辑范围为
-60 到 +70 度；肘关节以 OA=158 度为逻辑零点并反向，逻辑范围为
-152 到 +152 度。两者当前最大逻辑关节速度都是每秒 50 度：

```bash
python scripts/test_joint_position.py \
  --joint shoulder \
  --target-rad 0.10 \
  --velocity-rad-s 0.05 \
  --allow-same-id-response
```

不加 `--enable-motion` 时只生成在线预览。机械安装或编码器对应关系变化后，
必须重新标定零点、方向和限位。

`0x81` 是软件停止，不替代切断动力或使能的独立硬件急停。角度正方向、A4 速度
换算和同 ID 应答兼容来自当前 MG4010E-i36 协议解释及实测，换电机型号或固件后
仍需重新验证。
