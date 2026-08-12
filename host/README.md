# Host Python control library

本目录是蘑菇采摘平台的上位机 Python 代码。当前正式实现瓴控
MG4010E-i36 的只读状态查询、有限行程旋转关节位置控制和当前位置保持停止。

首版只发送以下命令：

- `0x94`：读取单圈绝对位置；
- `0x92`：读取当前上电周期多圈坐标；
- `0x9A`：读取电机状态和故障；
- `0x9C`：读取速度、温度、电流和编码器状态；
- `0xA4`：多圈绝对位置闭环控制命令 2及业务层当前位置保持；
- `0x81`：仅供显式维护诊断，正常停止和异常恢复不发送。

首版不实现转矩、连续速度、其他位置模式、参数写入、ROM 写入、自动使能、
自动清错、自动回零、后台监控线程或多关节轨迹规划。

## 分层

```text
host/
├── config/       模型/loader、project 参数、examples 与 ignored local 配置
├── drivers/      CAN 总线、MG4010 协议和电机驱动
├── robot/        弧度制有限行程关节
├── motion/       后续到位等待和运动协调
├── kinematics/   平面二连杆正逆运动学与后续坐标变换
├── tasks/        后续采摘任务状态机
├── scripts/      只读诊断和显式人工测试工具
└── tests/        按领域分组且不连接硬件的离线测试
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
- `Planar2RArmController` 将逆运动学解与肩肘逻辑软限位连接，
  在双关节参数预检查后背靠背下发两条位置命令。

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

如果 `0xA4` 已尝试发送但最终无法确认应答，驱动抛出
`MotorCommandResultUnknownError`。异常明确提示原命令可能已经被电机接收、机械
状态未知；驱动不会自动发送可能释放保持力矩的 `0x81`。

## 首次标定

`config/project/joints.py` 中的肩、肘配置已经写入当前上机测量值。机械安装、联轴器
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

MG4010 backend 只读和维护统一使用正式配置、设备发现和共享 runtime：

```bash
cd host
python scripts/maintenance/mg4010_joint.py basic-parameters --joint shoulder
python scripts/maintenance/mg4010_joint.py initialize --joint shoulder
python scripts/maintenance/mg4010_joint.py state --joint shoulder
python scripts/maintenance/mg4010_joint.py logical-angle --joint elbow --watch --interval 0.5
```

`logical-angle` 是只读诊断入口：它绕过软件限位拒绝，按正式减速比、逻辑零点和方向持续显示
相对逻辑零点的有符号最短角差（范围 `[-180, 180)` 度），并通过 `within_limits` 标明当前位置
是否位于配置的软件限位内。它不会执行初始化、使能或运动；按 `Ctrl+C` 停止持续读取。

单关节 move 默认只预览；真实动作必须显式双确认：

```bash
python scripts/maintenance/mg4010_joint.py move \
  --joint shoulder --position-deg 5 --velocity-deg-s 2

python scripts/maintenance/mg4010_joint.py move \
  --joint shoulder --position-deg 5 --velocity-deg-s 2 \
  --execute --confirm-motion
```

MG4010 `0x81` 仅保留为可能释放保持力矩的原始维护命令，必须显式确认自由运动风险：

```bash
python scripts/maintenance/mg4010_joint.py protocol-stop-0x81 \
  --joint shoulder --execute --confirm-free-motion-risk
```

当前 `shoulder` 为 ID 1、`elbow` 为 ID 2，两者均使用 36:1 减速比。肩关节以
OA=100 度为逻辑零点，逻辑范围为
-65 到 +65 度；肘关节以 OA=158 度为逻辑零点并反向，逻辑范围为
-160 到 +160 度。两者当前最大逻辑关节速度都是每秒 50 度。机械安装或编码器对应关系变化
后必须重新标定零点、方向和限位。

## 肩肘联合控制

Planar 2R 的 FK/IK 数学继续位于 `kinematics/planar_2r.py`，算法测试继续位于 `tests/`。人工
肩肘联合动作统一使用 `scripts/manual_motion.py move-group`，不再维护第二套直接 CAN 实机入口。

## STM32 正式 Host 客户端

`drivers/stm32_motion.py` 将固件子模块的 ASCII machine protocol v2 整理为根项目
正式客户端。`STM32SerialTransport` 只有显式 `open()` 才打开串口；
`STM32MotionClient` 负责 sequence、同步响应、异步 `DONE/ABORT/FAULT`、日志过滤、
状态解析和 timeout。v2 使用 `hardware_ready` 表示 MCU 侧通用 STEP/DIR/ENABLE 资源
可用；轴 fault 为 `NONE/LIMIT/POSITION_INVALID/HARDWARE_OR_CONFIG/HOMING`，不再暴露
StallGuard 或特定驱动芯片状态。协议版本和最大行长通过离线测试与以下固件真值锁定：

```text
firmware/stm32_motion_controller/App/Inc/app_protocol.h
firmware/stm32_motion_controller/App/README.md
```

客户端提供 Slide/Z 的查询、相对/绝对运动、归零、停止、禁用、使能、清错和全停，
以及吸盘查询、吸附、释放和停止。整数 API 使用 µm、µm/s 和 µm/s²；附加的
`move_relative_mm()`/`move_absolute_mm()` 在发送前进行对称单位换算和整数范围检查。
构造客户端不会连接硬件，代码也没有默认串口。

只读串口冒烟检查默认依次发送 `VR`、`QS Z`、`QS S`、`QH` 和 `SQ`：

```bash
python scripts/stm32_protocol_smoke.py /dev/ttyACM0
```

该脚本默认不会归零或运动；`--home`/`--position-mm` 只有同时显式提供
`--allow-motion` 时才可使用。

统一控制器会结合活动命令解释 STM32 的轴状态故障。执行 Slide/Z reference home
（机械归零）且尚未收到 `DONE`、`ABORT` 或 `FAULT` 时，如果同时为 `fault_code=2`
（`POSITION_INVALID`）、`homed=False`、`position_valid=False`，这是尚未建立位置参考的
预期暂态，不能提前判为终态设备故障。普通位置运动中的 `fault_code=2` 仍然是故障。
归零收到 `DONE` 后会重新查询状态，只有 `homed=True`、`position_valid=True`、
`busy=False`、`fault_code=0` 全部成立才返回 `ARRIVED`。

## 按 VID/PID 发现本机硬件

本机硬件配置采用 `config/hardware.py` 中的 frozen dataclass（冻结数据类）。首次使用时
复制模板，实际文件已被 Git 忽略：

```bash
cd host
mkdir -p config/local
cp config/examples/hardware.py config/local/hardware.py
```

当前三类设备身份互不相同：

| role | device | VID:PID | default parameter |
| --- | --- | --- | --- |
| `feetech` | Feetech USB 串口转换器 | `1A86:55D3` | 115200 baud |
| `stm32_motion` | STM32 STLink Virtual COM Port | `0483:374B` | 115200 baud |
| `can_adapter` | gs_usb CAN 适配器 | `1D50:606F` | 1,000,000 bit/s |

第一版只按 VID/PID（Vendor ID / Product ID，厂商标识符/产品标识符）精确匹配。
USB serial number 只保留在枚举结果、错误和诊断输出中，不参与匹配；端口名、USB bus 和
address 同样只是本次启动的运行时信息，不是永久设备身份。macOS、Linux 和 Windows 共用
同一份配置：pySerial 会自然返回 `/dev/cu.*`、`/dev/ttyACM*`、`/dev/ttyUSB*` 或
`COMx`，代码不按操作系统构造路径。`gs_usb` 不是串口，因此不返回串口路径。Windows
使用 `gs_usb` 时需要由部署人员正确安装 WinUSB 兼容驱动，发现代码不会修改系统驱动。

只读诊断命令如下；它们只枚举 USB descriptor 或匹配配置，不打开串口、不启动 CAN、
不设置 bitrate，也不发送任何控制命令：

```bash
cd host
.venv/bin/python scripts/list_hardware_devices.py --list-all
.venv/bin/python scripts/list_hardware_devices.py --resolve
```

默认解析要求 VID/PID 恰好唯一。没有匹配会抛出 `DeviceNotFoundError`；存在多个相同
VID/PID 会抛出 `AmbiguousDeviceError`，程序不会选择第一个候选。调试时可在本机配置中
设置 `port_override`，但覆盖路径仍必须通过 VID/PID 身份验证，否则抛出
`DeviceIdentityMismatchError`。

串口驱动只接收解析出的字符串，仍由调用方显式打开：

```python
from config.hardware import load_local_hardware_config
from drivers.device_discovery import resolve_usb_serial_port
from drivers.feetech_protocol import FeetechBus, FeetechSerialConfig
from drivers.stm32_motion import STM32SerialConfig, STM32SerialTransport

hardware = load_local_hardware_config()

stm32 = resolve_usb_serial_port("stm32_motion", hardware.stm32_motion)
stm32_transport = STM32SerialTransport(
    STM32SerialConfig(stm32.port, hardware.stm32_motion.baudrate)
)

feetech = resolve_usb_serial_port("feetech", hardware.feetech)
feetech_bus = FeetechBus(
    FeetechSerialConfig(feetech.port, hardware.feetech.baudrate)
)

# 后续由明确的运行入口调用 stm32_transport.open()/feetech_bus.open()。
```

CAN 侧可把已经唯一解析的设备注入 `CanMotorBus`。发现阶段不启动设备；只有调用方后续
显式执行 `open()` 时，`python-can` 才按已验证设备的本次 bus/address 打开后端并配置
bitrate：

```python
from drivers.can_bus import CanMotorBus
from drivers.device_discovery import resolve_gs_usb_device

can_adapter = resolve_gs_usb_device("can_adapter", hardware.can_adapter)
can_bus = CanMotorBus(
    interface="gs_usb",
    bitrate=hardware.can_adapter.bitrate,
    gs_usb_device=can_adapter.device,
)

# 后续由明确的运行入口调用 can_bus.open()。
```

为兼容旧工具，`CanMotorBus` 未注入设备时仍保留原有索引扫描方式；配置了设备身份的入口
应始终先调用 resolver，不再无条件使用第一个扫描结果。

> 如果以后增加第二个相同 VID/PID 的设备，再为设备配置增加可选 `serial_number`，将其
> 作为第二级精确匹配条件。当前阶段不要提前引入未使用的复杂匹配逻辑。

## Unified asynchronous point-to-point control

`motion.UnifiedMotionController` 为 `slide`、`z`、`shoulder`、`elbow` 和 `rotation`
提供统一的异步绝对点到点接口。直线轴公开单位为 mm、mm/s、mm/s²，旋转轴公开单位为
deg、deg/s、deg/s²；底层的 µm、rad、电机角和 raw count 不进入运动学或前端边界。

运动学只需生成 `MultiAxisTarget`。`validate_positions()` 可以在不发送控制 I/O 的情况下验证
整组位置、速度、加速度、能力和 Host 限位；`submit_positions()` 复用同一预校验，只有全部
目标合法才按输入顺序背靠背下发并立即返回 group handle。因此后序轴超限不会造成前序轴已经
运动的部分提交。它不等待机械到位。`wait_group()` 轮询所有参与轴，
使用各轴显式注入的 `ArrivalConfig` 判断到位、稳定窗口和 deadline。命令
`accepted=True` 只代表后端接受，不代表 `arrived` 或 `completed=True`。

```python
from motion import AxisName, AxisTarget, MultiAxisTarget

target = MultiAxisTarget(
    targets=(
        AxisTarget(AxisName.SLIDE, 300.0),
        AxisTarget(AxisName.Z, 120.0),
        AxisTarget(AxisName.SHOULDER, 25.0),
        AxisTarget(AxisName.ELBOW, -60.0),
        AxisTarget(AxisName.ROTATION, 90.0),
    )
)

handle = controller.submit_positions(target)
result = controller.wait_group(handle, timeout_s=10.0)
```

若目标省略 velocity/acceleration，构造控制器时必须通过
`default_motion_parameters` 注入已经确认的工程单位默认值；核心层不会从示例或未经验证
的机械参数猜测默认运动 profile。Slide/Z 描述符的位置范围必须由
`MotionRuntimeConfig.linear_position_limits()` 显式注入；速度和加速度上限由
`linear_motion_limits()` 注入。当前 Host 上限为 Slide `72 mm/s`、`180 mm/s²`，Z
`10 mm/s`、`25 mm/s²`；超限返回 `soft_limit`，STM32 firmware 仍独立执行最终硬保护。

This is coordinated point-to-point submission, not interpolated or strictly synchronized motion.

肩、肘同时出现在组目标且两轴都省略 velocity 时，统一控制器会按提交前读取的角度差和
逐轴默认速度上限分配速度，使两轴近似同时到达。该功能仍不是轨迹插补或严格多轴同步；
当前也没有完整采摘状态机。
`startup_position` 不属于普通运动目标，也不会参与坐标换算。Rotation 当前使用尚未完成真机
验证的当前位置保持式软件制动，不是可靠的独立 stop；timeout 不会自动 torque disable。所有真实运动仍要求显式硬件配置、已确认的默认
速度/加速度、逐轴机械验证和现场安全措施；软件 stop 不是硬件急停。

## Mechanical Base / TCP frame chain

公开坐标链现在统一为：

```text
Base -> Slide + Z + Shoulder/Elbow -> TCP / Rotation center -> Camera
```

`geometry.RigidTransform` 使用 `A_T_B`（把 B 转到 A）记号；组合顺序为
`A_T_C = A_T_B @ B_T_C`。机械 FK 直接输出 `base_T_tool`：

```python
from kinematics.five_axis import load_local_five_axis_kinematics

kinematics = load_local_five_axis_kinematics()
base_T_tool = kinematics.forward_kinematics(axis_state)
```

`config/local/five_axis_geometry.json` 只保存真实连杆尺寸和现场测得的
`tcp_height_at_z_zero_mm`，并要求 `geometry_confirmed=true`。Base XY/yaw、Slide +Y、Z +Z 和
Rotation/TCP 同心关系由机械定义固定。历史 Base–Slide-zero 工具保留为审计工具，不参与正常
FK、IK 或视觉规划；这些工具均默认预览/只读，不自动 home、move、stop、enable 或 torque enable：

```bash
.venv/bin/python scripts/calibrate_base_slide_frame.py --help
.venv/bin/python scripts/verify_base_slide_frame.py --help
.venv/bin/python scripts/set_tool_camera_transform.py --help
```

完整坐标约定见 `docs/interfaces/ROBOT_FRAME_CONVENTIONS.md`。历史标定工具说明见
`docs/calibration/BASE_SLIDE_FRAME_CALIBRATION.md`，仅用于兼容和审计。

## Upper motion runtime and safety modes

`create_upper_motion_runtime()` 是三类真实硬件的唯一公共组装入口。它加载调用方提供的
`HardwareConfig` 与 `MotionRuntimeConfig`，调用一次 `resolve_hardware()`，并用解析得到的
STM32/Feetech port 和 gs_usb device 创建 transport、bus、肩肘关节、Rotation、唯一的
内部唯一的 `UnifiedMotionController`。构造会进行设备发现，但不会打开通信、初始化关节、
使能、机械归零、torque enable 或提交运动。当前 MG4010 实机已确认在请求 ID `0x141/0x142`
上原样应答，因此 Runtime 显式启用 `allow_same_id_response`；CAN 层仍校验命令字、帧格式并
排除 gs_usb TX echo。

`runtime.open()` 才按 STM32 transport → CAN bus → Feetech bus 打开通信资源；任一步失败时，
已打开资源按相反顺序回滚。`runtime.close()` 按 Feetech → CAN → STM32 关闭，即使某一资源
关闭失败也会继续关闭其余资源，最后聚合报告错误。打开或关闭通信都不等价于 stop，也不会
自动 torque disable。

运行模式默认为 `READ_ONLY`：允许版本、状态、位置和故障读取，也允许肩肘使用只读
`initialize()` 建立绝对位置解释，但拒绝 `submit_absolute()`、`submit_positions()` 和
`home_reference()`。`MOTION` 必须由调用方显式选择，且只允许后续的显式运动调用继续执行，
本身不会发送命令。Rotation 因没有经过验证的独立 stop，即使在 `MOTION` 下也默认拒绝；
只有额外设置 `allow_unverified_rotation_motion=True` 才能进入现有位置提交逻辑。多轴目标包含
Rotation 时同样执行该门禁。Rotation timeout 会尝试把当前反馈位置写回 goal，但未确认静止时
不会被描述为已停止，也不会以 torque disable 冒充 stop。

到位容差、稳定窗口、轮询周期、timeout、默认速度、默认加速度以及 Slide/Z 的 Host 位置、
速度和加速度上限统一来自被 Git 忽略的 `config/local/motion.py`。当前本机线性范围同步 STM32
firmware 软限位：Slide `0..799.988 mm`、Z `-190..0 mm`；固件仍独立执行同一底层保护。
当前运行默认值为 Slide `60 mm/s`、`180 mm/s²`，Z `8 mm/s`、`25 mm/s²`，Shoulder 和
Elbow 均为 `30 deg/s`；它们不等于允许上限。完成整机有效行程验收后，应同时更新 firmware
与本地配置，避免 Host 和下位机范围不一致。
先复制 `config/examples/motion.py` 到 `config/local/motion.py`，再替换其中明确标为
`EXAMPLE / BENCH-TEST PLACEHOLDER`、`NOT PRODUCTION-CALIBRATED` 的数值。Rotation 的工程
速度/加速度映射尚未验证，因此示例保持 `None`。同一时刻只允许一个进程拥有这些真实硬件；
Web 后端、运动学执行器和台架工具都应复用同一个 runtime，不再各自扫描和组装后端。

```python
from bootstrap import create_upper_motion_runtime
from config.hardware import load_local_hardware_config
from config.motion_runtime import load_local_motion_config
from motion import AxisName, RuntimeMode

hardware_config = load_local_hardware_config()
motion_config = load_local_motion_config()

runtime = create_upper_motion_runtime(
    hardware_config,
    motion_config,
    mode=RuntimeMode.READ_ONLY,
)

with runtime:
    state = runtime.controller.get_state(AxisName.SHOULDER)
```

Entering the runtime context only opens communication resources.
It does not enable actuators, home axes, or issue motion commands.

长期人工控制和后端维护入口：

| Need | Command |
| --- | --- |
| 五轴统一状态和人工运动 | `scripts/manual_motion.py` |
| STM32/Slide/Z/Vacuum 维护 | `scripts/maintenance/stm32_motion.py` |
| Shoulder/Elbow/MG4010 维护 | `scripts/maintenance/mg4010_joint.py` |
| Rotation/Feetech 维护 | `scripts/maintenance/feetech_rotation.py` |
| USB 设备枚举 | `scripts/list_hardware_devices.py` |
| Base–Slide 标定 | `scripts/calibrate_base_slide_frame.py` |
| Tool–Camera 录入 | `scripts/set_tool_camera_transform.py` |

日常人工控制优先使用 `manual_motion.py`。只有排查特定 backend protocol、原始状态或 power
语义时才使用 maintenance 脚本。完整安全语义见
`docs/handoffs/UPPER_MOTION_DEBUG_CLI_GUIDE.md`。

默认五轴只读联合检查：

```bash
cd host
.venv/bin/python scripts/manual_motion.py inspect
```

`inspect` 没有 `--execute`，只读取 STM32 version、五轴 descriptor/capability 和逻辑状态。
真实硬件的分阶段验证顺序见 `docs/handoffs/UPPER_MOTION_RUNTIME_TEST_GUIDE.md`。

Slide/Z 的统一接口单轴归零使用独立受控入口。默认命令只读取所选轴状态；真实归零必须同时
提供两个显式开关，并且一次只能选择一个轴：

```bash
cd host

# READ_ONLY：只查询 Slide，不使能或运动
.venv/bin/python scripts/manual_motion.py home --axis slide

# MOTION：现场确认安全条件后，显式执行一次 Slide 机械归零
.venv/bin/python scripts/manual_motion.py home --axis slide \
  --execute \
  --confirm-home-motion
```

Z 使用 `--axis z`，默认 timeout 为 120 秒；Slide 默认 15 秒，也可用正数 `--timeout` 覆盖。
程序通过 `controller.home_reference()` 调用统一接口，只有结果为 `ARRIVED`，且最终
`homed=True`、`position_valid=True`、`busy=False`、`faulted=False` 才返回成功。controller
返回 terminal timeout/fault/abort 后 CLI 不重复 stop；仅在提交可能已发生但尚无 terminal result
的异常或 `Ctrl+C` 中最多尝试一次软件 stop。执行前还要求通信已连接、`busy=False`，且不存在
除 `fault_code=2`（预期的未归零位置无效状态）之外的轴故障。Runtime context close 仍不等于
stop，软件 stop 也不是 disable、断电或硬件急停。

### Guarded axis-subset point-to-point test

`scripts/manual_motion.py move-group` 通过同一个 `UpperMotionRuntime` 和
`UnifiedMotionController.submit_positions()` 提交用户显式指定的任意轴子集。它采用稳定轴顺序
`slide, z, shoulder, elbow, rotation`，不会给未指定轴补当前位置或保持目标。该入口是协调
点到点运动（Coordinated Point-to-Point Motion），不是轨迹插补或严格同步，也不验证轴间
路径无碰撞。肩肘同时参与且两者都未显式指定速度时，会按角度差自动分配默认速度以近似同时
到达；任一肩肘速度显式给出时不启用该计算。

至少一个轴目标必须显式给出；timeout 和本次调用允许的最大线性/旋转位移可按需要覆盖。各轴
速度与加速度可显式提供，是否支持仍由 descriptor 和统一 controller 校验。默认不运动，只打开
Runtime、对参与的 Shoulder/Elbow 执行只读绝对位置初始化，并检查参与轴连接、空闲、位置有效、
故障、Slide/Z 归零、Shoulder/Elbow enabled、目标软限位和目标位移：

```bash
cd host
.venv/bin/python scripts/manual_motion.py move-group --help
```

实际命令格式如下；尖括号内容必须替换为经过当前机构姿态和碰撞空间确认的数值，不能直接复制
执行：

```bash
.venv/bin/python scripts/manual_motion.py move-group \
  --slide <safe-slide-target> \
  --z <safe-z-target> \
  --shoulder <safe-shoulder-target> \
  --elbow <safe-elbow-target> \
  --slide-velocity <optional-positive-speed> \
  --slide-acceleration <optional-positive-acceleration> \
  --z-velocity <optional-positive-speed> \
  --z-acceleration <optional-positive-acceleration> \
  --shoulder-velocity <optional-positive-speed> \
  --elbow-velocity <optional-positive-speed> \
  --timeout <positive-seconds> \
  --max-linear-delta-mm <verified-per-axis-delta> \
  --max-rotary-delta-deg <verified-per-axis-delta>
```

先用上述默认 READ_ONLY 模式检查打印出的当前值、目标、delta、运动参数和上限。真实提交必须
追加普通双确认：

```text
--execute
--confirm-motion
```

目标包含 Rotation 时还必须追加：

```text
--allow-rotation-motion
--confirm-rotation-no-stop
--enable-rotation-torque
```

执行模式会先把 Rotation goal 预置为当前角度，再显式 torque enable，然后才提交轴子集目标。
脚本结束后不会自动 torque disable；Rotation 当前使用尚未经过真机验证的当前位置保持式
software stop，发生异常时仍必须准备使用物理急停。肩肘使用 `A4` 当前位置保持且不回退到 `0x81`；
其余轴在等待被异常中断时会各尝试一次协调停止；
group failure/timeout 则复用统一控制器自身的 stop 策略，不重复发送 stop。

### 应用入口与内部控制器边界

正常 CLI、GUI 和外部应用只调用 `MushroomRobotService`。历史 `FrontendMotionInterface`、
`KinematicsMotionInterface` 及对应 façade 已删除，`UpperMotionRuntime` 不再提供
`frontend_motion` / `kinematics_motion`。`UnifiedMotionController` 保留为内部实现，校准、
维护和硬件诊断脚本可直接使用同一个 `runtime.controller`，但它不是应用层正式入口。

## Feetech 末端旋转轴

`drivers/feetech_protocol.py` 提供帧编码、状态包验证、timeout、显式 open/close 和
可注入串口；`robot/feetech_rotation.py` 提供有限行程角度换算、反馈读取、位置命令
和显式 torque enable/disable。位置命令按 Feetech 磁编码协议从 `0x2A` 连续写入
`position/time/speed` 六字节。

`config/project/feetech.py` 包含两层配置：`SM45BL_C001_PROFILE` 固化型号、飞特自定义串口协议、
RS-485 半双工、USB 转换板自动收发切换、115200 baud、12-bit/4096 counts 磁编码器和
C001 寄存器表；`END_EFFECTOR_ROTATION_CONFIG` 固化当前项目安装参数：ID 1、
`zero_raw=2130`、`direction_sign=+1`（逻辑正方向为 `+X`）、`-150°..+150°` 当前限位和
`max_speed_raw=500`。默认位置命令速度同为 500 raw。限位和速度是当前调试配置，仍需
随最终机构负载完成机械验收。设备 port 不固化，由 hardware local config 和 VID/PID 设备发现
解析；写指令 status packet 行为也仍需实机复验。

Feetech backend 维护入口复用正式配置和 runtime 的设备发现，不再接受本机 port 参数：

```bash
cd host
.venv/bin/python scripts/maintenance/feetech_rotation.py ping
.venv/bin/python scripts/maintenance/feetech_rotation.py state
.venv/bin/python scripts/maintenance/feetech_rotation.py feedback
```

位置命令默认预览，真实执行还需确认 Rotation 无可靠独立 stop：

```bash
.venv/bin/python scripts/maintenance/feetech_rotation.py move --position-deg 5
.venv/bin/python scripts/maintenance/feetech_rotation.py move \
  --position-deg 5 --execute --confirm-motion --confirm-rotation-no-stop \
  --enable-torque
```

命令完成后 torque 状态不会自动改变。明确需要下力时使用 `torque-disable --execute
--confirm-free-motion-risk`；该动作可能使机构自由转动或失去保持力，不是 stop。
2026-08-03 已在 ID 1 实机完成 ping 和 `0x38` raw-position 只读测试，连续读数均为
`2047`；随后用户确认逻辑正方向正确，机械零点最终微调为 `zero_raw=2130`。当前
`±45°` 和 500 raw 已固化为调试约束，最终机械限位和负载安全速度仍待验收。

## 顶层 Robot Service

统一入口为 `scripts/robot_service.py`：

```bash
cd host
.venv/bin/python scripts/robot_service.py --mode read-only
.venv/bin/python scripts/robot_service.py --mode dry-run --fake-position 0 0 100
```

`read-only` 和 `dry-run` 都不执行设备发现或构造硬件 runtime；dry-run 使用正式几何、Base solver、TrayWorkspace、OffsetWorkspace 和 RobotMotionEnvelope 进行纯算法规划。`execute` 沿用现有 MotionAuthorization，并要求：

```bash
.venv/bin/python scripts/robot_service.py --mode execute \
  --confirm-motion --confirm-rotation-no-stop
```

不要在自动测试中运行 execute。当前本机 hand-eye missing，且没有 validated grasp profile；`observe` 可显示 Fake/Socket Camera observation，`plan-observation` 和 `pick` 会明确 fail-closed。详见 `docs/interfaces/ROBOT_SERVICE_RUNTIME.md`、`VISION_GATEWAY_PROTOCOL.md` 和 `PICK_WORKFLOW.md`。

Service 还提供 `axes`、`axis state/states`、`axis move-abs` 和 `axis move-rel`。这些是
raw/manual maintenance operations：只执行所选轴自身状态与软限位门禁，不经过 TrayWorkspace、
Base-frame IK、OffsetWorkspace、side-switch clearance 或碰撞路径检查。相对增量基于调用时读取的
当前有效逻辑位置，并在 controller 提交锁内解析为绝对目标；零增量在到位容差内直接完成且不提交硬件命令。
