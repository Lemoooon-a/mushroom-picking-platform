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

单独失能某个关节时，使用 `--disable` 发送 `0x81` 电机关闭命令：

```bash
python scripts/test_joint_position.py \
  --joint shoulder \
  --disable \
  --allow-same-id-response \
  --raw
```

`--disable` 不需要 `--enable-motion`，且不能与目标位置、速度或
`--dry-run` 同时使用。该命令只失能 `--joint` 选中的电机，不替代硬件急停。

`0x81` 是软件停止，不替代切断动力或使能的独立硬件急停。角度正方向、A4 速度
换算和同 ID 应答兼容来自当前 MG4010E-i36 协议解释及实测，换电机型号或固件后
仍需重新验证。

## 肩肘双关节 XY 测试

`scripts/test_planar_2r_motion.py` 将末端 XY 目标转换为肩肘逻辑角，
并使用当前标定软限位筛选解。不加 `--enable-motion` 时完全离线，
不打开 CAN。下面的 300/250 只是演示尺寸，上机前必须替换为实际
`L1/L2`；连杆长度和 `x/y` 必须使用同一单位：

```bash
python scripts/test_planar_2r_motion.py \
  --link1-length 300 \
  --link2-length 250 \
  --x 511.948 \
  --y 177.094 \
  --elbow-branch positive \
  --velocity-rad-s 0.1745329252
```

确认预览中的肩肘角、软限位和机械空间后，在同一条命令末尾加上：

```bash
  --allow-same-id-response \
  --raw \
  --enable-motion
```

肩和肘共享一个 `CanMotorBus`，两条 `0xA4` 依次快速发送。
这可用于空载联动测试，但不是严格同步轨迹，也不保证两关节同时到位。

## STM32 正式 Host 客户端

`drivers/stm32_motion.py` 将固件子模块的 ASCII machine protocol v1 整理为根项目
正式客户端。`STM32SerialTransport` 只有显式 `open()` 才打开串口；
`STM32MotionClient` 负责 sequence、同步响应、异步 `DONE/ABORT/FAULT`、日志过滤、
状态解析和 timeout。协议版本和最大行长通过离线测试与以下固件真值锁定：

```text
firmware/stm32_motion_controller/App/Inc/app_protocol.h
firmware/stm32_motion_controller/App/README.md
```

客户端提供 Slide/Z 的查询、相对/绝对运动、归零、停止、禁用、使能、清错和全停，
以及吸盘查询、吸附、释放和停止。构造客户端不会连接硬件，代码也没有默认串口。

## 按 VID/PID 发现本机硬件

本机硬件配置采用 `config/hardware.py` 中的 frozen dataclass（冻结数据类）。首次使用时
复制模板，实际文件已被 Git 忽略：

```bash
cd host
cp config/hardware_local.example.py config/hardware_local.py
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

运动学只需生成 `MultiAxisTarget`。`submit_positions()` 先完整验证目标，再按输入顺序
背靠背下发并立即返回 group handle；它不等待机械到位。`wait_group()` 轮询所有参与轴，
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
的机械参数猜测默认运动 profile。Slide/Z 描述符当前复现已锁定 STM32 固件中的临时保守
machine soft limits，最终机械验收后应通过 `axis_descriptors` 注入更新值。

This is coordinated point-to-point submission, not interpolated or strictly synchronized motion.

当前没有轨迹插补、严格多轴同步、同时到达规划或完整采摘状态机；
`startup_position` 不属于普通运动目标，也不会参与坐标换算。Rotation 当前没有可靠的独立
stop，timeout 不会自动 torque disable。所有真实运动仍要求显式硬件配置、已确认的默认
速度/加速度、逐轴机械验证和现场安全措施；软件 stop 不是硬件急停。

## Upper motion runtime and safety modes

`create_upper_motion_runtime()` 是三类真实硬件的唯一公共组装入口。它加载调用方提供的
`HardwareConfig` 与 `MotionRuntimeConfig`，调用一次 `resolve_hardware()`，并用解析得到的
STM32/Feetech port 和 gs_usb device 创建 transport、bus、肩肘关节、Rotation、唯一的
`UnifiedMotionController` 及两个 façade。构造会进行设备发现，但不会打开通信、初始化关节、
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
Rotation 时同样执行该门禁，timeout 不会被描述为已停止，也不会以 torque disable 冒充 stop。

到位容差、稳定窗口、轮询周期、timeout、默认速度和默认加速度统一来自被 Git 忽略的
`config/motion_local.py`。先复制 `config/motion_local.example.py`，再替换其中明确标为
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

默认三设备只读联合检查：

```bash
cd host
.venv/bin/python scripts/test_upper_motion_runtime.py
```

`--execute` 只切换授权模式，脚本仍不发送运动；Rotation 风险接受还需同时给出
`--allow-rotation-motion`。真实硬件的分阶段验证顺序见
`docs/handoffs/UPPER_MOTION_RUNTIME_TEST_GUIDE.md`。

### Frontend and kinematics client boundaries

`motion.FrontendMotionInterface` 和 `motion.KinematicsMotionInterface` 是建议冻结的同进程
调用边界。`FrontendMotionFacade` 与 `KinematicsMotionFacade` 只转发方法，必须复用同一个
`UnifiedMotionController`：

```python
from bootstrap import create_upper_motion_runtime

runtime = create_upper_motion_runtime(hardware_config, motion_config)
frontend_motion = runtime.frontend_motion
kinematics_motion = runtime.kinematics_motion
```

`bootstrap.py` 负责创建唯一 controller 和通信生命周期；前端与运动学 façade 始终共享它。
应用入口创建一次 runtime 后注入各消费者，消费者不得自行重建硬件后端。

当前统一 API 映射如下：

| Client member | Unified controller member | Direct forwarding | Minimal core addition |
| --- | --- | ---: | ---: |
| `list_axes` / `describe_axis` / `get_state` | same name | Yes | No |
| `get_axis_states` | same name | Yes | Yes: ordered batch query |
| `submit_absolute` / `submit_positions` | same name | Yes | No |
| `get_command_result` | same name | Yes | No |
| `get_group_result` | same name | Yes | Yes: non-blocking aggregation |
| `wait_group` | same name | Yes | No |
| `stop` / `home_reference` | same name | Yes | No |

两个新增 core 方法只组合现有公开状态/result，不新增单位换算、命令记录、底层分发或硬件
访问。前端 façade 不暴露阻塞 `wait`/`wait_group`；运动学 façade 不暴露单轴提交、stop 或
home。

两个示例完全使用 fake，不读取本地硬件配置：

```bash
cd host
.venv/bin/python -m examples.frontend_motion_usage
.venv/bin/python -m examples.kinematics_motion_usage
```

成员交接文档位于：

- `docs/handoffs/FRONTEND_MOTION_INTERFACE_HANDOFF.md`
- `docs/handoffs/KINEMATICS_MOTION_INTERFACE_HANDOFF.md`

## Feetech 末端旋转轴

`drivers/feetech_protocol.py` 提供帧编码、状态包验证、timeout、显式 open/close 和
可注入串口；`robot/feetech_rotation.py` 提供有限行程角度换算、反馈读取、位置命令
和显式 torque enable/disable。位置命令按 Feetech 磁编码协议从 `0x2A` 连续写入
`position/time/speed` 六字节。

`config/feetech.py` 包含两层配置：`SM45BL_C001_PROFILE` 固化型号、飞特自定义串口协议、
RS-485 半双工、USB 转换板自动收发切换、115200 baud、12-bit/4096 counts 磁编码器和
C001 寄存器表；`END_EFFECTOR_ROTATION_CONFIG` 固化当前项目安装参数：ID 1、
`zero_raw=2130`、`direction_sign=+1`（逻辑正方向为 `+X`）、`-45°..+45°` 调试限位和
`max_speed_raw=500`。默认位置命令速度同为 500 raw。限位和速度是当前调试配置，仍需
随最终机构负载完成机械验收。设备 port 不固化，真实访问必须在运行时显式传入；写指令
status packet 行为也仍需实机复验。

人工工具默认完全离线。以下命令只生成 C001 ping 和读位置帧，不打开串口：

```bash
cd host
.venv/bin/python scripts/test_feetech_rotation.py --ping
.venv/bin/python scripts/test_feetech_rotation.py --read-raw-position
```

只读实机检查需要显式提供 `--execute` 和 port；baudrate 默认取 profile 的 115200：

```bash
.venv/bin/python scripts/test_feetech_rotation.py \
  --ping \
  --execute --port /dev/cu.YOUR_USB_RS485

.venv/bin/python scripts/test_feetech_rotation.py \
  --read-raw-position \
  --execute --port /dev/cu.YOUR_USB_RS485
```

CLI 支持直接使用 degree。以下命令自动采用项目配置，仍是完全离线 dry-run：

```bash
.venv/bin/python scripts/test_feetech_rotation.py \
  --position-deg 5
```

`--servo-id`、`--zero-raw`、`--direction-sign`、角度限位、`--speed-raw` 和
`--max-speed-raw` 均可显式覆盖，便于受控调试；越过当前 500 raw 上限会在打开串口和
enable torque 之前拒绝。放宽上限必须同时显式传入 `--max-speed-raw`。

只有确认预览、机械区域和急停条件后，才能在同一命令末尾增加：

```text
--enable-torque --execute --port /dev/cu.YOUR_USB_RS485
```

命令完成后 torque 仍保持开启；需使用 `--disable --execute --port ...` 显式关闭。
2026-08-03 已在 ID 1 实机完成 ping 和 `0x38` raw-position 只读测试，连续读数均为
`2047`；随后用户确认逻辑正方向正确，机械零点最终微调为 `zero_raw=2130`。当前
`±45°` 和 500 raw 已固化为调试约束，最终机械限位和负载安全速度仍待验收。
