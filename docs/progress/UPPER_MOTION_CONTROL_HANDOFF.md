# 上层运动控制整合交接

更新日期：2026-08-02

## 1. 当前结论

本轮已完成上层运动控制的代码盘点，并建立三个可离线验证的边界：

1. STM32 ASCII machine protocol v1 已有根项目正式 Host client；
2. shoulder/elbow 的 MG4010E CAN（Controller Area Network，控制器局域网）关节层与
   Planar 2R 已连接到最小双关节下发器；
3. `feetech_arm.zip` 经安全审查后，仅参考其中协议结构，重新实现了配置注入、无
   import 副作用、默认 dry-run 的 Feetech 末端旋转轴驱动。

当前仍处于“组件实现和离线集成”阶段。六个执行器后端尚无统一到位等待、超时停止、
故障传播或完整采摘状态机。本轮未连接或驱动任何真实硬件。

## 2. Git 与基线

### 根仓库

- path：项目根目录；
- branch：`main`；
- 本轮开始时 HEAD：`919213ba1ff681abc77345f39f31494d2e38513b`；
- commit：`chore: register STM32 firmware submodule`；
- 本轮开始前工作树已有修改：
  - modified：`host/README.md`、`host/motion/README.md`、
    `host/robot/__init__.py`、`host/robot/joint.py`、
    `host/scripts/test_joint_position.py`、`host/tests/suites/motion/test_joint.py`；
  - untracked：`feetech_arm.zip`、`host/robot/planar_arm.py`、
    `host/scripts/test_planar_2r_motion.py`、`host/tests/suites/kinematics/test_planar_arm.py`。
- 上述已有修改未 stash、未丢弃；本轮在 `host/README.md` 和
  `host/robot/__init__.py` 上做了必要的增量编辑。

以上文件名记录的是 2026-08-03 当时的历史工作树事实；相关人工脚本随后已由
`host/scripts/manual_motion.py` 和 `host/scripts/maintenance/` 三个 backend 入口替代并删除，
不再作为当前推荐命令。

### STM32 submodule

- path：`firmware/stm32_motion_controller`；
- branch：`main`；
- HEAD：`cb075675e32cd5c5e9e5d1d43ddaa5e539fdc8d4`；
- tag：`stm32-motion-v0.1.0`；
- remote：`git@github.com:Lemoooon-a/surf2026-stm32-motion-controller.git`；
- 本轮开始时工作树 clean；
- 本轮未修改子模块文件、HEAD 或根仓库 gitlink。

## 3. 当前 Host 代码链路

```text
Slide / Z / Vacuum
  STM32SerialTransport -> STM32MotionClient -> protocol v1

Shoulder / Elbow
  CanMotorBus -> mg4010_protocol -> MG4010Driver
              -> CanRotaryJoint -> Planar2RKinematics
              -> Planar2RArmController

End-effector rotation
  FeetechBus -> FeetechRotationAxis
```

### 3.1 MG4010E 与肩肘

| 层 | 文件 | 当前能力 |
| --- | --- | --- |
| CAN transport | `host/drivers/can_bus.py` | `gs_usb`、SocketCAN、共享 `RLock`、旧帧清理、timeout、有限重试、应答 ID 校验、显式同 ID 兼容 |
| protocol | `host/drivers/mg4010_protocol.py` | `0x94`、`0x92`、`0x9A`、`0x9C`、`0xA4`、`0x81` 编解码 |
| motor driver | `host/drivers/mg4010_driver.py` | 只读状态、位置命令、软件停止；不自动使能或清错 |
| joint | `host/robot/joint.py` | 减速比、逻辑零点、方向、有限行程解析、软限位、速度限制、运动前状态检查 |
| kinematics | `host/kinematics/planar_2r.py` | XY 平面正/逆运动学、双解、不可达与奇异处理 |
| arm bridge | `host/robot/planar_arm.py` | 按肩肘软限位筛解，背靠背下发；失败时尽力停止两关节 |

`0x94` 的单圈绝对位置用于跨重启机械位置解释；`0x92` 只用于本次上电周期的
`0xA4` 多圈目标构造。当前没有 enable、clear fault、arrival wait、stable window、
motion timeout 或严格同步轨迹。

### 3.2 肩肘配置

来源文件：`host/config/project/joints.py`。

| 关节 | CAN ID | ratio | logical zero | direction | limits | max velocity |
| --- | ---: | ---: | --- | ---: | --- | --- |
| shoulder | 1 | 36:1 | output absolute 100° | +1 | -65°..+65° | 50°/s |
| elbow | 2 | 36:1 | output absolute 158° | -1 | -160°..+160° | 50°/s |

这些值由代码注释声明来自上机测量，但 `docs/calibration/` 没有独立校准记录。因此其
软件来源可追踪，原始测量证据和机械复验记录不足。Planar 2R 的真实 `L1/L2` 也未写入
项目配置，CLI 中长度必须显式传入。

## 4. STM32 Host 接口

### 4.1 帧与状态语义

```text
@<seq> <cmd> [args...]
=<seq> OK
=<seq> ERR <error_code>
!<seq> DONE|ABORT|FAULT ...
```

- `seq`：`0..65535`，循环递增；
- frame 最大 96 ASCII 字节，不含换行；
- 轴：`Z` 或 `S`；
- 位置/距离：整数 `µm`；速度：整数 `µm/s`；加速度：整数 `µm/s²`；
- `MR`、`MA`、`HM`、`SU`、`SR` 的 `OK` 只表示接受，机械或流程完成看原 sequence
  对应的事件；
- 调试日志与协议共用串口，非 `=`/`!` 行由 Host 忽略。

### 4.2 命令表

| command | arguments | immediate | event | 可能动作/前置条件 |
| --- | --- | --- | --- | --- |
| `QS` | axis | `ST ...`/`ERR` | 无 | 只读 |
| `MR` | axis distance speed accel | `OK`/`ERR` | DONE/ABORT/FAULT | 会运动；要求 ready、homed、valid、限位合法 |
| `MA` | axis position speed accel | `OK`/`ERR` | DONE/ABORT/FAULT | 会运动；同上 |
| `HM` | axis | `OK`/`ERR` | DONE/ABORT/FAULT | 会运动；一次只允许一轴归零 |
| `ST` | axis | `OK`/`ERR` | 被中止命令发 ABORT | 立即停 STEP，保持 ENABLE |
| `DI` | axis | `OK`/`ERR` | 被中止命令发 ABORT | 停止、禁用、位置失效 |
| `EN` | axis | `OK`/`ERR` | 无 | 只使能，不产生 STEP |
| `SA` | 无 | `OK`/`ERR` | 被中止命令发 ABORT | 停止并禁用所有运动轴，位置失效 |
| `CF` | axis | `OK`/`ERR` | 无 | 当前只支持允许清除的 Slide 堵转锁存 |
| `QH` | 无 | `HS ...`/`ERR` | 无 | 只读归零状态 |
| `SQ` | 无 | `SS ...`/`ERR` | 无 | 只读吸盘状态 |
| `SU` | 无 | `OK`/`ERR` | DONE/ABORT/FAULT | 开始吸附；无真空传感器 |
| `SR` | 无 | `OK`/`ERR` | DONE/ABORT/FAULT | 释放后回安全空闲 |
| `SX` | 无 | `OK`/`ERR` | 被中止吸盘命令发 ABORT | 立即停止吸盘，不停止运动轴 |
| `VR` | 无 | `VR protocol firmware` | 无 | 只读版本 |

稳定错误码为 `0..15`，包括 frame、argument、busy、not ready/not homed、soft limit、
fault 和 actuator fault 等。详见固件 `App/Inc/app_protocol.h`。

`QS` 返回 `configured/enabled/busy/homed/valid/position_um/fault`。位置是脉冲估计的
machine position；成功归零后 `homed` 和 `valid` 才成立，`DI`、`SA` 及部分故障会使其
失效。当前固件上电不自动使能、不自动运动、不自动归零，吸盘默认关闭。

### 4.3 正式客户端

新增 `host/drivers/stm32_motion.py`：

- `STM32SerialConfig`；
- `STM32SerialTransport.open()/close()`；
- `STM32MotionClient.query_axis/query_home/query_suction/version`；
- `move_relative/move_absolute/home`；
- `stop/disable/enable/stop_all/clear_fault`；
- `suction_start/suction_release/suction_stop`。

采用根 Host 正式 client，而非直接复制子模块示例。协议常量仍不可自动共享，因此通过
`host/tests/suites/protocol/test_stm32_motion.py` 读取 submodule 当前 header/README，锁定 version、最大
行长和命令集合，发生漂移时离线测试失败。

当前 `motion_platform_config.c` 是 Z `-60800..0 step`、Slide `0..33333 step`。Z 已按
最高点归零、向下为负和 `-190..0 mm` 实测行程同步固件、Host 配置及协议示例；重新烧录后
仍需执行一次归零与软限位复验。Slide 最终机械全行程仍需实测。

## 5. `feetech_arm.zip` 审查

### 5.1 安全与结构

- SHA-256：`0794d6313bb38d400fc00ef51c3c6239259b94c9e7887af11e6b2b5bc77245ce`；
- 1019 entries，解压后标称 32,301,318 bytes；
- 999 entries 属于 Windows `.venv`，另有 `.idea`、`__pycache__`/`.pyc`；
- 未发现路径穿越、绝对路径、嵌套 `.git`、凭据、设备日志或串口抓包；
- 未提供 LICENSE、README、依赖清单或第三方来源说明；
- `unzip -t` 通过；原 zip 保持未修改、未跟踪；
- 仅将 5 个项目源码文件解压到 `/tmp` staging，未执行任何脚本，虚拟环境和二进制未
  纳入项目。

### 5.2 原代码分类

| 内容 | 分类 | 原因 |
| --- | --- | --- |
| header、length、instruction、checksum 基本结构 | 重构后复用 | 结构可与官方协议交叉验证 |
| feedback 地址与字段线索 | 重构后复用 | `0x38` position 已由 C001 实机只读确认，其他 feedback 字段仍待复核 |
| `main.py` | 不纳入 | 硬编码 `COM9`、115200、ID 1，运行后 enable、0°/180°运动和死循环读取 |
| `protocol.py` | 不直接复用 | sleep + `in_waiting` 读包、不校验 ID/error/checksum、异常吞掉、默认参数可变 |
| `servo.py` | 不直接复用 | 静默角度 clamp，stop 语义不可靠，位置命令只写 4 字节 |
| `.venv`/IDE/cache | 不纳入 | 生成物、本机环境和二进制，不可审计且无需版本控制 |

原压缩包没有具体型号和适配器类型。2026-08-03 用户确认项目实机为
`SM-45BL-C001`、RS-485、USB 转 485 自动收发切换板、115200 baud。该信息与原代码的
飞特自定义 `FF FF` 协议及官方 C001 资料一致；不适用于使用 Modbus-RTU 的 C002。

官方磁编码协议手册给出的示例从 `0x2A` 连续写 6 字节：position、time、speed；原
`servo.py` 的 4 字节布局遗漏 time 且寄存器偏移冲突。本轮采用官方布局。参考：

- [Feetech 磁编码舵机协议手册](https://www.feetechrc.com/Data/feetechrc/upload/file/20240702/%E8%88%B5%E6%9C%BA%E5%8D%8F%E8%AE%AE%E6%89%8B%E5%86%8C-%E7%A3%81%E7%BC%96%E7%A0%81%E7%89%88%E6%9C%AC.pdf)
- [FE-URT-1 官方页面](https://www.feetechrc.com/FE-URT1-C001.html)

## 6. Feetech 正式集成

### 文件与 API

- `host/drivers/feetech_protocol.py`
  - `build_instruction_packet()`、`parse_status_packet()`；
  - `FeetechSerialConfig`、`FeetechBus.open/close/ping/read_registers/write_registers`；
  - ID、length、checksum、device error、short write 和 timeout 验证；
- `host/robot/feetech_rotation.py`
  - `FeetechRotationConfig`：强制注入 ID、counts、zero、direction、limits、max speed；
  - `resolve_raw_position()`、`position_rad_to_raw()`；
  - `FeetechRotationAxis.read_position/read_feedback/command_position`；
  - `enable_torque/disable_torque`，不自动 enable；
- `host/config/project/feetech.py`
  - `SM45BL_C001_PROFILE` 固化型号、协议、RS-485、115200、4096 counts 和寄存器表；
  - `END_EFFECTOR_ROTATION_CONFIG` 固化 ID 1、`zero_raw=2130`、`direction_sign=+1`、
    `-150°..+150°` 当前限位和 500 raw 调试速度上限；逻辑正方向记录为 `+X`；
- `host/scripts/maintenance/feetech_rotation.py`
  - 通过 runtime 复用正式 Feetech 配置、VID/PID 设备发现、bus 和 Rotation axis；
  - 提供 ping/state/feedback/move/torque/register maintenance 子命令；
  - write 默认 preview，真实动作使用操作专属确认；
  - torque disable 明确提示自由转动或失去保持力风险；
  - 不提供虚构的 Rotation stop。

端口来自 `robot_hardware.py` 和集中设备发现，没有硬编码设备路径。串口在上下文退出或
异常时关闭。2026-08-03 已通过 `/dev/cu.usbmodem5B790798091`、115200 baud 对 ID 1 完成
实机 ping，并从 `0x38` 连续读取三次 `position_raw=2047`；完整 feedback 字段、写回包、
torque 行为仍未完整验证。用户随后完成受控小角度方向和零点确认，最终项目配置为
`direction_sign=+1`、`+X`、`zero_raw=2130`；`±45°` 和 500 raw 仍是待整机复验的调试值。

## 7. 执行器能力矩阵

“是”表示当前代码提供接口，不等价于实机验证。

| Capability | Slide | Z | Shoulder | Elbow | Rotation | Vacuum |
| --- | --- | --- | --- | --- | --- | --- |
| query state | 是 | 是 | 是 | 是 | 是 | 是 |
| command position | 是 | 是 | 是 | 是 | 是 | 不适用 |
| relative move | 是 | 是 | 否 | 否 | 否 | 不适用 |
| stop | 是 | 是 | `0x81` | `0x81` | 无独立 stop | `SX` |
| disable | `DI` | `DI` | `0x81` | `0x81` | torque disable | 不适用 |
| homing | 是 | 是 | 否 | 否 | 否 | 不适用 |
| position valid | 显式状态 | 显式状态 | 初始化后有效 | 初始化后有效 | 无独立 valid 状态 | 不适用 |
| wait for completion | event | event | 否 | 否 | 否 | event |
| motion timeout | client wait | client wait | 否 | 否 | 否 | client wait |
| fault | 状态/event | 状态/event | 状态读取 | 状态读取 | feedback error raw | 状态/event |

`host/motion/capabilities.py` 以不可变数据对象记录当前边界，并用
`test_upper_motion_smoke.py` 验证六个 fake 后端可在零硬件 I/O 下装配和查询。

## 8. 验证

工作目录：`host/`。

```bash
.venv/bin/python -m unittest discover -s tests -q
```

- 结果：全部通过；
- 覆盖：MG4010、joint、Planar 2R、Feetech frame/axis/CLI、STM32 client、能力 smoke；
- 硬件参与：无；
- Debug/Release STM32 build：本轮不修改固件，也未重跑；不能把既有编译结果当成本轮
  Host 集成的硬件验证。

还执行了：

```bash
.venv/bin/python -m py_compile \
  drivers/feetech_protocol.py drivers/stm32_motion.py \
  robot/feetech_rotation.py motion/capabilities.py \
  scripts/manual_motion.py scripts/maintenance/feetech_rotation.py
git diff --check
```

最终 test count 和 Git 状态以本轮最终回复为准。

## 9. 未确认参数与安全限制

不得猜测：

- Feetech 最终机械角度限位、负载安全速度和加速度安全上限；
- Feetech feedback/current/load/error 位定义与 write status return level；
- shoulder/elbow 独立校准原始记录；
- Planar 2R 实际 `L1/L2`；
- Slide/Z 最终机械全行程；
- camera-to-robot 坐标变换；
- 吸盘真空确认阈值（当前无传感器）。

`DONE V 1` 只证明 STM32 已打开泵，不证明吸住蘑菇。所有 Host CLI 的真实运动都要求
显式开关；本轮没有发送 CAN、串口运动、归零、吸附或使能命令。

## 10. 下一阶段最小里程碑

P0 — 固化可复现基线：区分并提交用户原有 shoulder/elbow/Planar 2R 修改与本轮 Host
边界修改，保留 zip 来源记录；验收条件为 clean checkout 可安装依赖并通过全量测试。

P1 — Feetech 只读台架：ping 和 raw position 已通过，且串口退出后无进程占用。下一步
补读完整 feedback，验证温度/error 与断线/错误 ID 行为，全程不 enable torque。

P2 — Feetech 参数复验：在安全空间复验当前 zero/direction 和调试 limits/max speed，
新增 `docs/calibration/` 原始记录；验收为参数可重复、软限位拒绝、torque disable 可控，
不声称急停能力。

P3 — 到位与停止语义：为 shoulder/elbow/rotation 增加 stable window、deadline、timeout
停止和 fault propagation；验收为 fake 故障矩阵和低速台架超时测试通过。

P4 — 低速点到点协调：实现 Slide/shoulder/elbow/Z/rotation/vacuum 的小型 coordinator，
不做连续轨迹和严格同步；验收为纯 fake 全流程和分阶段硬件联调，任一故障触发定义明确的
统一停止。

## 11. 新会话读取顺序

1. `AGENTS.md`；
2. `docs/progress/CURRENT_STATUS.md`；
3. 本文件；
4. `host/README.md`；
5. `host/drivers/stm32_motion.py`、`host/drivers/feetech_protocol.py`；
6. `host/robot/joint.py`、`host/robot/feetech_rotation.py`、
   `host/robot/planar_arm.py`；
7. `host/motion/capabilities.py` 和对应 tests。

首个命令：

```bash
git status --short
git submodule status
cd host
.venv/bin/python -m unittest discover -s tests -q
```

当前已确认 `direction_sign=+1`、逻辑正方向 `+X`、`zero_raw=2130`。主动任务应是“测量最终机械限位并确定
负载安全速度”，而不是直接实现完整采摘状态机或使用临时调试范围运行。
