# Upper Motion Manual and Maintenance CLI Guide

更新日期：2026-08-05

## 1. Entry-Point Overview

| 需要 | 长期入口 |
| --- | --- |
| 五轴统一状态和人工运动 | `scripts/manual_motion.py` |
| STM32/Slide/Z/Vacuum 维护 | `scripts/maintenance/stm32_motion.py` |
| Shoulder/Elbow/MG4010 维护 | `scripts/maintenance/mg4010_joint.py` |
| Rotation/Feetech 维护 | `scripts/maintenance/feetech_rotation.py` |

日常人工控制只使用 `manual_motion.py`。只有排查特定 backend protocol、原始状态、寄存器或
power 语义时才使用 maintenance 脚本。设备枚举和标定工具继续保持独立，不属于普通控制入口。

## 2. Unified Manual Control

`manual_motion.py` 的调用链为：

```text
当前机械臂正式 hardware/motion 配置
  -> create_upper_motion_runtime()
  -> UpperMotionRuntime
  -> UnifiedMotionController
```

子命令：

```bash
cd host

.venv/bin/python scripts/manual_motion.py inspect
.venv/bin/python scripts/manual_motion.py state --axis shoulder
.venv/bin/python scripts/manual_motion.py plan-base \
  --tcp-x-mm 510 --tcp-y-mm 0 --tcp-z-mm 180 --tcp-yaw-deg 0
.venv/bin/python scripts/manual_motion.py move \
  --axis shoulder --position 20 --velocity 2
.venv/bin/python scripts/manual_motion.py move-group \
  --shoulder 20 --elbow -40
.venv/bin/python scripts/manual_motion.py home --axis slide
.venv/bin/python scripts/manual_motion.py stop --axis elbow
```

`inspect`、`state` 和 `plan-base` 是只读操作。`move`、`move-group`、`home` 和 `stop` 默认只预览；真实动作
必须显式授权。`move-group` 只包含用户指定轴，稳定顺序为
`slide, z, shoulder, elbow, rotation`，不会自动补当前位置。这是背靠背点到点提交，不是轨迹
插补、严格同步，也不保证同时起步或到达。

正常轴动作只通过 controller。两个 backend 例外集中在共享 helper：Shoulder/Elbow 的进程内
只读位置初始化，以及 Rotation 明确确认后的当前位置预装与 torque enable。

### 2.1 Base TCP 五轴目标预览

`plan-base` 读取当前五轴逻辑状态与已验证的机械几何，计算当前/目标局部坐标和工作区状态，再调用
只读 `validate_positions()` 检查每一个完整五轴阶段。当前 Slide 有合法解时必须保持；否则依次
尝试唯一工作区中心和 10 mm 有限 fallback。

`INSIDE→INSIDE` 打印一个 `DIRECT`；`OUTSIDE→INSIDE` 打印 `LIFT`、`TRANSIT`、`LOWER`。当前 Robot motion
envelope 的 workspace-entry 策略是 Base `Z=200 mm` 绝对最低高度，实际过渡高度为当前、目标与
200 mm 三者中的最高值，并由正式运动学换算
为 Z 轴目标。当前 TCP 已高于 200 mm 时不会继续上抬，`LIFT` 为零位移检查并直接进入横移。
输出包含每阶段 Base 位姿、五轴目标、局部
工作区和 FK 残差。它没有 `--slide-mm`、`--allow-unvalidated-frame-transform` 或 `--execute`；标定
缺失或未验证会拒绝规划，不会 submit、wait、home、stop、torque enable 或写回标定。

唯一 arm-local 矩形 `[100,600]×[150,350] mm` 是运动学约束，不只是可视化提示；最终普通五轴解必须位于其中。真实 Base
目标执行仍是后续独立任务。

## 3. STM32 Maintenance

```bash
.venv/bin/python scripts/maintenance/stm32_motion.py --help
```

子命令：`version`、`state`、`move`、`home`、`stop`、`enable`、`disable`、`clear-fault`、
`suction-state`、`suction-start`、`suction-release`、`suction-stop`。

`version`、`state`、`suction-state` 为只读。move 参数使用 `mm`、`mm/s`、`mm/s²`，CLI 会打印
工程单位、转换后的整数 `um` protocol 值、轴名和 machine command。其他写命令默认预览，分别
使用独立确认参数。该入口复用 runtime 中的 `STM32MotionClient`，不硬编码串口。

## 4. MG4010 Maintenance

```bash
.venv/bin/python scripts/maintenance/mg4010_joint.py --help
```

子命令：`raw-status`、`basic-parameters`、`logical-angle`、`initialize`、`state`、`move`、
`software-stop`；支持 `shoulder` 和 `elbow`。move 参数是逻辑输出轴 `position-deg` 和
`velocity-deg-s`，并通过正式 `CanRotaryJoint` 完成换算与命令。

`logical-angle` 不执行限位内位置初始化；它只读 `0x94`，按正式减速比、零点和方向输出相对逻辑
零点的有符号最短角差。即使 `within_limits=false` 也会显示诊断角度；加 `--watch` 可在同一打开的
runtime 中持续读取，`--interval` 设置刷新周期。

`initialize` 只读取稳定绝对位置并建立当前进程的逻辑位置解释；它不是 enable、home 或 motion。
`software-stop` 发送 MG4010 `0x81`，不表示驱动下力、断电或硬件急停。肩肘联合动作统一使用
`manual_motion.py move-group`，maintenance 不再维护第二套双关节实机入口。

## 5. Feetech Maintenance

```bash
.venv/bin/python scripts/maintenance/feetech_rotation.py --help
```

子命令：`ping`、`state`、`feedback`、`move`、`torque-enable`、`torque-disable`、
`read-register`、`write-register`。它复用 runtime 的正式 Feetech config、discovery、bus 和
`FeetechRotationAxis`，不接受或硬编码本机串口路径。

move 不自动 torque enable；确实需要时必须另加 `--enable-torque`。`torque-enable` 只改变 torque
状态，不自动发送运动目标。`torque-disable` 可能使机构自由转动或失去保持力，必须确认
`--confirm-free-motion-risk`。Rotation 没有可靠独立 stop，因此该脚本不存在 stop 子命令。

## 6. Safety Confirmations

| 操作 | 必须同时提供 |
| --- | --- |
| Unified move/move-group | `--execute --confirm-motion` |
| Unified home | `--execute --confirm-home-motion` |
| Unified stop | `--execute --confirm-stop` |
| Unified Rotation motion | 另加 `--allow-rotation-motion --confirm-rotation-no-stop --enable-rotation-torque` |
| STM32 enable/disable/clear fault | 各自的 `--confirm-enable`、`--confirm-disable`、`--confirm-clear-fault` |
| STM32 suction write | `--confirm-suction-action` |
| MG4010 software stop | `--confirm-software-stop` |
| Feetech torque enable | `--confirm-torque-enable` |
| Feetech torque disable | `--confirm-free-motion-risk` |
| Feetech register write | `--confirm-register-write` |

进入 `RuntimeMode.MOTION` 本身不发送动作。所有 preview 均不发送控制命令。

## 7. Stop vs Disable vs Torque Disable

- Unified/STM32 `stop`：终止当前运动，不保证下力。
- STM32 `disable`：STM32 backend 维护命令，可能使开环位置参考失效。
- MG4010 `software-stop`：`0x81`，只能称 software stop。
- Feetech `torque-disable`：失去保持力的维护动作，不能称为 stop。
- `runtime.close()`：只关闭通信，不等于上述任何动作。

`manual_motion.py` 不提供统一 disable。

## 8. Rotation Restrictions

Rotation 没有经过验证的独立 stop。统一运动需要普通运动双确认、无 stop 风险确认和独立 torque
preparation 确认；任务结束后不会自动 torque disable。多轴失败结果也不能证明 Rotation 已停止。
torque disable 只存在于 Feetech maintenance。

## 9. Calibration Preparation

标定脚本不会自动移动机构。准备流程统一使用：

```text
manual_motion.py inspect
manual_motion.py state
manual_motion.py home
manual_motion.py move
manual_motion.py move-group
```

移动到已确认姿态并停止所有轴后才运行标定采集。若标定后需要 Rotation 下力，使用
`maintenance/feetech_rotation.py torque-disable` 并确认自由转动/失去保持力风险；不得把
`manual_motion.py stop` 当作 torque disable。

## 10. Removed Legacy Commands

旧 diagnostics/debug wrappers、`test_upper_motion_*`、MG4010 人工 `test_*.py`、Planar 2R 实机
入口和旧 Feetech 人工入口已被上述四个 CLI 覆盖并删除。Planar 2R 数学库、robot bridge 与自动
测试继续保留。历史 review/progress 文件可保留旧文件名作为当时事实，但不再是当前推荐命令。

## 11. Current Limitations

当前不提供 Base 目标真实执行、轨迹插补、连续速度、严格同步、同时起步/到达、碰撞检测、自动
enable、自动 home、startup position、完整初始化状态机、自动标定运动、Rotation 独立 stop 或
统一模糊的 disable-all。
离线测试和编译结果不构成真实硬件验收。

## 12. Five-Axis Motion Demo: Suction and Holding Torque

真实五轴闭环验证使用：

```bash
cd host
.venv/bin/python scripts/run_motion_demo.py --execute
```

执行模式启动顺序为：读取状态、`suction idle`、使能 Shoulder/Elbow/Rotation、验证使能、Z
Homing、Slide Homing、进入 startup safe pose。任一关节使能失败都会阻断后续 Homing 和运动。
不加 `--execute` 时只做状态读取与规划预览，不发送吸盘、扭矩、Homing、运动或 stop 写命令。

交互命令示例：

```text
status
joints status
joints enable

workspace
move <tray_x_mm> <tray_y_mm> <tray_z_mm> <yaw_deg>

suction status
suction grip
suction release
suction idle

stop
joints status

joints disable
joints status
quit
```

`tray_x_mm/tray_y_mm/tray_z_mm` 必须从 `workspace` 显示、且经用户确认的 Base-frame 培养槽范围
内选择。本机当前确认范围为 X `[100, 600] mm`、Y `[150, 800] mm`、Z `[10, 200] mm`；缺失或
未确认配置会在打开硬件前拒绝 CLI 启动。

`workspace` 还会显示 arm-local 单一工作区、startup pose、workspace-entry clearance 和轴/关节
限位，并明确 Tray 不是机械范围、arm-local workspace 不是 Base 坐标、Robot motion envelope 不是碰撞模型。

这里的 Z 是最终 TCP 在 Base frame 中的绝对高度。配置是静态快照，工具长度或任务边界变化后必须
重新计算并确认。

吸盘实际映射：`grip`=`SU`（泵 ON、释放阀 CLOSED），`release`=`SR`（泵 OFF、阀 OPEN，固件默认
500 ms 后自动 IDLE），`idle`=`SX`（泵 OFF、阀 CLOSED）。`status` 显示的是 commanded output
state；当前协议没有真空传感器结果，因此 `Physical vacuum verified` 始终为 `no`，不能据此判断
物体已经吸牢。

`stop` 只停止当前运动，不调用 MG4010 `0x80` 或 Feetech Torque Disable；startup、DIRECT、
LIFT、TRANSIT、LOWER、init、吸盘动作以及 `quit` 都不会自动移除保持力。显式失能后，任何普通
Base 目标或旋转关节运动会提示先执行 `joints enable`。重新使能会重读 Shoulder、Elbow、
Rotation 实际位置并打印当前 FK/TCP。

> WARNING: Support the mechanism before disabling rotary-joint torque.

真实硬件建议按以下四阶段低速验证，不要同时施加大外力或测试吸盘与运动：

1. `joints status` → `joints enable` → 轻触确认保持 → `stop` → 再次确认仍使能；
2. 支撑机构后 `joints disable`，确认 move 被拒绝，再 `joints enable` 并核对 FK；
3. 机械臂静止时依次执行 `suction status/grip/status/release/status/idle`，人工确认泵阀方向；
4. `joints enable` → `init` → 保守目标 → `suction grip/release` → `init`。

`quit` 会提示：旋转关节保持使能，除非此前显式执行过 `joints disable`。关闭 Host 通信不等于
对硬件供电状态做保证，现场仍需确认电源、驱动器与机构支撑。
