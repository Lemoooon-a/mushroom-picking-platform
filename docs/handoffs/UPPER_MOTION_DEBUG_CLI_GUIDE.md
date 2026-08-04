# Upper Motion Manual and Maintenance CLI Guide

更新日期：2026-08-04

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
hardware/motion local config
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
  --tcp-x-mm 510 --tcp-y-mm 0 --tcp-z-mm 65 --tcp-yaw-deg 0 \
  --allow-unvalidated-frame-transform
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

`plan-base` 读取当前五轴逻辑状态，把 Base 根的 TCP `x/y/z/yaw` 目标转换到 Slide-zero，生成并
筛选五轴逆运动学候选，调用统一 controller 的只读 `validate_positions()`，最后打印选中解、分支、
评分、FK 残差、软限位余量和完整 `MultiAxisTarget`。不提供 `--execute`，不会 submit、wait、home、
stop 或 torque enable。

不提供 `--slide-mm` 时采用“当前 Slide 优先、否则最近离散候选”策略；提供时只在指定 Slide 上
求解。当前 `base_T_slide_zero` 尚为 provisional，因此必须显式提供：

```text
--allow-unvalidated-frame-transform
```

该参数只允许本次只读预览，不写回标定文件，也不表示标定已经验证。若目标无解，CLI 会保留已
打印的 Slide-zero 目标，并报告失败阶段和候选统计，不生成伪造目标。真实 Base 目标执行将在完成
独立姿态验证后作为单独任务增加。

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

子命令：`raw-status`、`basic-parameters`、`initialize`、`state`、`move`、`software-stop`；支持
`shoulder` 和 `elbow`。move 参数是逻辑输出轴 `position-deg` 和 `velocity-deg-s`，并通过正式
`CanRotaryJoint` 完成换算与命令。

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
