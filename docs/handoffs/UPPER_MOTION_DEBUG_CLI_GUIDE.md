# Upper Motion Debug CLI Guide

更新日期：2026-08-04

## 1. Scope

本指南覆盖 Host 上层运动的四个长期人工调试入口：

| 用途 | 入口 |
| --- | --- |
| 五轴只读检查 | `host/scripts/diagnostics/inspect_upper_motion.py` |
| 单轴状态、绝对运动、软件 stop | `host/scripts/debug_motion/debug_axis_motion.py` |
| 任意轴子集点到点运动 | `host/scripts/debug_motion/debug_multi_axis_motion.py` |
| Slide/Z 机械归零 | `host/scripts/debug_motion/home_linear_axis.py` |

这些入口复用 `create_upper_motion_runtime()` 创建的唯一 runtime 和
`UnifiedMotionController`，不重新组装设备、transport、bus、joint 或 axis。它们不是生产
状态机、轨迹规划器、标定运动器或 disable/下力维护工具。

## 2. Safety Model

`inspect`、`state` 和所有未带 `--execute` 的 move、multi-axis、home、stop 命令均为只读或
预览。预览仍会打开配置中解析到的真实通信资源，但不发送运动、home、stop、enable、torque
write、clear fault、suction 或配置写入。

真实 move 必须同时提供 `--execute --confirm-motion`；真实 home 必须同时提供
`--execute --confirm-home-motion`；软件 stop 必须同时提供 `--execute --confirm-stop`。选择
`MOTION` runtime 本身不会自动运动、enable、home 或移动到 startup position。任何真实测试仍需
人工确认机械空间、支撑、方向、限位和物理急停条件。

## 3. Read-Only Inspection

```bash
cd host
.venv/bin/python scripts/diagnostics/inspect_upper_motion.py
```

该命令读取 STM32 version、五轴 descriptor/capability 和统一逻辑状态。Shoulder/Elbow 会在
一个共享 helper 中执行进程内只读 position initialization；它不是 enable、home 或 motion。
新入口没有 `--execute` 参数。

## 4. Single-Axis State

```bash
cd host
.venv/bin/python scripts/debug_motion/debug_axis_motion.py state --axis shoulder
```

`state` 支持 `slide`、`z`、`shoulder`、`elbow`、`rotation`，使用 `READ_ONLY` runtime，输出
descriptor、单位、软限位、capability 和当前状态，不发送写命令。

## 5. Single-Axis Absolute Motion

默认只预览和校验：

```bash
.venv/bin/python scripts/debug_motion/debug_axis_motion.py move \
  --axis shoulder --position 20 --velocity 2
```

现场确认后才可显式执行：

```bash
.venv/bin/python scripts/debug_motion/debug_axis_motion.py move \
  --axis shoulder --position 20 --velocity 2 \
  --execute --confirm-motion
```

公开位置单位为 Slide/Z 的 `mm` 和旋转轴的 `deg`；速度分别为 `mm/s`、`deg/s`。加速度只在
descriptor 声明支持时使用。命令通过 `validate_positions()`、`submit_absolute()` 和 `wait()`
进入统一控制器，不自动 enable 或 home。

## 6. Single-Axis Stop

预览或执行单轴软件/protocol stop：

```bash
.venv/bin/python scripts/debug_motion/debug_axis_motion.py stop --axis shoulder
.venv/bin/python scripts/debug_motion/debug_axis_motion.py stop \
  --axis shoulder --execute --confirm-stop
```

Slide/Z 使用 STM32 software/protocol stop；Shoulder/Elbow 使用 MG4010 software stop
`0x81`。Rotation 没有经过验证的独立 stop，因此明确拒绝。该命令不会调用 disable。

## 7. Multi-Axis Point-to-Point Motion

至少提供一个目标；只有显式列出的轴参与。参数解析后采用稳定顺序
`slide, z, shoulder, elbow, rotation`，不会为未指定轴补当前位置或保持命令。

```bash
# READ_ONLY preview
.venv/bin/python scripts/debug_motion/debug_multi_axis_motion.py \
  --shoulder 20 --elbow -40

# 现场确认后执行同一轴子集
.venv/bin/python scripts/debug_motion/debug_multi_axis_motion.py \
  --shoulder 20 --elbow -40 \
  --execute --confirm-motion
```

实现调用一次 `submit_positions()` 和一次 `wait_group()`，并打印 group 与逐轴终态。这是背靠背
点到点提交，不是轨迹插补，不保证同时起步、同时到达或严格同步。控制器拥有 timeout、fault、
partial submission 和 peer best-effort stop；CLI 收到 terminal failure 后不会重复 stop。

## 8. Slide/Z Reference Homing

```bash
# READ_ONLY preflight
.venv/bin/python scripts/debug_motion/home_linear_axis.py --axis slide

# 真实机械归零
.venv/bin/python scripts/debug_motion/home_linear_axis.py \
  --axis slide --execute --confirm-home-motion
```

只接受 Slide/Z，并通过 `home_reference()` 执行真实机械归零。成功要求结果为 `ARRIVED`，且最终
`homed=True`、`position_valid=True`、`busy=False`、`faulted=False`。控制器返回 terminal
timeout/fault/abort 后 CLI 不重复 stop；提交可能已发生但尚无 terminal result 的异常或
`KeyboardInterrupt` 才由 CLI 对该轴最多尝试一次软件 stop。stop 失败只记录，不覆盖原始异常。
STM32 stop accepted 不表示驱动已 disable，轴仍可能保持 ENABLE。

## 9. Rotation Restrictions

Rotation 状态可只读查询，但没有可靠独立 stop。真实 Rotation move 除普通双确认外，还要求：

```text
--allow-rotation-motion
--confirm-rotation-no-stop
--enable-rotation-torque
```

当前硬件流程必须先写入反馈得到的当前位置，再显式 torque enable，最后提交目标；该 power
preparation 集中在共享 helper，任何确认缺失都会在写入前失败。任务完成后不会自动 torque
disable。Rotation torque disable 不属于本轮公共 CLI。

## 10. Stop vs Disable vs Torque Disable

- `stop`：请求当前运动停止；Slide/Z 和 Shoulder/Elbow 保持各自现有 protocol 语义。
- `disable`：撤销 STM32/MG4010 驱动使能，不由这些公共 CLI 提供。
- `torque disable`：Feetech Rotation 的下力维护动作，不由这些公共 CLI 提供。

三者不可互换；software stop 也不等于硬件急停、断电或已下力。

## 11. Legacy Command Compatibility

旧入口仍在原路径并直接 import 新实现，不使用 subprocess：

- `scripts/test_upper_motion_runtime.py` → 新只读诊断；旧授权参数只显示弃用并被忽略；
- `scripts/test_upper_motion_home.py` → 新 Slide/Z home；
- `scripts/test_upper_motion_five_axis.py` → 新多轴实现，继续解析旧五轴参数和五项确认。

运行旧入口会向 stderr 输出 `DEPRECATED`，但不会阻止原命令继续执行。新开发和文档示例应只
引用长期入口。

## 12. Calibration Preparation

标定脚本仍不会自动移动机构。操作者应先用 `home_linear_axis.py` 完成必要的 Slide/Z 归零，
再用单轴或轴子集 CLI 把机构人工移动到经确认的参考姿态，停止所有轴后运行只读/预览标定。
移动授权和标定文件写入授权是两个独立步骤，任何标定命令都不会代替运动确认。

## 13. Current Limitations

当前不提供：轨迹插补、连续速度、严格同步、同时起步/到达、碰撞检测、自动 enable、自动
home、startup position、统一 disable/下力、Rotation 独立 stop、自动 torque disable、完整系统
初始化状态机或标定自动运动。底层 backend maintenance 脚本仍用于协议和驱动级诊断，不能当作
长期统一运动入口。
