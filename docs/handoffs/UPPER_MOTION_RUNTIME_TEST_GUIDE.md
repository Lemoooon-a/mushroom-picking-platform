# 上层运动 Runtime 分阶段测试指南

更新日期：2026-08-03

## 1. 当前结论与安全边界

当前代码已经具备统一设备解析、对象组装、通信生命周期、只读/运动授权门禁和离线测试。
这不代表任何真实单轴、多轴或五轴运动已通过机械验证。Rotation 当前没有经过验证的独立
stop；软件 stop 也不是硬件急停。

每次真实测试前必须确认：机械区域清空、急停可达、限位与方向已复核、机构有可靠支撑、
供电和通信可安全切断，并且同一时刻只有一个进程拥有三类硬件。不得跳过前一阶段的验收
直接进入后一阶段。

## 2. 本地配置

本机配置均被 Git 忽略，不提交真实端口或未经验收的运动参数：

```bash
cd host
cp config/hardware_local.example.py config/hardware_local.py
cp config/motion_local.example.py config/motion_local.py
```

设备身份只使用已确认的 VID/PID；串口路径由启动时枚举得到。`motion_local.example.py` 中的
值是 `EXAMPLE / BENCH-TEST PLACEHOLDER`，不是生产标定。Rotation 工程速度和加速度映射
未验证时必须保持 `None`。

## 3. 分阶段顺序

### 1. 全量离线测试

```bash
cd host
.venv/bin/python -m unittest discover -s tests -q
```

要求全部通过，且 STM32 子模块状态无修改。此阶段不连接或访问硬件。

### 2. 三设备只读联合测试

```bash
cd host
.venv/bin/python scripts/list_hardware_devices.py --list-all
.venv/bin/python scripts/list_hardware_devices.py --resolve
.venv/bin/python scripts/test_upper_motion_runtime.py
```

最后一个命令默认创建 `READ_ONLY` runtime，只打开通信并读取 STM32 版本/状态、肩肘绝对
位置和 Feetech 反馈；不会 home、enable、torque enable 或发送位置命令。确认三个设备身份、
肩肘只读初始化、五轴状态摘要和逆序关闭均正常。

### 3. Slide 单轴归零和小距离运动

仅在现场完成方向、原点开关、软限位和急停复核后，由专用受控入口显式选择 `MOTION`。
先归零，再用保守速度执行一个小距离目标，核对最终位置、到位容差和 timeout 行为。不要与
其他轴联动。

```bash
# 默认只读预检
.venv/bin/python scripts/test_upper_motion_home.py --axis slide

# 确认现场安全后才执行一次真实机械归零
.venv/bin/python scripts/test_upper_motion_home.py \
  --axis slide \
  --execute \
  --confirm-home-motion
```

### 4. Z 单轴归零和小距离运动

重复 Slide 的检查，并额外确认负载不会因失能、断电或错误方向下落。先完成机械支撑和
重力风险控制，再进行任何 Z 运动。

```bash
# 默认只读预检
.venv/bin/python scripts/test_upper_motion_home.py --axis z

# Slide 已独立验收、Z 负载已支撑后才执行
.venv/bin/python scripts/test_upper_motion_home.py \
  --axis z \
  --execute \
  --confirm-home-motion
```

该程序一次只接受一个轴；只有统一结果为 `ARRIVED` 且最终状态同时满足 `homed=True`、
`position_valid=True` 才判定成功。执行前必须确认 `busy=False`，且不存在除
`fault_code=2`（未归零导致的位置无效）之外的故障。异常或中断只会尝试软件 stop，不能
替代硬件急停。

### 5. Shoulder 低速小角度

先在只读模式取得至少三次稳定的 `0x94` 绝对位置样本，确认逻辑零点、方向、当前角度和
软限位，再显式进入 `MOTION`，以低速执行小角度目标。记录命令前后状态和软件 stop 结果。

### 6. Elbow 低速小角度

按 Shoulder 同样流程独立验证，重点检查 `direction_sign=-1` 的逻辑方向和可能的机械干涉。

### 7. Slide + Z

只在两个直线轴分别通过单轴验收后进行点到点联合提交。当前接口是背靠背提交，不是轨迹
插补或严格同步；分别检查每个轴的完成状态和超时结果。

### 8. Shoulder + Elbow

只在两关节分别通过单轴验收后进行低速、小角度联合点到点。先验证运动学目标在两轴软限位
内，并观察连杆扫掠范围。不得把“命令已接受”当作“机械已到位”。

### 9. 四轴联合点到点

联合 Slide、Z、Shoulder 和 Elbow，但仍不包含 Rotation。使用保守目标逐步扩大范围，并记录
逐轴到位、fault、timeout 和 stop 结果。当前不保证同时起步或同时到达。

### 10. Rotation 单轴低速验证

Rotation 必须最后单独验证。只有在现场明确接受“没有可靠独立 stop、timeout 后不能保证已
停止”的风险后，才可同时选择：

```text
RuntimeMode.MOTION
allow_unverified_rotation_motion=True
```

`scripts/test_upper_motion_runtime.py --execute --allow-rotation-motion` 只展示授权模式，仍不会发送
运动。真实 Rotation 目标必须来自另一个明确、受控的调用入口。不得用 torque disable、关闭
串口或 timeout 冒充 stop。

### 11. 确认 Rotation 故障降级策略

在允许任何自动联动前，书面确认通信中断、超时、失控和急停场景下的人工/电气处置方法，
并决定 Rotation 是否继续参与自动多轴任务。在可靠独立 stop 尚未验证前，默认门禁保持关闭。

### 12. 最后才允许五轴联合

只有前 11 阶段均有记录并通过现场验收，才可考虑五轴联合点到点。即使进入该阶段，现有
实现仍不是轨迹插补、严格同步、碰撞检测或完整采摘状态机，必须继续使用外部机械安全措施。

## 4. Runtime 语义速查

- `create_upper_motion_runtime()`：设备解析和对象构造，不打开通信；
- `runtime.open()`：STM32 → CAN → Feetech，只打开通信；
- `runtime.close()`：Feetech → CAN → STM32，不自动 stop/disable/torque disable；
- `READ_ONLY`：允许查询，拒绝运动和 home；
- `MOTION`：仅允许后续显式普通轴动作通过门禁，不自动发送动作；
- Rotation：还需要额外授权，且 `stop` capability 仍为 false。

## 5. 记录要求

每阶段至少记录 commit、硬件 VID/PID、配置版本、目标与速度、现场安全条件、实际方向、最终
位置、fault/timeout、stop 语义和操作者结论。编译成功或离线测试成功不得记录为硬件验证。
