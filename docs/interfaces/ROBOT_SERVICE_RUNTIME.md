# Robot Service Runtime

状态：implemented、offline-tested；execute 接口保留但本轮未运行真实硬件。

`MushroomRobotService` 是进程级唯一应用入口，通过私有 axis-motion port 使用内部统一控制器，并持有 `MushroomRobotController`、Vision Gateway、`VisionPickWorkflow`、应用状态和可选 JSON Lines recorder。它不公开 runtime/controller，也不重新实现正运动学（Forward Kinematics, FK）、逆运动学（Inverse Kinematics, IK）或执行器协议。

## API

```python
startup()
shutdown()
status() -> RobotServiceStatus
list_axes() -> tuple[AxisDescriptor, ...]
get_axis_state(axis) -> AxisState
get_axis_states(axes=None) -> tuple[AxisState, ...]
move_axis_absolute(axis, position, velocity=None, acceleration=None, timeout_s=None)
move_axis_relative(axis, delta, velocity=None, acceleration=None, timeout_s=None)
plan_base_target(target: BaseToolTarget)
move_base_target(target: BaseToolTarget) -> MotionResult
request_observation() -> VisionTargetObservation
plan_observation(observation, grasp_profile=None) -> PickPlan
execute_pick_plan(plan) -> PickResult
pick(grasp_profile=None) -> PickResult
return_to_startup()
stop()
enable_joints() / disable_joints()
suction("grip" | "release" | "idle")
```

状态为 `CREATED → STARTING → READY`，观察使用 `OBSERVING`，规划使用 `PLANNING`，真实动作使用 `EXECUTING`，关节失能后为 `DISABLED`，不可恢复执行错误为 `FAULT`，关闭后为 `SHUTDOWN`。Service 用一个私有 active-operation token 串行化 startup、运动、pick、holding 和 suction；状态检查与 token 注册在短临界区内原子完成，阻塞 backend 调用和到位等待不持有状态锁。单轴提交前的参数、busy、位置有效性、Homing、holding 和软限位拒绝保持 READY；获得 command handle 后的 timeout、BUSY、通信或设备故障会 best-effort stop 后进入 FAULT。

`status()` 始终立即返回进程级 `state`、`mode` 和 `fault`；只有 `READY` 或 `DISABLED`
会进一步读取实时 backend 状态。活动状态、`FAULT`、`CREATED` 和 `SHUTDOWN` 的
`backend_status` 为 `None`，避免状态轮询与运动/停止线程竞争硬件 I/O，也避免故障后反复访问
已经失效的 transport。

`stop()` 和 `shutdown()` 会使当前 token 失效，因此迟到的运动线程不能覆盖较新的 `FAULT` 或 `SHUTDOWN`。没有活动操作时，`stop()` 保持 `CREATED/READY/DISABLED/FAULT/SHUTDOWN` 原状态；它不再充当 lifecycle 或 holding 恢复入口。`DISABLED` 可直接调用 `enable_joints()`，只有重新确认 Shoulder、Elbow、Rotation holding 以及五轴连接、静止、无 fault、位置有效后才进入 `READY`。

Rotation 的软件停止会读取当前反馈位置并立即写回 goal，在保留转矩的前提下尝试制动；显式统一 stop 最多等待 2 秒确认 `moving=False`。该方式尚未完成低速真机验收，不是厂商独立 stop、失能或硬件急停；顶层 Service 不再要求单独的 Rotation 启动确认。

## 两种运动入口

- Base-frame task movement：使用 `move_base_target()`，经过 TrayWorkspace、IK、OffsetWorkspace、transition planning 和 motion envelope。
- Raw/manual axis movement：使用 `move_axis_absolute()` / `move_axis_relative()`，只检查所选轴自身状态、holding/Homing 和软限位，不进行 Base-frame 工作区、IK、side-switch、碰撞或 TCP 路径检查。

相对运动在 `UnifiedMotionController` 的同一提交锁内读取调用时当前有效逻辑位置，计算 `current + delta`，校验并走现有绝对提交/到位通路。增量位于轴到位容差内时立即返回 ARRIVED，不发送硬件命令；result 中 `target_position` 仍是解析后的绝对逻辑位置。

STM32 machine protocol 客户端用一把可重入协议锁串行化 sequence/pending 状态、写入和每次
完整 `read_line()`。同步响应和异步终态仍按 sequence 缓冲和路由；锁不会跨越整个运动到位
等待，因此 STOP 只需等待当前一次串口读取，不会等待原运动超时。通信错误继续清空 pending、
关闭 transport 并 fail-closed，不自动 reconnect。

`get_current_tcp_pose()` 只在 `READY` 读取五轴有效位置，并交给 Controller 已配置的
Base-frame 正运动学提供者计算 `x/y/z/yaw`。Web 前端不保存或复制机械尺寸、坐标变换或
运动学公式。`return_to_startup()` 继续复用原有 startup-safe pose 与执行通路。

`scan_and_pick()` 在一个顶层 active-operation 内按配置生成 2×4 共 8 个 Base TCP 扫描位。
每个扫描位到位后重复 `observe → pick → lift to Base Z=150 mm → fixed place → release → return same scan pose`，只有收到
`no_target` 才进入下一区域。目标规划拒绝只结束当前区域；运动或吸盘失败停止并进入
`FAULT`；达到 `max_picks_per_scan_pose` 会在已经返回扫描位后停止整个任务并报告，但不进入
`FAULT`。dry-run 仅允许离线后端推进虚拟位姿，不提交硬件命令。

普通目标放置点为 Base `(150, 1000, 150, 0)`，过大目标放置点为
Base `(450, 1000, 150, 0)`。视觉观察中的 `size_class` 决定使用哪个点；两个放置点是
scan-pick 仅有的 Tray workspace 区外例外。放置点和返回扫描位会在移动前作为两段序列
完整规划，返回扫描位仍执行正常 Tray 门禁。
放置点到位后立即释放并直接返回，不包含放置前接近或放置后回撤阶段。区外例外不会绕过
OffsetWorkspace、逆运动学、轴/关节限位或 RobotMotionEnvelope。

## 模式

- `read-only`：加载并检查配置、status/capabilities/workspace；不构造硬件 runtime，不打开硬件。
- `dry-run`：使用纯配置 `OfflinePlanningBackend`、FakeVisionGateway、真实 FK/IK/工作区/transition planner；后端没有硬件 submit API。
- `execute`：使用现有 `DemoMotionFlow`、`UnifiedMotionController` 和 `MotionAuthorization`，CLI 要求 `--confirm-motion`。本轮未执行。

入口：

```bash
cd host
.venv/bin/python scripts/robot_service.py --mode read-only
.venv/bin/python scripts/robot_service.py --mode dry-run --fake-position X Y Z
.venv/bin/python scripts/robot_service.py --mode execute \
  --confirm-motion
```

支持原有命令，并新增 `axes`、`axis state`、`axis states`、`axis move-abs` 和 `axis move-rel`。单轴移动只允许 execute + READY；状态查询允许 execute 的 READY/EXECUTING/DISABLED/FAULT，也允许已有明确模拟状态的 dry-run。

Service 固定加载 `host/config/robot_runtime.json`，且六个业务区块必须同时有效。JSONL 记录由
其中的 `recording.enabled` 和相对 Host 根目录的 `recording.jsonl_path` 控制；路径必须
位于 `host/runtime/` 运行缓存目录内，当前写入 `host/runtime/scan-pick-real.jsonl`。
该目录已被 Git 忽略，并在首次记录时自动创建。记录包含版本、状态、输入、计划、阶段结果和错误，不记录
`tool_T_camera` 标定矩阵。
