# 1. Executive Summary

审计对象是当前工作树中的 `MushroomRobotService`，不是仅按 `HEAD` 还原后的版本。基线为：

- 分支：`main`
- HEAD：`a6c2e46d8c19fdd2e259bc8b6f717599f73b7ccc`
- STM32 gitlink：`6ee9a62b210e798e6199a97889cde424afca6b8f`
- 开始审计时：12 个用户已有的 unstaged 文件；无 staged、无 untracked；STM32 子模块工作树干净
- 基线测试：`595` 项，全部通过，耗时 `1.002s`
- 专项复现：`/tmp/robot_service_state_repro.py`，仅使用 Fake/Mock 和线程事件

结论：当前状态机不满足“任意时刻只有一个高层写操作”的核心不变量。已确认两个运动可同时从 `READY` 通过检查并都提交；旧运动线程可以把较新的 `FAULT` 或 `SHUTDOWN` 覆盖成 `READY`；`stop()` 可把 `CREATED`、`SHUTDOWN`、`DISABLED` 无条件推进到 `READY`；`DISABLED` 又不能直接执行 `enable_joints()`。这些不是文档歧义，而是可重复的实际缺陷。

还确认了以下高风险行为：

- `shutdown()` 不先 stop、不等待活动操作，也没有操作所有权；关闭失败仍报告 `SHUTDOWN`。
- 单轴命令已经提交后，`wait()` 若抛出带 `BUSY` 等“提交前”错误码的异常，仍被误分类为未提交拒绝，最终回到 `READY` 且不 stop。
- startup 在 Home 或 startup pose 阶段失败时会关闭通信 Runtime，但已启用的关节 holding 不会统一回滚，也不会统一 stop；Service 进入不能直接重试 startup 的 `FAULT`。
- `suction()`、`enable_joints()`、`disable_joints()` 执行期间不占用任何中间状态，可与运动并发；异常后的 Service 状态也不一定反映部分硬件改变。

只读诊断方面，`status()`、`get_axis_state()` 和 `get_axis_states()` 在 `EXECUTING`、`DISABLED`、`FAULT` 下没有被 `READY` 门禁错误阻塞；线程复现也没有发现 Service 层死锁。这是当前实现中符合预期的部分。

本轮未修改生产代码，未增加仓库内 characterization test，只新增本报告；未打开串口、CAN 或 Feetech，未执行任何真实硬件命令。

## 审计范围

重点读取并交叉检查：

- `host/application/robot_service.py`
- `host/application/runtime_state.py`
- `host/application/execution_record.py`
- `host/application/controller.py`
- `host/application/pick_workflow.py`
- `host/application/pick_planner.py`
- `host/application/demo_backend.py`
- `host/application/offline_backend.py`
- `host/application/ports.py`
- `host/motion/unified_controller.py`
- `host/motion/unified_protocol.py`
- `host/scripts/robot_service.py`
- `host/scripts/run_motion_demo.py`
- `host/bootstrap.py`
- `host/tests/suites/application/`
- `host/tests/suites/motion/`
- `host/tests/suites/integration/`
- `host/tests/suites/cli/`
- `host/tests/suites/vision/`
- `docs/interfaces/ROBOT_SERVICE_RUNTIME.md`

# 2. Actual State Set

实际枚举定义于 `host/application/runtime_state.py:6-15`：

| 状态 | 实际用途 | 合法进入路径 | 合法/实际退出路径 | 审计结论 |
|---|---|---|---|---|
| `CREATED` | 构造完成，尚未 startup | `__init__` | `startup → STARTING`；但 `stop → READY` | `stop` 可绕过 startup |
| `STARTING` | controller startup 正在执行 | `startup` | 成功 `READY`；异常 `FAULT`；并发 `shutdown` 可写 `SHUTDOWN` | 有实际进入路径，但无所有权保护 |
| `READY` | 顶层接受大部分操作 | startup/操作成功/stop | 多种工作状态、`DISABLED`、`SHUTDOWN`、`FAULT` | 同时承担“生命周期已启动”和“当前空闲”，语义过载 |
| `OBSERVING` | Vision observation 阻塞中 | `request_observation` | 成功或异常均 `READY` | 使用中；异常过宽地都归为 rejected |
| `PLANNING` | Base/Pick 规划中，或非 execute 的 pick validation | plan 方法、非 execute `execute_pick_plan` | 通常 `READY`；未捕获异常可滞留 | 使用中 |
| `EXECUTING` | Base/axis/return/execute pick | 多个写操作 | `READY` 或 `FAULT`；也可被 stop/shutdown 改写 | 使用中但无 operation owner |
| `DISABLED` | `disable_joints()` 成功后的 Service 状态 | 仅 `disable_joints` | `shutdown`；实际 `stop → READY`；不能直接 enable | 近似死状态，恢复路径错误 |
| `FAULT` | startup/执行/stop 故障 | 多个故障路径 | `shutdown`；实际有效 stop 可 `READY` | 可恢复条件没有统一定义 |
| `SHUTDOWN` | Service 认为 Runtime 已关闭 | `shutdown` 的 finally | `startup → STARTING`；实际 `stop → READY` | 可被旧线程或 stop 非法覆盖 |

没有“枚举后完全从未使用”的状态；但 `DISABLED` 的正常退出设计不成立。状态字段还混合了三个正交概念：生命周期、当前操作、硬件健康/holding。

# 3. Current Transition Table

表中“硬件命令”是控制或资源命令；只读状态 I/O 另行标注。状态与模式均按当前源码，不按文档推断。

| 公开方法 | 允许模式 | 允许起始状态 | 中间状态 | 成功终态 | 提交前失败终态 | 运行中失败终态 | 硬件命令 |
|---|---|---|---|---|---|---|---|
| `startup()` | 全部 | `CREATED/SHUTDOWN` | `STARTING` | `READY` | controller 未调用前异常也为 `FAULT` | `FAULT` | depends；默认 read-only 不启动，dry-run 为模拟，execute 为真实 startup |
| `shutdown()` | 全部 | 任意 | 无显式中间态 | `SHUTDOWN` | `SHUTDOWN` | close 抛异常仍 `SHUTDOWN` | started 时 close；不先 stop |
| `status()` | 全部 | 任意 | 不变 | 不变 | 不变 | backend status 异常只写返回字符串 | 可能只读 I/O |
| `capabilities` | 全部 | 任意 | 不变 | 不变 | 不变 | 不变 | 无直接命令 |
| `list_axes()` | 全部 | 任意 | 不变 | 不变 | 不变 | 不变 | 通常无 I/O |
| `get_axis_state(s)()` | 全部 | 任意；dry-run/execute 还要求 `_started_controller` | 不变 | 不变 | 不变 | 不变 | 只读 I/O |
| `move_axis_absolute()` | execute | `READY` 且 runtime/port 可用 | `EXECUTING` | ARRIVED→`READY` | 参数/state/mode/capability：原状态；被分类为 pre-submit 的异常→`READY` | terminal failure/多数异常→`FAULT` | depends；no-op 不提交 |
| `move_axis_relative()` | execute | 同上 | `EXECUTING` | 同上 | 同上 | 同上 | depends |
| `resolve_camera_point()` | 全部 | `READY` | 不变 | `READY` | `READY` | `READY` | 只读五轴状态 |
| `plan_base_target()` | dry-run/execute | `READY` | `PLANNING` | `READY` | `READY` | 所有异常也 `READY` | 不提交运动；可能读取 backend 状态 |
| `move_base_target()` | dry-run/execute | `READY` | `PLANNING→READY`；execute 再写 `EXECUTING` | `READY` | plan 异常→`READY` | execute 异常→stop→`FAULT` | execute 模式提交 |
| `request_observation()` | dry-run/execute | `READY` | `OBSERVING` | `READY` | `READY` | 所有异常也 `READY` | Vision 请求与只读姿态采样，不提交机器人运动 |
| `plan_observation()` | dry-run/execute | `READY` | `PLANNING` | `READY` | 门禁失败保持 `READY` | 所有 planner 异常→`READY` | 不提交运动 |
| `execute_pick_plan()` | 全部 | `READY` | execute→`EXECUTING`；其他→`PLANNING` | 非 FAILED→`READY` | 门禁失败保持 `READY` | workflow 返回 FAILED→`FAULT`；抛异常则滞留中间态 | execute 时最多三段 Base + suction |
| `pick()` | dry-run/execute（由 observation 门禁形成） | `READY` | `OBSERVING→READY→PLANNING→READY→EXECUTING/PLANNING` | `READY` 或 `FAULT` | observe/plan 异常→`READY` | execute 失败→`FAULT`；未捕获异常可滞留 | execute 时提交 |
| `return_to_startup()` | execute | `READY` | `EXECUTING` | `READY` | 门禁失败保持原状态 | stop→`FAULT` | 是 |
| `enable_joints()` | execute | 仅 `READY` | 无 | `READY` | 原状态 | controller 异常仍为 `READY` | 是 |
| `disable_joints()` | execute | 仅 `READY` | 无 | `DISABLED` | 原状态 | controller 异常仍为 `READY` | 是 |
| `suction()` | execute | `READY` | 无 | `READY` | 参数/门禁失败保持原状态 | stop→`FAULT` | 是 |
| `stop()` | 全部 | 任意 | 无 | execute+可验证静止→`READY`；其他非 FAULT 也→`READY` | controller stop 异常→`FAULT` | 状态验证失败→`FAULT` | execute+started 时 stop，随后只读验证 |

源码证据：`host/application/robot_service.py:194-225,227-419,476-684`。

文档 `docs/interfaces/ROBOT_SERVICE_RUNTIME.md:30,55` 描述了单轴提交前拒绝保持 READY、运行故障进入 FAULT，以及 DISABLED 状态；但没有说明：

- 已经提交后的特定异常仍可能被当作 pre-submit rejection；
- `DISABLED` 不能直接 enable；
- `stop` 会把 CREATED/SHUTDOWN/DISABLED 改为 READY；
- shutdown 或 stop 后旧线程可以覆盖终态。

# 4. State and Lock Ownership

## 4.1 当前所有权

- `MushroomRobotService.state`、`fault`、`_started_controller` 都是普通可变字段，Service 没有锁，也没有当前操作 ID、generation 或 cancellation token。见 `host/application/robot_service.py:131-145`。
- 所有门禁都是“读取 state 后返回”；状态写入发生在后续语句。典型窗口是 `host/application/robot_service.py:350-368` 和 `521-539`。
- `UnifiedMotionController` 有一个 `threading.RLock`，见 `host/motion/unified_controller.py:185-187`。绝对/相对/多轴提交在该锁内完成，见 `539-612`。
- `wait()` 不跨整个等待周期持锁；每次 poll 经 `get_command_result()` 短暂获取控制器锁，见 `619-653,714-729`。因此 stop/status 可以并行进入。
- `UnifiedMotionController.stop()` 本身不获取 `_lock`，直接访问 backend，见 `813-873`。它可能与锁内的 backend 查询/提交并发；是否安全依赖各 driver 的内部实现，顶层没有统一约束。
- `JsonLinesExecutionRecorder` 没有线程锁，每次直接 open-append-write，见 `host/application/execution_record.py:24-46`。

## 4.2 锁顺序与死锁

当前没有 Service lock，因此不存在可证明的 `Service lock → controller lock → recorder lock` 固定顺序，也没有发现现存的 Service 级锁反转。Fake 复现中，运动等待期间 `stop()`、`status()`、`get_axis_states()` 均能返回，没有死锁。

这不等于线程安全：当前主要风险是缺少互斥和终态所有权，而非锁反转。底层控制器锁内会调用 backend I/O，另一线程的无锁 stop 也会调用同一 backend；真实 driver 是否允许这种并发需要单独验证。

# 5. Confirmed Defects

## SM-001 — READY 检查与操作注册不原子，可同时提交两个真实运动

Severity: **Critical**  
Status: **Confirmed defect**  
Affected methods: `move_axis_absolute/relative`、`move_base_target`、`return_to_startup`、`execute_pick_plan`，并扩展到所有从 READY 开始的写操作

Current transition: 两线程分别读取 `READY` → 各自继续 → 都写 `EXECUTING`/规划状态 → 都提交。  
Expected transition: 第一个线程原子占有写操作后，第二个线程必须在提交前得到 busy/state rejection。

Reproduction:

1. Fake barrier 放在 `_require_ready()` 返回前。
2. 线程 A 调用 `move_axis_relative(z, -10)`；线程 B 调用 `move_base_target(...)`。
3. 同时释放 barrier。
4. 观察到 `axis_submitted=True` 且 `base_submitted=True`，两线程均无异常。

Evidence:

- `host/application/robot_service.py:350-368`
- `host/application/robot_service.py:521-539`
- trace：`R1 simultaneous: {'axis_submitted': True, 'base_submitted': True, 'final': 'ready', 'errors': []}`

Root cause: Service 没有状态锁；检查与写入不在同一个临界区。底层 controller 的 per-axis/submit 锁不能串行化 Base 与 raw-axis 两条高层入口。  
Hardware command submitted: **yes（Fake 中两条都提交；execute 实机路径对应真实提交）**  
Possible consequence: 两套规划/单轴命令同时改变不同或重叠执行器，破坏 Base-frame 路径假设和碰撞/工作区约束。  
Minimal fix direction: 短持有 state lock，原子执行“检查→分配 operation token→设置 operation state”；阻塞 I/O 前释放锁。  
Required regression tests: axis-vs-base、base-vs-pick、suction/holding-vs-motion 的 barrier 并发测试，断言最多一个提交。

## SM-002 — 旧操作线程可覆盖较新的 FAULT 或 SHUTDOWN

Severity: **Critical**  
Status: **Confirmed defect**  
Affected methods: 所有在阻塞调用返回后无条件写终态的方法，已实测 axis move；同类代码存在于 Base move、return、pick/plan/observe

Current transition:

- `EXECUTING --stop verification failure--> FAULT --old wait ARRIVED--> READY`
- `EXECUTING --shutdown--> SHUTDOWN --old wait ARRIVED--> READY`

Expected transition: stop/shutdown 使当前 operation token 失效；旧操作只记录结果，不能再提交 Service 终态。

Reproduction:

1. 线程 A 在 Fake `wait()` 阻塞。
2. 主线程执行 stop 并令状态验证失败，得到 `FAULT`；或执行 shutdown，得到 `SHUTDOWN`。
3. 释放旧 wait，使其返回 ARRIVED。
4. 最终状态变成 `READY`。

Evidence:

- `host/application/robot_service.py:368-419,618-655,209-216`
- trace：`after_stop=('fault', ...)`，随后 `final=('ready', 'stop did not confirm valid stationary axes')`
- trace：`after_shutdown='shutdown'`，随后 `final='ready', runtime_started=False`

Root cause: 无 operation generation/token；终态写入不检查当前状态和操作所有权。  
Hardware command submitted: **yes**  
Possible consequence: Service 对外宣称 READY，但 Runtime 已关闭，或硬件状态不可信且 `fault` 仍非空。  
Minimal fix direction: 每个写操作持有 token；仅 token 仍为 current 且 lifecycle 未 shutdown 时才能提交 READY/FAULT。  
Required regression tests: stop→late success、stop→late exception、shutdown→late success、shutdown→late exception。

## SM-003 — stop 可把 CREATED、SHUTDOWN、DISABLED 复活为 READY

Severity: **High**  
Status: **Confirmed defect**  
Affected methods: `stop()`

Current transition: 当不满足 execute+started+state-reader 分支，且当前不是 FAULT 时，无条件写 `READY`。  
Expected transition: CREATED/SHUTDOWN 保持生命周期状态；DISABLED 保持 holding 状态；只有被 stop 的活动操作在验证静止后才恢复到可定义的空闲状态。

Reproduction: 分别在 CREATED、已 shutdown、已 disable 的 Service 上调用 stop，结果均为 READY。

Evidence:

- `host/application/robot_service.py:618-655`
- trace：`R9 ... {'created_to': 'ready', 'shutdown_to': 'ready', 'disabled_to': 'ready'}`

Root cause: stop 的 fallback 分支没有按生命周期/holding 状态分类，也没有保存“stop 前稳定状态”。  
Hardware command submitted: **depends**；未 started 时不提交，DISABLED+started 时会发 stop。  
Possible consequence: 未 startup 或已关闭 Runtime 被标成 READY；关节仍 disabled 却被标成 READY，随后顶层门禁与 backend 状态矛盾。  
Minimal fix direction: stop 是控制信号，不应无条件成为生命周期转换；根据 active token 和验证结果提交终态，非活动状态保持不变。  
Required regression tests: CREATED/READY/EXECUTING/DISABLED/FAULT/SHUTDOWN × stop 矩阵。

## SM-004 — DISABLED 无法直接 enable，形成错误恢复路径

Severity: **High**  
Status: **Confirmed defect**  
Affected methods: `disable_joints()`、`enable_joints()`、`startup()`、`stop()`

Current transition: `READY --disable--> DISABLED`；`enable_joints()` 又调用 `_require_execute()`，其内部要求 READY，因此 `DISABLED --enable` 被拒绝；startup 也只接受 CREATED/SHUTDOWN。当前只能借助有缺陷的 `stop(): DISABLED→READY` 再 enable。  
Expected transition: `DISABLED --enable + 真实状态重读/验证--> READY`，失败则保持 DISABLED 或进入 FAULT，取决于硬件可信度。

Evidence:

- `host/application/robot_service.py:657-666,686-697`
- trace：`RobotServiceStateError: joints enable requires READY, got disabled`
- CLI 用户实测同样得到该错误。

Root cause: enable 与一般 execute operation 共用了“必须 READY”的门禁，没有为 DISABLED 定义恢复转换。  
Hardware command submitted: **no（被 Service 拒绝）**  
Possible consequence: 正常 holding 恢复不可达，用户被迫重启或利用 stop 的错误状态转换。  
Minimal fix direction: enable 只允许 DISABLED（是否允许 READY 幂等需明确），成功后重读三关节使能和全轴有效位置，再进入 READY。  
Required regression tests: disable→enable success、部分 enable failure、反馈未知、重读位置失败。

## SM-005 — shutdown 不协调活动操作，且 close 失败仍报告 SHUTDOWN

Severity: **High**  
Status: **Confirmed defect**  
Affected methods: `shutdown()`

Current transition: 任意状态直接 close；不先 stop、不失效活动操作、不等待；`finally` 无条件 `_started_controller=False` 和 `state=SHUTDOWN`。  
Expected transition: shutdown 先阻止新操作并取消/stop 活动操作；资源关闭结果必须可区分 complete 与 failed/partial。

Reproduction:

- 活动 axis wait 期间 shutdown：未观察到 stop，close 返回后状态 SHUTDOWN；旧线程随后覆盖为 READY。
- Fake close 抛 `RuntimeError('close failed')`：调用方收到异常，但 Service 仍为 SHUTDOWN 且 `_started_controller=False`。

Evidence:

- `host/application/robot_service.py:209-216`
- `host/application/demo_backend.py:132-133`
- `host/bootstrap.py:108-127`（实际 Runtime 会尝试关闭全部资源并聚合错误）
- trace：`R4`、`R7`

Root cause: shutdown 的 finally 把“关闭已尝试”等同于“关闭成功”；没有 lifecycle closing 状态或 close health。  
Hardware command submitted: **close yes；stop no**  
Possible consequence: 外部认为资源已释放，但部分资源 close 失败；旧线程继续访问已关闭 backend。  
Minimal fix direction: 在短锁内设置 shutdown intent 并失效 token；必要时 stop；协调活动线程；close 失败保留 fault/close_error，不能宣称健康 SHUTDOWN。  
Required regression tests: READY/EXECUTING/FAULT shutdown、close partial failure、重复 shutdown、late return。

## SM-006 — 已提交单轴异常可能被误判为 pre-submit rejection

Severity: **Critical**  
Status: **Confirmed defect**  
Affected methods: `_move_axis()`

Current transition: submit 已成功，`submitted=True`；wait 抛 `UnifiedMotionError(BUSY)`；代码仅按 error code 判断 `_is_pre_submission_rejection()`，写 READY 且不 stop。  
Expected transition: 一旦 handle 已返回，后续异常必须按运行中故障/取消处理，best-effort stop 并进入 FAULT 或明确 cancelled 终态。

Evidence:

- `host/application/robot_service.py:369-392,799-811`
- trace：`R10 ... {'submitted': True, 'stop_called': False, 'final': 'ready', ...}`

Root cause: 分类函数只看异常类型/错误码，不使用本地 `submitted` 标志。  
Hardware command submitted: **yes**  
Possible consequence: 底层命令仍活动，Service 却重新接受运动。  
Minimal fix direction: 先按 `submitted` 分界；只有 `submitted is False` 才允许 pre-submit rejection 回稳定状态。  
Required regression tests: 每个 pre-submit code 在 submit 阶段与 wait 阶段各一组，断言 stop/state/record。

## SM-007 — startup 后段失败会留下 holding，且 FAULT 不能直接重试

Severity: **High**  
Status: **Confirmed defect**  
Affected methods: `startup()`、`DemoFlowApplicationBackend.startup()`、`DemoMotionFlow.startup()`

Current transition: Runtime open 后，流程依次 suction idle、holding enable、Z Home、Slide Home、startup pose。flow 任意异常由 adapter close Runtime，Service 写 FAULT；但 Home/startup pose 失败后没有统一 stop/disable holding，且 startup 只接受 CREATED/SHUTDOWN。  
Expected transition: 每阶段定义补偿；已提交运动失败至少 stop；holding 是否保留必须显式、安全且可观测；失败后提供受控 retry/recover 路径。

Evidence:

- `host/application/demo_backend.py:38-44`
- `host/scripts/run_motion_demo.py:288-355,670-728`
- `host/application/robot_service.py:194-207`
- Fake 矩阵：Z Home、Slide Home、startup pose 失败后均 `state=fault`、Runtime close、`holding_after=True`、`stop_called=False`，再次 startup 被状态门禁拒绝。

Root cause: Runtime 资源回滚存在，但机器人动作/holding 补偿没有由 startup coordinator 统一拥有；Service 的 `_started_controller` 仅在全部成功后置 true。  
Hardware command submitted: **按失败阶段 depends；后段为 yes**  
Possible consequence: 通信关闭但执行器继续保持；操作员无法从 Service 状态判断补偿是否完成。  
Minimal fix direction: startup 建立阶段账本；后段失败执行 best-effort stop，并记录 holding/资源关闭结果；定义 FAULT recovery。  
Required regression tests: open、suction、部分 holding enable、Z Home、Slide Home、startup pose 六阶段矩阵。

## SM-008 — holding 与 suction 操作不占用写操作状态，异常可留下状态/硬件不一致

Severity: **High**  
Status: **Confirmed by source path; concurrency test gap remains for each pair**  
Affected methods: `enable_joints()`、`disable_joints()`、`suction()`

Current transition: 三者从 READY 调用 backend 时仍保持 READY。另一线程可同时通过 READY 门禁并发起运动。disable backend 按 Rotation→Elbow→Shoulder 逐个移除 holding，任一步失败可形成部分 disabled；Service 因没有 except，仍保持 READY。  
Expected transition: 都应注册为互斥高层写操作；部分硬件改变失败必须通过反馈决定 DISABLED/FAULT，而不是沿用 READY。

Evidence:

- `host/application/robot_service.py:657-684`
- `host/motion/unified_controller.py:436-537`

Root cause: 只有显式运动被赋予 EXECUTING，其他改变机器人状态的命令未进入统一 operation 边界。  
Hardware command submitted: **yes**  
Possible consequence: 运动期间移除 holding 或切换 suction；部分 disable 后 Service 仍接受运动。  
Minimal fix direction: 使用统一 write-operation helper，不要求新增大型异步框架。  
Required regression tests: disable-vs-move、enable-vs-move、suction-vs-move barrier；逐关节 disable failure。

## SM-009 — execute_pick_plan 未捕获异常时会滞留 EXECUTING/PLANNING

Severity: **High**  
Status: **Confirmed defect by control flow**  
Affected methods: `execute_pick_plan()`、`pick()`

Current transition: 先写 EXECUTING/PLANNING，再直接调用 workflow；只有 workflow 正常返回 `PickResult` 才写终态。编程错误、recorder 错误或未被 workflow 捕获的异常会传播并保留中间状态。  
Expected transition: 所有退出路径必须提交明确终态；execute 中未知异常应 best-effort stop 并 FAULT，纯 planning 编程错误至少不能永久占用 PLANNING。

Evidence: `host/application/robot_service.py:586-603`。  
Root cause: 缺少统一 operation `try/except`/token 完成路径。  
Hardware command submitted: **depends**  
Possible consequence: Service 永久拒绝后续操作，或异常发生在已提交阶段却未 stop。  
Minimal fix direction: 统一 public operation wrapper；workflow 内部步骤继续用 controller/internal helper，避免重复状态边界。  
Required regression tests: workflow 在提交前、第一段后、suction 后、retreat 后抛异常。

# 6. Design Ambiguities

## DA-001 — FAULT 恢复条件

当前 `stop()` 在所有轴 `busy=False`、无 fault、position_valid 时清除 `fault` 并进入 READY，甚至不检查 connected、homed、position unit/current position，也不检查 rotary holding。见 `host/application/robot_service.py:631-652`。需要明确：FAULT 是“活动运动故障”还是“硬件健康故障”；哪些故障允许 stop 后恢复。

## DA-002 — DISABLED 是 Service 状态还是硬件健康维度

DISABLED 只描述三个旋转关节 holding，但它替代了整个 Service lifecycle/operation state。线性轴状态、资源打开状态和诊断能力仍存在。更合理的是独立的 holding/health 字段；最小修复阶段至少必须定义 DISABLED 的 enter/exit。

## DA-003 — planning/observation 的所有异常是否都应回 READY

`plan_base_target`、`request_observation`、`plan_observation` 捕获所有 `Exception` 并标为 rejected/READY。参数或无解属于提交前拒绝；通信丢失、backend 编程错误、recorder 错误未必应该被视为健康 READY。需要异常分类合同。

## DA-004 — shutdown close error 的终态名称

Runtime `close()` 会尽力关闭全部资源，即使最终抛 `HardwareCloseError`，其 `_is_open` 也为 false。Service 需要区分“资源关闭尝试完成但存在 close error”和“全部关闭成功”；可以保留 lifecycle=SHUTDOWN 同时 health=FAULT/close_error，但不能仅靠当前单一 state 表达。

## DA-005 — Web/API 请求取消

同步方法阻塞等待。调用线程被 Web 服务器取消或连接断开，并不会自动取消底层命令；当前也没有 operation handle/cancellation token。应明确上层取消只取消等待还是同时请求 robot stop。

# 7. Missing Tests

现有测试覆盖了单线程正常/错误路径：axis pre-submit rejection、terminal timeout、stop 后有效/无效状态，以及底层相对提交锁。证据见：

- `host/tests/suites/application/test_robot_service.py:359-450`
- `host/tests/suites/motion/test_unified_controller.py:400-425`

缺少：

- Service 级两个写操作同时通过 READY 的并发测试；
- stop/shutdown 与阻塞 wait 的交错；
- 旧线程迟到覆盖 FAULT/SHUTDOWN；
- 已提交后抛“pre-submit error code”的分类测试；
- CREATED/SHUTDOWN/DISABLED/FAULT 的 stop 矩阵；
- DISABLED→enable；
- shutdown close failure和活动操作协调；
- startup 六阶段完整补偿矩阵；
- holding/suction 与运动并发；
- execution recorder 多线程和 request/terminal 配对；
- execute_pick_plan 未预期异常；
- capability 的 supported/currently_available 合同。

# 8. Concurrency Reproductions

命令：

```bash
cd /Users/sd/Projects/mushroom-picking-platform/host
PYTHONPATH=. .venv/bin/python /tmp/robot_service_state_repro.py
```

摘要：

| 场景 | 结果 | 结论 |
|---|---|---|
| axis relative 与 Base move 同时越过 READY | 两者都提交 | Critical defect |
| axis wait 阻塞时 stop，验证无效 | stop 立即返回 FAULT，无死锁 | stop 可并行 |
| 上述旧 wait 后返回 ARRIVED | 最终 READY，fault 字符串仍在 | Critical stale overwrite |
| axis wait 阻塞时 status/get_axis_states | 返回 EXECUTING 和 5 轴状态 | 诊断不被错误阻塞 |
| axis wait 阻塞时 shutdown | 未先 stop；先 SHUTDOWN，后被旧线程改 READY | Critical race |
| stop 成功后旧 wait 返回 TIMEOUT | READY 又变 FAULT | 最终状态取决于调度 |
| close 抛异常 | state 仍 SHUTDOWN | lifecycle/health 不一致 |
| wait 抛 BUSY，handle 已提交 | READY 且未 stop | Critical misclassification |

复现脚本位于 `/tmp`，未加入仓库、未提交；每次使用新 Service/Fake，所有等待都有 1 秒上限。

# 9. Startup/Shutdown Failure Matrix

## 9.1 Startup

| 注入阶段 | 是否打开后关闭 Runtime | holding 最终值（Fake） | 是否统一 stop | Service 终态 | 可直接重试 startup |
|---|---:|---:|---:|---|---:|
| Runtime open | 否；open 自身负责已打开资源回滚 | false | 否 | FAULT | 否 |
| suction idle | 是 | false | 否 | FAULT | 否 |
| holding enable 抛异常 | 是 | false（注入发生在改变前） | 否 | FAULT | 否 |
| Z Home timeout | 是 | true | 否 | FAULT | 否 |
| Slide Home timeout | 是 | true | 否 | FAULT | 否 |
| startup pose timeout | 是 | true | 否 | FAULT | 否 |

补充：底层 `enable_rotary_joints()` 对“本次新启用”的关节有 best-effort rollback，见 `host/motion/unified_controller.py:371-434`；但 startup 后续阶段失败不调用 disable rollback。Runtime open 的资源逆序回滚由 `host/bootstrap.py:79-138` 实现并有离线测试。

## 9.2 Shutdown

| 起始条件 | 当前行为 | 风险 |
|---|---|---|
| READY | 直接 close→SHUTDOWN | 正常路径可工作 |
| EXECUTING | 不 stop、不等待，直接 close | 活动线程访问关闭资源；终态竞态 |
| FAULT | 直接 close→SHUTDOWN | fault 原因仍保留但 state 不表达 health |
| 重复 shutdown | `_started_controller=False` 后只重复写记录 | 大体幂等，但可能掩盖首次 close 失败 |
| close 部分失败 | 仍 `_started_controller=False`、SHUTDOWN，异常上抛 | 对外状态声称过强 |

# 10. Stop and Fault Semantics

当前 stop 的积极面：

- execute+started 时先调用 controller stop；
- 随后读取五轴；只有全部 `busy is False`、无 fault、position_valid 才恢复 READY；
- 状态无法确认时进入 FAULT；
- wait 不长期持底层锁，Fake 中 stop 可立即执行。

当前缺陷：

- 不关联 active operation，无法阻止旧线程写终态；
- fallback 会把 CREATED/SHUTDOWN/DISABLED 改 READY；
- 验证条件没检查 connected、homed、finite position、position unit、holding；
- controller stop 是全局 best-effort，而 axis timeout 只 stop 单轴；Rotation 没有可靠独立软件 stop；
- stop 记录只写 `final_status`，没有逐轴验证结果或 active operation token；
- stop 成功会无条件清空 fault，即使 fault 与运动静止无关。

建议把 stop 定义为“对当前 operation 发取消/停止信号并验证后端”，而不是通用的“把 Service 改 READY”方法。

# 11. Capability Consistency

当前 axis capability 同时混合 supported 和 currently available：

- `axis_listing` 只看方法是否存在；
- `axis_state_query` 还看 mode/runtime 是否 started；
- `axis_absolute_motion/relative_motion` 还看 execute、READY、started 和 callable。

证据：`host/application/robot_service.py:147-188`。

因此 `axis_absolute_motion=False` 可能表示“不支持”、Runtime 未启动、Service 正忙、FAULT 或 DISABLED，调用方无法区分。另一方面 `base_frame_motion`、`joint_holding`、`suction_command` 主要反映 backend 静态能力，在 FAULT/SHUTDOWN 仍可能为 true，与 axis 字段语义不一致。

这是 **Medium / Design ambiguity**。建议 DTO 最终区分：

- `supported`：代码和 backend 能力；
- `currently_available`：mode、lifecycle、operation、health 门禁后的当前可调用性；
- 可选 `unavailable_reason`。

最小状态机修复不必立即扩展所有 capability DTO，但必须先统一字段语义并加测试。

# 12. Execution Record Consistency

当前 recorder 在显式路径时可写 commit、state、输入和终态；单轴请求还记录 `submitted/no_op/terminal_outcome`。但仍有以下问题：

1. 大多数高层命令没有“开始/accepted”记录，只在成功或异常后记录；进程退出时无法知道是否已有命令在途。
2. 没有 Service operation ID/generation。Vision request_id 只覆盖视觉链，startup/Base/axis/stop/shutdown 没有统一唯一 ID。
3. axis 的 `submitted` 局部变量是准确线索，但异常分类没有使用它；因此可出现记录 `submitted=true, final_status=rejected`、Service READY、实际未 stop。
4. stop 只记录最终 Service state，不记录 controller stop 是否成功、逐轴状态、验证失败原因或被取消的 operation。
5. shutdown close 失败时 finally 仍记录 `final_status=shutdown`，随后异常上抛；记录声称比事实更强。
6. `return_to_startup()` 和 `enable_joints()/disable_joints()` 成功没有 execution record；`suction()` 成功也没有记录。
7. `move_base_target()` 会先单独记录 plan，再记录 move；这是可接受的两阶段记录，但没有共同 operation ID。
8. `pick()` 顺序调用公开 observe/plan/pick，会生成多条记录；当前没有状态自阻塞，但缺少 parent operation ID，难以关联。
9. recorder 没有线程同步；并发命令可能产生顺序与状态快照不一致，`host/application/execution_record.py:34-46`。

分级：**Medium / Confirmed defect + design gap**。

# 13. Minimal Fix Design

适合当前同步阻塞 API 的最小方案，不引入大型异步框架：

## 13.1 分离最少的状态概念

- lifecycle：`CREATED / STARTING / OPEN / SHUTTING_DOWN / SHUTDOWN`
- current operation：`None` 或 `{id, kind, phase, cancellation_requested}`
- health：`healthy / disabled / fault`，并保存 fault detail

若短期不改公开枚举，内部也至少应维护以上三个字段，再由它们派生旧 `RobotServiceState`。

## 13.2 两类锁

- `state_lock`：普通 `Lock` 或 `RLock`，只保护字段和 token，持有时间必须很短；禁止在锁内做 backend I/O、wait 或 recorder I/O。
- `operation_lock`/单命令串行器：用于确保高层写操作只能有一个 owner。可由非阻塞 acquire 实现 busy rejection；不要让 stop/status 等待这个锁。

推荐顺序：先短暂获取 `state_lock` 注册 token，再释放；不在持锁状态下获取 controller/backend/recorder 锁，因此避免形成长锁链。

## 13.3 操作模板

```text
public write operation
  state_lock:
    校验 lifecycle/mode/health/current operation
    创建唯一 token
    current_operation = token
    派生 EXECUTING/PLANNING/...
  unlock

  执行可能阻塞的 backend 调用

  state_lock:
    仅当 token 仍为 current 且 shutdown intent 未覆盖时提交终态
    清理 current_operation
  unlock
  写 terminal record
```

提交前/运行中分界必须由“是否获得 command handle/是否发生硬件副作用”决定，不只看异常码。

## 13.4 stop

```text
state_lock: 读取 current token，并标记 cancellation_requested
unlock
backend stop + 轴状态验证
state_lock:
  使 token 失效
  根据 lifecycle、health、验证结果提交终态
unlock
```

stop 不应修改 CREATED/SHUTDOWN；DISABLED stop 后仍应保持 disabled health。旧线程发现 token 失效后只能记录 late terminal result，不得写 READY/FAULT。

## 13.5 shutdown

1. 原子设置 shutdown intent，拒绝新写操作。
2. 若有 active operation，发 best-effort stop/cancel；不要持 state lock 等待。
3. 等待/协调 operation owner 到安全点，使用有限超时。
4. close Runtime。
5. 分别记录 close completed 与 close failed；即使 lifecycle 最终为 SHUTDOWN，也保留 health/close_error。

## 13.6 内部 helper 与嵌套流程

公开方法只管理一次顶层状态边界。Pick 内部继续调用 controller/workflow 的不管理 Service 状态 helper；不要在 outer operation 已占有 token 时再调用另一公开写方法。当前 `pick()` 是三个公开操作串联而非一个原子 pick；修复时需明确要“整次 pick 独占”还是允许 observe 与 plan 之间插入别的操作。生产抓取更适合整次 pick 一个 parent token，内部阶段只更新 phase。

## 13.7 Web 调用

外部仍只暴露 `MushroomRobotService`。Web/GUI 不直接持有 controller；同步请求可获得 operation ID。请求取消不应默默遗失底层运动，必须显式选择：继续后台完成，或调用 token-aware stop。

# 14. Required Regression Tests

最小修复验收集合：

1. 两个写操作同一 barrier 从 READY 出发，最多一个 backend submit。
2. axis/base/pick/return/suction/holding 的两两关键并发组合。
3. move wait 中 stop 成功：stop 可及时返回；旧线程不能覆盖 stop 终态。
4. move wait 中 stop 验证失败：保持 FAULT，late success/exception 均不能覆盖。
5. move wait 中 shutdown：先阻止新命令，按策略 stop/等待，最终不会回 READY。
6. timeout 与 stop 两种调度顺序得到同一确定终态。
7. submit 前 BUSY 与 submit 后 wait BUSY 的分类不同；后者必须 stop/FAULT。
8. CREATED、READY、EXECUTING、DISABLED、FAULT、SHUTDOWN 的 stop 表驱动测试。
9. DISABLED→enable：重读 holding/轴位置成功才 READY；失败不进入 READY。
10. startup 六阶段失败矩阵，断言资源、stop、holding、state、retry policy。
11. shutdown close partial failure、重复 shutdown、startup after clean shutdown。
12. status/get axis/capabilities 在 READY/EXECUTING/DISABLED/FAULT 可读取且不等待 operation lock。
13. execute_pick_plan 各阶段抛异常；无中间态泄漏。
14. recorder：每个 command 有唯一 ID、accepted/submitted/terminal 配对，stop/shutdown 记录真实结果。
15. recorder 并发写入 JSON Lines，每行可独立解析且 operation ID 不重复。

# 15. Recommended Implementation Order

1. 先添加上述 characterization/regression tests，锁定 SM-001、SM-002、SM-003、SM-006。
2. 引入短持有 `state_lock`、current operation token 和统一 write-operation helper；先覆盖 axis/Base/return/pick。
3. 让 stop token-aware，消除旧线程覆盖；补全 stop 状态矩阵。
4. 协调 shutdown 与活动 operation，区分 close success/failure。
5. 修复 DISABLED→enable，并让 holding/suction 进入统一写操作边界。
6. 建立 startup 阶段账本和失败补偿矩阵。
7. 统一异常分类：门禁/提交前拒绝、已提交运行故障、编程错误。
8. 最后统一 capability 的 supported/currently_available 语义和 execution record schema。

Robot Service state-machine audit completed.

No production state transition, robot parameter, calibration value, firmware,
Git submodule, or real hardware state was modified.

No real hardware command was issued.

Waiting for review before implementing state-machine fixes.
