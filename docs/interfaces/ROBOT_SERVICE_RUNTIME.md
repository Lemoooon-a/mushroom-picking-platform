# Robot Service Runtime

状态：implemented、offline-tested；execute 接口保留但本轮未运行真实硬件。

`MushroomRobotService` 是进程级唯一应用入口，持有 runtime、`MushroomRobotController`、Vision Gateway、`VisionPickWorkflow`、应用状态和可选 JSON Lines recorder。它不重新实现正运动学（Forward Kinematics, FK）、逆运动学（Inverse Kinematics, IK）或执行器协议。

## API

```python
startup()
shutdown()
status() -> RobotServiceStatus
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

状态为 `CREATED → STARTING → READY`，观察使用 `OBSERVING`，规划使用 `PLANNING`，真实动作使用 `EXECUTING`，关节失能后为 `DISABLED`，不可恢复执行错误为 `FAULT`，关闭后为 `SHUTDOWN`。视觉/质量/规划拒绝回 READY；motion/suction failure 会 best-effort stop 后进入 FAULT。

## 模式

- `read-only`：加载并检查配置、status/capabilities/workspace；不构造硬件 runtime，不打开硬件。
- `dry-run`：使用纯配置 `OfflinePlanningBackend`、FakeVisionGateway、真实 FK/IK/工作区/transition planner；后端没有硬件 submit API。
- `execute`：使用现有 `DemoMotionFlow`、`UnifiedMotionController` 和 `MotionAuthorization`，CLI 还要求 `--confirm-motion --confirm-rotation-no-stop`。本轮未执行。

入口：

```bash
cd host
.venv/bin/python scripts/robot_service.py --mode read-only
.venv/bin/python scripts/robot_service.py --mode dry-run --fake-position X Y Z
.venv/bin/python scripts/robot_service.py --mode execute \
  --confirm-motion --confirm-rotation-no-stop
```

支持 `status`、`capabilities`、`workspace`、`startup`、`return`、`stop`、`move`、`plan`、`observe`、`plan-observation`、`pick`、`suction`、`joints`、`quit` 和 `help`。

`--record-jsonl PATH` 才会写记录；默认不写仓库。记录包含版本、状态、输入、计划、阶段结果和错误，不记录 `tool_T_camera` 标定矩阵。
