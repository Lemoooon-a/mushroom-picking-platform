# Vision Pick Workflow

状态：软件框架 implemented、offline-tested；真实视觉运动和自动采摘 hardware-blocked/unavailable。

## 分层与顺序

```text
VisionGateway → VisionTargetObservation → VisionTargetResolver
              → PickPlanner → MushroomRobotController
              → UnifiedMotionController
```

`VisionPickWorkflow` 同步执行以下顺序：确认关节 holding、读取静止五轴状态、创建 `CaptureSnapshot`、请求同一 `request_id` 的视觉结果、再次确认状态未变、解析目标、完整规划 pre-grasp/contact/retreat；仅在 execute 模式依次执行 pre-grasp、contact、`suction_grip()`、retreat。

`VisionGateway` 不做坐标变换，`VisionTargetResolver` 不做规划，`PickPlanner` 不访问硬件，`MushroomRobotController` 仍是 Base 工作区与规划门禁，`UnifiedMotionController` 不包含采摘流程。

## PickPlan 原子性

| 阶段 | Base Z | TrayWorkspace | 其他约束 |
| --- | --- | --- | --- |
| pre-grasp | object Z + `approach_offset_mm` | 不应用最终任务 Z 门限 | Base solver、OffsetWorkspace、轴/关节限位、RobotMotionEnvelope |
| contact | object Z + `contact_offset_mm` | 必须通过 | 同上 |
| retreat | object Z + `retreat_offset_mm` | 不应用最终任务 Z 门限 | 同上 |

三个目标都在规划阶段验证；任何一个失败都不返回部分 `PickPlan`。高位阶段只绕过 Tray 的最终目标门限，仍复用同一个 Base 规划出口。

## 结果语义

dry-run 返回 `PLANNED` 且零 submit。execute 阶段失败会 best-effort stop 并返回 `FAILED`；完成 retreat 后返回 `PHYSICAL_PICK_UNVERIFIED`。当前没有真空反馈，因此不得报告 object successfully picked。第一版不自动放置、释放、重试或修改失败目标。

视觉 timeout、no target、低 confidence、过期 observation、hand-eye/grasp 缺失、工作区拒绝和规划失败都不运动并回到 READY；运动或吸盘命令失败进入 FAULT。
