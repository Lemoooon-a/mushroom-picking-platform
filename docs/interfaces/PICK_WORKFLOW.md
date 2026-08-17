# Vision Pick Workflow

状态：软件框架 implemented、offline-tested；真实视觉运动和自动采摘 hardware-blocked/unavailable。

## 分层与顺序

```text
VisionGateway → VisionTargetObservation → VisionTargetResolver
              → PickPlanner → MushroomRobotController
              → UnifiedMotionController
```

`VisionPickWorkflow` 同步执行以下顺序：确认关节 holding、读取静止五轴状态、创建 `CaptureSnapshot`、请求同一 `request_id` 的视觉结果、再次确认状态未变、解析目标、完整规划 overhead/contact/lift；仅在 execute 模式依次执行 overhead、contact、`suction_grip()`、稳定等待、lift。

`VisionGateway` 不做坐标变换，`VisionTargetResolver` 不做规划，`PickPlanner` 不访问硬件，`MushroomRobotController` 仍是 Base 工作区与规划门禁，`UnifiedMotionController` 不包含采摘流程。

## PickPlan 原子性

| 阶段 | Base Z | TrayWorkspace | 其他约束 |
| --- | --- | --- | --- |
| overhead | 共享工作高度 150 mm | 不应用最终任务 Z 门限 | Base solver、OffsetWorkspace、轴/关节限位、RobotMotionEnvelope |
| contact | object Z + `contact_offset_mm` | 必须通过 | 同上 |
| lift | 共享工作高度 150 mm | 不应用最终任务 Z 门限 | 同上 |

三个目标都在规划阶段验证；任何一个失败都不返回部分 `PickPlan`。高位阶段只绕过 Tray 的最终目标门限，仍复用同一个 Base 规划出口。
contact 必须严格低于 150 mm，以保证 contact 段为下探运动。吸盘开启后按计划中的 `suction_settle_time_s` 等待，当前实机配置为 2 秒；等待期间取消后不会提交 lift。

## 结果语义

dry-run 返回 `PLANNED` 且零 submit、零等待。execute 阶段失败会 best-effort stop 并返回 `FAILED`；完成 lift 后返回 `PHYSICAL_PICK_UNVERIFIED`。当前没有真空反馈，因此不得报告 object successfully picked。单次 pick 不自动放置；scan-pick 在 lift 后按视觉 `size_class` 移动到普通 Base 放置点 `(150, 1000, 150, 0)` 或过大目标放置点 `(450, 1000, 150, 0)`，立即释放并直接返回扫描位，不执行放置前接近或放置后回撤。

视觉 timeout、no target、低 confidence、过期 observation、hand-eye/grasp 缺失、工作区拒绝和规划失败都不运动并回到 READY；运动或吸盘命令失败进入 FAULT。
