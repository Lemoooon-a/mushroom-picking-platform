# Host 配置职责

本仓库只服务当前机械臂，不提供 local、example 或多机器 profile。所有正式配置直接位于
`host/config/` 并由 Git 跟踪；导入纯配置模块不会打开串口、CAN（Controller Area Network，
控制器局域网）或创建 runtime。

## 正式配置

| 文件 | 角色 |
| --- | --- |
| `robot_runtime.json` | 唯一业务 Runtime 配置，包含 frame transforms、Tray workspace、视觉连接、抓取、扫描放置和 JSONL 记录 |
| `robot_hardware.py` | 当前机械臂 USB/CAN 设备身份与通信参数 |
| `robot_motion.py` | 到位、timeout、速度、加速度和直线轴范围 |
| `robot_geometry.json` | 连杆长度与 Z=0 时 TCP Base 高度 |
| `hardware.py` / `motion_runtime.py` / `robot_runtime.py` | 强类型模型和 loader |
| `frame_transforms.py` / `tray_workspace.py` | 可复用的区块模型与校验器 |
| `project/` | 关节、Rotation、工作区规划和运动阶段等代码策略 |

Robot Service 与 Web API 固定读取 `robot_runtime.json`，不接受配置路径覆盖。该文件的六个区块
全部必需且必须同时通过校验；任何错误都会在构造硬件 runtime 前终止启动。标定和诊断工具可用
`--config` 指向临时的完整 Runtime 文件，写入时只原子更新 `frame_transforms` 区块。

密钥、令牌和密码不得写入这些文件；未来如需凭据，应使用环境变量或专用密钥管理方式。
`robot_runtime.json` 中的 Tray workspace 只描述 Base frame 最终任务 TCP 门限，不是机械极限；
`project/robot_motion_envelope.py` 也不是碰撞模型或安全认证。

## 三类工作区与安全策略

| 配置 | 坐标系 | 应用对象 | 不应用于 |
| --- | --- | --- | --- |
| `TrayWorkspaceConfig` | Base frame | 普通 `plan/move_to_base_pose()` 和未来视觉最终 TCP | Homing、startup、return、`LIFT`、`TRANSIT`、raw debug |
| `OffsetWorkspaceConfig` | arm-local / Slide-relative | solver 侧别、Slide 保持/候选、正负偏置切换判断 | Tray 门限、startup pose、Base clearance |
| `RobotMotionEnvelopeConfig` | Base/轴逻辑混合但字段显式标注 | startup/return 与 side-switch 中间阶段 | 普通最终任务许可、完整碰撞检测、物理安全认证 |

正常目标调用链为：

```text
Base final TCP → TrayWorkspace → Base solver → OffsetWorkspace
               → transition planner → RobotMotionEnvelope policy → execution
```

中间 `LIFT`/`TRANSIT` 可以高于 Tray 最终任务 Z 上限，但仍必须通过轴/关节软限位与完整 FK。
startup/return 使用 `RobotMotionEnvelopeConfig.startup_pose` 的专用路径，不获得普通目标门限的隐式例外。
