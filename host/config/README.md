# Host 配置职责

`host/config/` 同时保存纯类型/loader、单台项目机器的正式策略参数、模板和被 Git 忽略的机器配置。
文件是否位于本目录，不代表它会在 `import config` 时自动加载。包级导出保持惰性；导入纯配置模块
不会打开串口、CAN（Controller Area Network，控制器局域网）、读取 local 文件或创建 runtime。

## 文件角色

| 文件 | 类型 | Tracked | 机器专属 | 运行时默认加载 | 角色 |
| --- | --- | ---: | ---: | ---: | --- |
| `__init__.py` | 惰性导出 | 是 | 否 | 否 | 暴露稳定配置类型/项目常量，不触发 local loader |
| `hardware.py` | typed model + loader | 是 | 否 | 是 | 硬件设备发现与端口/CAN 配置模型 |
| `examples/hardware.py` | template | 是 | 否 | 否 | `local/hardware.py` 的占位模板 |
| `local/hardware.py` | local Python | 否，ignored | 是 | 是 | 当前机器端口、VID/PID、CAN 等硬件选择 |
| `motion_runtime.py` | typed model + loader | 是 | 否 | 是 | 到位、timeout、速度/加速度和线性轴范围模型 |
| `examples/motion.py` | template | 是 | 否 | 否 | `local/motion.py` 的占位模板 |
| `local/motion.py` | local Python | 否，ignored | 是 | 是 | 当前机器的 motion runtime 参数 |
| `frame_transforms.py` | typed JSON loader | 是 | 否 | 按 Base/视觉入口加载 | Base/Slide-zero 与 Tool/Camera 外参文档模型 |
| `examples/frame_transforms.json` | template | 是 | 否 | 否 | 未验证的 frame transform 模板 |
| `local/frame_transforms.json` | local JSON | 否，ignored | 是 | 按入口默认路径加载 | 当前机器 Base 与手眼外参及独立 validation metadata |
| `examples/five_axis_geometry.json` | template | 是 | 否 | 否 | 五轴几何占位模板 |
| `local/five_axis_geometry.json` | local JSON | 否，ignored | 是 | 五轴 Base 规划时加载 | 当前机构连杆、轴方向和 Tool 固定几何 |
| `tray_workspace.py` | typed JSON loader | 是 | 否 | 按应用入口加载 | Base-frame 普通最终任务 TCP 门限模型 |
| `examples/tray_workspace.json` | template | 是 | 否 | 否 | 不含可执行边界的模板，不是通用默认值 |
| `local/tray_workspace.json` | local JSON | 否，ignored | 是 | Demo/application 默认路径 | 用户确认的 Base-frame 绝对最终任务工作区 |
| `project/joints.py` | 项目正式参数 | 是 | 当前单台项目机器人 | 是 | Shoulder/Elbow ID、零点、方向、36:1 与软限位 |
| `project/feetech.py` | 项目正式参数 | 是 | 当前单台项目机器人 | 是 | Rotation 型号、协议、零点、方向与软限位 |
| `project/workspace_planning.py` | 项目规划策略 | 是 | 当前机器人策略 | 是 | arm-local 正负偏置区、Slide 候选、fallback 与数值容差 |
| `project/robot_motion_envelope.py` | 软件安全阶段策略 | 是 | 当前机器人策略 | 是 | startup/return pose 与跨区绝对 Base Z clearance |
| `project/vision_runtime.py` | typed model + loader | 是 | 否 | 按 Robot Service 加载 | socket、frame、timeout、消息上限与质量门限；默认未验证 |
| `examples/vision_runtime.json` | template | 是 | 否 | 否 | 真实 socket 配置占位模板 |
| `local/vision_runtime.json` | local JSON | 否，ignored | 是 | 可选 | 真实视觉 producer 地址与验证状态 |
| `project/grasp_strategy.py` | validated loader | 是 | 否 | 按 Robot Service 加载 | 只接受完整且 `validated=true` 的 GraspProfile |
| `examples/grasp_profile.json` | template | 是 | 否 | 否 | 所有真实 offset 保持 `null` |
| `local/grasp_profile.json` | local JSON | 否，ignored | 是 | 可选 | 经确认的真实抓取策略 |
| `project/scan_pick.py` | validated loader | 是 | 否 | 按 Robot Service 加载 | 校验固定 2×4 扫描点、固定放置位和单区域抓取上限 |
| `examples/scan_pick.json` | template | 是 | 否 | 否 | 扫描、放置坐标保持 `null`，yaw 固定为 0 |
| `local/scan_pick.json` | local JSON | 否，ignored | 是 | 可选 | 经确认的真实扫描与放置策略 |

## 配置来源规则

- `local/` 除 `__init__.py` 外均为 machine-specific（机器专属）配置，必须保持 ignored。
- `examples/` 只用于说明 schema 和必填字段，绝不自动视为 validated runtime configuration。
- `project/joints.py` / `project/feetech.py` 是当前单台项目机器的正式参数，不是设备系列默认值。
- `project/workspace_planning.py` 只描述 arm-local 逆运动学（Inverse Kinematics, IK）规划约束。
- `project/robot_motion_envelope.py` 只描述 startup 与中间安全阶段策略，不是碰撞模型或安全认证。
- `local/tray_workspace.json` 只描述 Base frame 中普通最终任务 TCP 的绝对允许范围，不是机械极限。
- `host/calibration/` 只保存标定算法、状态模型和 capture/solve 逻辑；机器专属标定结果统一保存在 ignored `host/config/local/`。
- `vision_runtime`、`grasp_profile` 和 `scan_pick` 的 tracked example 都默认 fail-closed；不得把 example 复制后的 placeholder 当作 validated 数据。

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
