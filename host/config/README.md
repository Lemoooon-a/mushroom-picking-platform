# Host 配置职责

`host/config/` 同时保存纯类型/loader、单台项目机器的正式策略参数、模板和被 Git 忽略的机器配置。
文件是否位于本目录，不代表它会在 `import config` 时自动加载。包级导出保持惰性；导入纯配置模块
不会打开串口、CAN（Controller Area Network，控制器局域网）、读取 local 文件或创建 runtime。

## 文件角色

| 文件 | 类型 | Tracked | 机器专属 | 运行时默认加载 | 角色 |
| --- | --- | ---: | ---: | ---: | --- |
| `__init__.py` | 惰性导出 | 是 | 否 | 否 | 暴露稳定配置类型/项目常量，不触发 local loader |
| `hardware.py` | typed model + loader | 是 | 否 | 是 | 硬件设备发现与端口/CAN 配置模型 |
| `hardware_local.example.py` | template | 是 | 否 | 否 | `hardware_local.py` 的占位模板 |
| `hardware_local.py` | local Python | 否，ignored | 是 | 是 | 当前机器端口、VID/PID、CAN 等硬件选择 |
| `motion_runtime.py` | typed model + loader | 是 | 否 | 是 | 到位、timeout、速度/加速度和线性轴范围模型 |
| `motion_local.example.py` | template | 是 | 否 | 否 | `motion_local.py` 的占位模板 |
| `motion_local.py` | local Python | 否，ignored | 是 | 是 | 当前机器的 motion runtime 参数 |
| `frame_transforms.py` | typed JSON loader | 是 | 否 | 按 Base/视觉入口加载 | Base/Slide-zero 与 Tool/Camera 外参文档模型 |
| `frame_transforms.example.json` | template | 是 | 否 | 否 | 未验证的 frame transform 模板 |
| `frame_transforms.local.json` | local JSON | 否，ignored | 是 | 按入口默认路径加载 | 当前机器 Base 与手眼外参及独立 validation metadata |
| `five_axis_geometry.example.json` | template | 是 | 否 | 否 | 五轴几何占位模板 |
| `five_axis_geometry.local.json` | local JSON | 否，ignored | 是 | 五轴 Base 规划时加载 | 当前机构连杆、轴方向和 Tool 固定几何 |
| `tray_workspace.py` | typed JSON loader | 是 | 否 | 按应用入口加载 | Base-frame 普通最终任务 TCP 门限模型 |
| `tray_workspace.example.json` | template | 是 | 否 | 否 | 不含可执行边界的模板，不是通用默认值 |
| `tray_workspace.local.json` | local JSON | 否，ignored | 是 | Demo/application 默认路径 | 用户确认的 Base-frame 绝对最终任务工作区 |
| `joints.py` | 项目正式参数 | 是 | 当前单台项目机器人 | 是 | Shoulder/Elbow ID、零点、方向、36:1 与软限位 |
| `feetech.py` | 项目正式参数 | 是 | 当前单台项目机器人 | 是 | Rotation 型号、协议、零点、方向与软限位 |
| `workspace_planning.py` | 项目规划策略 | 是 | 当前机器人策略 | 是 | arm-local 正负偏置区、Slide 候选、fallback 与数值容差 |
| `robot_motion_envelope.py` | 软件安全阶段策略 | 是 | 当前机器人策略 | 是 | startup/return pose 与跨区绝对 Base Z clearance |

## 配置来源规则

- `*.local.*` 是 machine-specific（机器专属）配置，必须保持 ignored，不得强制暂存或复制到文档。
- `*.example.*` 只用于说明 schema 和必填字段，绝不自动视为 validated runtime configuration。
- `joints.py` / `feetech.py` 是当前单台项目机器的正式参数，不是设备系列的通用出厂默认值。
- `workspace_planning.py` 只描述 arm-local 逆运动学（Inverse Kinematics, IK）规划约束。
- `robot_motion_envelope.py` 只描述已知 startup 与中间安全阶段策略，不是碰撞模型或安全认证。
- `tray_workspace.local.json` 只描述 Base frame 中普通最终任务 TCP 的绝对允许范围，不是机械极限。

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
