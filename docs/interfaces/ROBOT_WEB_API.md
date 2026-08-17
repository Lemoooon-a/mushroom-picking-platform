# Robot Web API

## 1. 边界与安全原则

Robot Web API 是单个 `MushroomRobotService` 实例的 FastAPI 薄适配层：

```text
Web frontend → HTTP JSON → FastAPI adapter → MushroomRobotService
```

适配层不直接访问运动控制器、硬件驱动、运动学、规划器或视觉网关，也不保存第二份机器人
状态。工作区、关节限位、规划、执行、并发和 FAULT 状态仍完全由现有 Service 链路负责。

启动 Web 服务不会自动调用 `startup()`，不会归零，也不会运动。服务默认使用 `read-only`
模式并仅绑定 `127.0.0.1`。

> 当前没有登录鉴权。`execute` 模式只能运行在受控机器和受控网络中，不得直接暴露到公网。

## 2. 安装与启动

```bash
cd /Users/sd/Projects/mushroom-picking-platform/host
.venv/bin/pip install -r requirements.txt

.venv/bin/python scripts/robot_web_api.py \
  --mode read-only \
  --host 127.0.0.1 \
  --port 8000
```

启动后可访问：

- OpenAPI 文档：`http://127.0.0.1:8000/docs`
- Service 状态：`http://127.0.0.1:8000/api/status`

`dry-run` 启动后仍需显式调用 `POST /api/startup`，才会初始化纯离线规划后端：

```bash
.venv/bin/python scripts/robot_web_api.py --mode dry-run
```

`execute` 继续复用 CLI 的显式运动授权参数：

```bash
.venv/bin/python scripts/robot_web_api.py \
  --mode execute \
  --confirm-motion \
  --host 127.0.0.1
```

即使提供授权参数，启动服务器本身也不会自动 startup、home 或 move。操作者仍需显式调用
生命周期和运动接口，并承担真实机器人周边安全隔离责任。

## 3. 路由

| 方法 | 路径 | Service 调用 | 说明 |
| --- | --- | --- | --- |
| GET | `/api/health` | 无 | 仅表示 Web 进程存活 |
| GET | `/api/status` | `status()` | 返回 Service 状态；仅空闲 runtime 附带实时 backend 状态 |
| GET | `/api/capabilities` | `capabilities` | 返回当前能力门禁 |
| POST | `/api/startup` | `startup()` | 显式启动 Service |
| POST | `/api/shutdown` | `shutdown()` | 每个进程最多调用 Service 一次 |
| POST | `/api/stop` | `stop()` | 可在阻塞运动请求期间并发进入；停止运动并将吸附输出切换到 `idle`（气泵关闭） |
| GET | `/api/axes` | `list_axes()` | 返回公开轴描述符 |
| GET | `/api/axes/{axis}` | `get_axis_state()` | 查询单轴状态 |
| POST | `/api/axes/{axis}/move-absolute` | `move_axis_absolute()` | raw/manual 绝对运动 |
| POST | `/api/axes/{axis}/move-relative` | `move_axis_relative()` | raw/manual 相对运动 |
| POST | `/api/motion/base/plan` | `plan_base_target()` | 只规划，不运动 |
| POST | `/api/motion/base/execute` | `move_base_target()` | 通过既有完整 Base 规划/执行链 |
| GET | `/api/motion/base/current` | `get_current_tcp_pose()` | 读取五轴位置并通过现有 FK 返回 Base-frame TCP |
| POST | `/api/motion/return-to-startup` | `return_to_startup()` | 返回已配置的 startup-safe pose；仅 execute/READY |
| POST | `/api/joints/enable` | `enable_joints()` | 保留既有 holding 状态复核 |
| POST | `/api/joints/disable` | `disable_joints()` | 保留既有状态转换 |
| POST | `/api/suction` | `suction()` | `grip`、`release` 或 `idle` |
| POST | `/api/vision/observe` | `request_observation()` | 无请求体 |
| POST | `/api/vision/plan` | `request_observation()` → `resolve_camera_point()` → `plan_base_target()` | 新拍一帧，按 capture 快照转换到 Base 后只规划；不执行运动 |
| POST | `/api/pick` | `pick()` | 无请求体，使用已加载 GraspProfile |
| POST | `/api/scan-positions/{scan_index}/move` | `move_to_scan_position()` | 移动到已校验的第 `1..8` 个扫描位；无请求体 |
| POST | `/api/scan-positions/{scan_index}/pick-one` | `pick_one_at_scan_position()` | 确保位于指定扫描位，识别并抓取一颗，放置后返回；无请求体 |
| POST | `/api/scan-pick` | `scan_and_pick()` | 无请求体；同步完成固定 8 区域扫描、区域内重复抓取与固定位置放置 |

`web/` 前端通过 `GET /api/status` 每秒轮询进程级状态。Service 仅在 `READY` 或
`DISABLED` 读取实时 backend 状态；活动状态和 `FAULT` 的 `backend_status` 为 `null`。
前端也只在 `READY/DISABLED` 且没有写请求时轮询选中轴，运动期间保留最后显示值并在请求
结束后刷新。Current TCP 由后端现有正运动学计算，页面初次进入 `READY` 以及 Jog、Return、
Base Execute 完成后更新；前端不复制运动学。Startup 仅在 `CREATED/SHUTDOWN` 启用，
Return 仅在 execute/READY 启用。当前没有 WebSocket、后台任务队列、operation handle、
鉴权或数据库。

`/api/scan-pick` 的成功响应包含 `result`、`total_picked` 和
`visited_scan_positions`。每个区域记录 `scan_index`、`detected_count`、`picked_count` 与
`final_reason`。运行时固定从 Git 跟踪的 `host/config/robot_runtime.json` 的 `scan_pick` 区块
读取参数；区块缺失或未确认时，整个 Service 在构造硬件 Runtime 前 fail-closed。

八个扫描位统一使用 Base Z=150 mm；该高度也用于视觉抓取的 overhead/lift 和正负偏置
工作区换向，不再由 scan-pick JSON 单独配置。

`scan_index` 固定为 `1..8`，顺序与 `ScanPickProfile.scan_poses` 一致：`1..4` 是第一个 X
位置下依次排列的四个 Y，`5..8` 是第二个 X 位置下依次排列的四个 Y。真实坐标只保存在后端
已校验配置中，前端不得复制坐标。非法编号返回 400，扫描配置缺失或未校验返回 409。

`/api/scan-positions/{scan_index}/move` 返回现有 `MotionResult`；dry-run 只规划并返回
`executed=false`。`/api/scan-positions/{scan_index}/pick-one` 返回现有
`ScanAndPickResult`，且每次都会先确保到达指定扫描位。成功抓取、放置并返回时
`final_reason="picked_and_placed_unverified"`；无目标时返回 HTTP 200、`total_picked=0`、
`final_reason="no_target"`；目标规划被拒绝时返回 HTTP 200，`final_reason` 为
`target_rejected:<错误类型>`。运动、吸盘或通信故障仍返回错误响应并由 Service 进入 FAULT。
由于当前没有真空反馈，`picked_count=1` 只表示动作与吸盘命令完成，不表示物理抓取已经验证。
普通目标放置点为 Base `(150, 1000, 150, 0)`，过大目标放置点为
Base `(450, 1000, 150, 0)`；视觉 `size_class` 决定放置分流。两个放置点是仅有的 Tray
区外放置例外。抓取回撤后直接到达所选点并释放，然后直接返回当前扫描位，不包含放置前后
回撤。

## 4. 请求示例

单轴绝对运动：

```json
{
  "position": -20.0,
  "velocity": null,
  "acceleration": null,
  "timeout_s": null
}
```

单轴相对运动：

```json
{
  "delta": -10.0,
  "velocity": null,
  "acceleration": null,
  "timeout_s": null
}
```

Base 目标规划或执行：

```json
{
  "x_mm": 300.0,
  "y_mm": 400.0,
  "z_mm": 120.0,
  "yaw_deg": 0.0
}
```

吸盘控制：

```json
{
  "action": "grip"
}
```

合法轴名为 `slide`、`z`、`shoulder`、`elbow` 和 `rotation`。数值字段必须是有限值；速度、
加速度和超时若提供则必须大于零；额外请求字段会被拒绝。

> raw/manual 单轴运动不经过 Base-frame 工作区、逆运动学（Inverse Kinematics, IK）、偏置
> 工作区、跨区 clearance 或碰撞路径规划，只适用于维护、标定和受控的小范围调试。Web 层
> 不增加第二份单轴限位，实际门禁和软限位仍由 Service 与统一运动层执行。

## 5. HTTP 错误

错误统一返回：

```json
{
  "error": {
    "type": "RobotServiceStateError",
    "message": "axis move requires READY, got executing"
  }
}
```

| HTTP 状态码 | 语义 |
| --- | --- |
| 400 | 请求字段、数值或轴名非法 |
| 409 | mode/state/busy 或能力门禁不允许当前操作 |
| 422 | 工作区拒绝、不可达或规划/视觉目标解析失败 |
| 503 | Service、硬件通信或运行故障暂时不可用 |
| 500 | 未预期的适配器错误；浏览器只收到通用消息 |

503 和 500 响应不会回传异常堆栈、串口名或本地路径。同步 Service 方法由 FastAPI 的普通
同步路由在线程池中执行；请求只有在 Service 调用实际返回后才成功，不会伪造后台完成。

## 6. CORS 与网络暴露

默认只允许以下本地开发源：

- `http://localhost:3000`
- `http://127.0.0.1:3000`
- `http://localhost:5173`
- `http://127.0.0.1:5173`

使用可重复参数覆盖默认列表：

```bash
.venv/bin/python scripts/robot_web_api.py \
  --cors-origin http://localhost:4173 \
  --cors-origin http://127.0.0.1:4173
```

不接受通配源 `*`。如显式绑定 `0.0.0.0`，必须先确认网络隔离；在增加鉴权前不得暴露公网。

抓取相关的手眼标定、GraspProfile 或真实视觉 producer 不满足门禁时，`/api/pick` 和
`plan_observation()` 仍按现有 Service 语义 fail-closed。仅 `/api/vision/plan` 可把明确标记为
provisional 的 `tool_T_camera` 用于打印、人工检查和 dry-run Base 规划；该路由没有运动执行
调用。

## 7. 本机真实视觉只规划联调

`host/config/robot_runtime.json` 的 `vision_runtime` 区块配置 Vision Gateway Protocol v1 服务。
当前配置使用 `172.20.10.2:9000` 和 `camera_color_optical_frame`。启动时必须显式选择 socket
gateway；Web API 启动后还要初始化 dry-run 离线规划后端：

```bash
cd /Users/sd/Projects/mushroom-picking-platform/host

.venv/bin/python scripts/robot_web_api.py \
  --mode dry-run \
  --vision-gateway socket \
  --host 127.0.0.1 \
  --port 8000

curl -X POST http://127.0.0.1:8000/api/startup
curl -X POST http://127.0.0.1:8000/api/vision/observe
curl -X POST http://127.0.0.1:8000/api/vision/plan
```

`/api/vision/observe` 的观察对象和 `/api/vision/plan` 的 `camera` 区块都会返回
`size_class=normal|oversized`。`/api/vision/plan` 会重新拍摄一帧，并返回 `request_id`、
Camera 点、capture 五轴快照，以及
Camera 下的 `target_compensation_camera_mm`，以及 Base 下的 `raw_position_mm`、
`target_compensation_base_mm` 和最终 `position_mm`。最终点用于
现有 Base planner；响应还包含 `tool_T_camera` 的 provisional/validated 状态、完整计划和最终
五轴解。成功路径只调用规划接口；不会调用 `move_base_target()`、`execute_base_plan()`、
`/api/pick` 或任何吸盘接口。422 规划错误额外返回现有 `rejection_reason`（例如
`outside_tray_workspace`、`outside_offset_workspace` 或 `planar_unreachable`）。
