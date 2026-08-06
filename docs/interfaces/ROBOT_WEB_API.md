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

`execute` 继续复用 CLI 的两项显式授权参数：

```bash
.venv/bin/python scripts/robot_web_api.py \
  --mode execute \
  --confirm-motion \
  --confirm-rotation-no-stop \
  --host 127.0.0.1
```

即使提供授权参数，启动服务器本身也不会自动 startup、home 或 move。操作者仍需显式调用
生命周期和运动接口，并承担真实机器人周边安全隔离责任。

## 3. 路由

| 方法 | 路径 | Service 调用 | 说明 |
| --- | --- | --- | --- |
| GET | `/api/health` | 无 | 仅表示 Web 进程存活 |
| GET | `/api/status` | `status()` | 返回真实 Service 状态 |
| GET | `/api/capabilities` | `capabilities` | 返回当前能力门禁 |
| POST | `/api/startup` | `startup()` | 显式启动 Service |
| POST | `/api/shutdown` | `shutdown()` | 每个进程最多调用 Service 一次 |
| POST | `/api/stop` | `stop()` | 可在阻塞运动请求期间并发进入 |
| GET | `/api/axes` | `list_axes()` | 返回公开轴描述符 |
| GET | `/api/axes/{axis}` | `get_axis_state()` | 查询单轴状态 |
| POST | `/api/axes/{axis}/move-absolute` | `move_axis_absolute()` | raw/manual 绝对运动 |
| POST | `/api/axes/{axis}/move-relative` | `move_axis_relative()` | raw/manual 相对运动 |
| POST | `/api/motion/base/plan` | `plan_base_target()` | 只规划，不运动 |
| POST | `/api/motion/base/execute` | `move_base_target()` | 通过既有完整 Base 规划/执行链 |
| POST | `/api/joints/enable` | `enable_joints()` | 保留既有 holding 状态复核 |
| POST | `/api/joints/disable` | `disable_joints()` | 保留既有状态转换 |
| POST | `/api/suction` | `suction()` | `grip`、`release` 或 `idle` |
| POST | `/api/vision/observe` | `request_observation()` | 无请求体 |
| POST | `/api/pick` | `pick()` | 无请求体，使用已加载 GraspProfile |

第一版前端通过 `GET /api/status` 轮询状态。当前没有 WebSocket、后台任务队列、operation
handle、鉴权、数据库或前端页面。

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

手眼标定、GraspProfile 或真实视觉 producer 不满足门禁时，观察、视觉规划和抓取仍按现有
Service 语义 fail-closed，Web API 不提供绕过入口。
