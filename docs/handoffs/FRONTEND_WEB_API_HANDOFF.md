# 蘑菇采摘平台前端 Web API 交接说明

> 交接快照：2026-08-12
> API 标题：`Mushroom Robot Service API`
> API 版本：`1.0.0`
> 当前根仓库基线：`main` / `26dbc53`，但相关 Web/API 文件存在未提交修改；本交接包以打包时的工作树为准。

## 1. 先看结论

- 浏览器与机器人服务之间使用 HTTP JSON；默认 API 地址为 `http://127.0.0.1:8000`。
- 当前后端公开 21 个路由，现有页面接入其中 10 个。
- 后端是同步薄适配层。运动类 `POST` 会一直等待底层操作结束后才返回，没有 WebSocket、后台任务队列、operation handle 或进度流。
- 浏览器取消请求、刷新页面或断网，不会自动停止已经提交的机器人动作。停止动作必须显式调用 `POST /api/stop`。
- 当前没有登录、Token、Cookie 或其他鉴权；默认只监听本机，禁止直接暴露公网。
- 前端必须同时使用 `state`、`mode` 和 `capabilities` 控制按钮，不能只看 HTTP 是否在线。
- 所有坐标和位置字段都使用 `mm` 或 `deg`，不要在前端自行做运动学、坐标变换或软限位推导。

本文中的 API（Application Programming Interface，应用程序编程接口）、HTTP（Hypertext
Transfer Protocol，超文本传输协议）、JSON（JavaScript Object Notation，JavaScript
对象表示法）和 TCP（Tool Center Point，工具中心点）均按上述含义使用。

## 2. 交接包内容

| 文件 | 用途 |
| --- | --- |
| `docs/handoffs/FRONTEND_WEB_API_HANDOFF.md` | 本说明，前端首读 |
| `docs/handoffs/frontend-web-api.types.ts` | TypeScript 类型参考 |
| `docs/handoffs/frontend.env.example` | Vite 环境变量示例 |
| `docs/interfaces/ROBOT_WEB_API.md` | 后端原始接口说明与安全边界 |
| `web/` | 当前完整前端源码、依赖锁文件与测试；压缩包不含 `node_modules`、`dist` |
| `host/application/web_api.py` | 路由、请求模型、错误映射的代码事实来源 |
| `host/application/robot_service.py` | mode、state、capabilities 与操作语义 |
| `host/application/runtime_state.py` | mode 与 state 枚举 |

## 3. 通信架构与边界

```text
Web frontend
    ↓ HTTP + JSON
FastAPI adapter
    ↓ synchronous service calls
MushroomRobotService
    ↓
规划、状态机、运动控制、视觉与硬件适配层
```

前端职责：

- 展示状态、能力、规划结果和错误；
- 根据状态与能力启停控件；
- 发送明确的用户动作；
- 为长操作显示 pending，并保留独立 STOP；
- 对网络离线、业务拒绝和硬件故障给出不同提示。

前端不得承担：

- 正运动学（Forward Kinematics, FK）或逆运动学（Inverse Kinematics, IK）；
- Camera、Tool、Base 坐标系转换；
- 机械限位、托盘工作区或碰撞路径判定；
- 根据页面状态推断机器人已经物理到位或已经吸住蘑菇。

## 4. 本地开发环境

### 4.1 前端

当前项目使用原生 JavaScript、Vite 7 和 Node 内置测试：

- Node.js：Vite 锁定版本要求 `^20.19.0 || >=22.12.0`；
- npm：使用仓库内 `package-lock.json`；
- 默认开发地址：`http://127.0.0.1:5173`；
- API 地址由 `VITE_API_BASE_URL` 配置，未配置时回退到 `http://127.0.0.1:8000`。

```bash
cd /Users/sd/Projects/mushroom-picking-platform/web
cp ../docs/handoffs/frontend.env.example .env.local
npm ci
npm run dev
```

构建与测试：

```bash
cd /Users/sd/Projects/mushroom-picking-platform/web
npm test
npm run build
```

### 4.2 后端

后端依赖 FastAPI（Python Web 框架）和 Uvicorn（Asynchronous Server Gateway Interface
Server，ASGI 服务器）：

```bash
cd /Users/sd/Projects/mushroom-picking-platform/host
.venv/bin/pip install -r requirements.txt
.venv/bin/python scripts/robot_web_api.py \
  --mode read-only \
  --host 127.0.0.1 \
  --port 8000
```

可访问：

- 健康检查：`http://127.0.0.1:8000/api/health`
- Swagger 用户界面（User Interface, UI）：`http://127.0.0.1:8000/docs`
- OpenAPI：`http://127.0.0.1:8000/openapi.json`

用于前端联调的离线规划模式：

```bash
cd /Users/sd/Projects/mushroom-picking-platform/host
.venv/bin/python scripts/robot_web_api.py \
  --mode dry-run \
  --host 127.0.0.1 \
  --port 8000

curl -X POST http://127.0.0.1:8000/api/startup
```

`dry-run` 不发送硬件命令，但当前只能验证状态、Base 规划、Base execute 的不执行分支以及部分视觉流程。它不能完整模拟真实轴反馈和所有 execute-only 操作。

真实执行模式仅允许设备负责人在安全隔离完成后启动：

```bash
cd /Users/sd/Projects/mushroom-picking-platform/host
.venv/bin/python scripts/robot_web_api.py \
  --mode execute \
  --confirm-motion \
  --confirm-rotation-no-stop \
  --host 127.0.0.1 \
  --port 8000
```

前端开发人员不要自行启动 `execute` 模式，也不要把服务器绑定到公网地址。

## 5. 传输协议

### 5.1 基本约定

| 项目 | 约定 |
| --- | --- |
| 协议 | HTTP，当前仅 `GET`、`POST` |
| 数据格式 | JSON；请求体使用 `Content-Type: application/json` |
| API Base URL | 默认 `http://127.0.0.1:8000` |
| 鉴权 | 当前无 |
| Cookie/凭据 | CORS 配置为 `allow_credentials=false` |
| 成功状态 | 当前统一为 HTTP 200 |
| 空返回值 | 序列化为 `{ "ok": true }` |
| 错误格式 | `{ "error": { "type", "message", "rejection_reason?" } }` |
| 调用模型 | 同步阻塞；实际完成后才返回 200 |
| 自动重试 | 写请求禁止自动重试 |

### 5.2 数值、单位与字段校验

- `slide`、`z`：位置 `mm`，速度 `mm/s`，加速度 `mm/s^2`；
- `shoulder`、`elbow`、`rotation`：位置 `deg`，速度 `deg/s`，加速度 `deg/s^2`；
- 所有数值必须是有限数，不能发送 `NaN`、`Infinity` 或字符串数字；
- `velocity`、`acceleration`、`timeout_s` 若提供，必须大于 0；
- 请求模型禁止额外字段；字段拼写错误会返回 400；
- `yaw_deg` 可省略或传 `null`，含义是沿用当前工具 yaw；现有页面始终发送数值。

### 5.3 CORS 与网络安全

CORS（Cross-Origin Resource Sharing，跨源资源共享）默认允许：

- `http://localhost:3000`
- `http://127.0.0.1:3000`
- `http://localhost:5173`
- `http://127.0.0.1:5173`

如前端使用其他端口，后端启动时重复传入：

```bash
.venv/bin/python scripts/robot_web_api.py \
  --mode dry-run \
  --cors-origin http://127.0.0.1:4173 \
  --cors-origin http://localhost:4173
```

后端拒绝通配源 `*`，只允许 `Content-Type` 请求头。当前没有鉴权，因此不得直接监听公网。

## 6. Service 模式与状态

### 6.1 模式 `mode`

| 值 | 含义 | 前端行为 |
| --- | --- | --- |
| `read-only` | 默认安全模式，不构造真实硬件 runtime | 只展示状态/能力；大部分动作会被拒绝 |
| `dry-run` | 使用离线规划后端，不提交硬件命令 | 可做 Base 规划；Base execute 返回 `executed=false` |
| `execute` | 真实硬件模式，启动时需要两项显式授权 | 才能 Jog、吸盘、关节使能和真实运动 |

### 6.2 状态 `state`

| 值 | 含义 | UI 建议 |
| --- | --- | --- |
| `created` | Service 已创建，尚未 startup | 只开放 Startup、状态查询、STOP |
| `starting` | 正在初始化 | 锁定普通写操作，保留 STOP |
| `ready` | 可接受符合 mode/capabilities 的新操作 | 按能力开放控件 |
| `observing` | 正在获取视觉目标 | 显示忙，保留 STOP |
| `planning` | 正在规划 | 显示忙，保留 STOP |
| `executing` | 正在执行操作 | 显示忙，保留 STOP |
| `disabled` | 旋转关节 holding 已移除 | 禁止运动；可在 execute 模式请求 enable |
| `fault` | Service 或执行链故障 | 显示故障详情；禁止普通写操作，保留 STOP/Shutdown |
| `shutdown` | Service 已关闭 | 可重新 Startup |

`backend_status` 只保证在空闲 `ready` 或 `disabled` 时尝试读取；活动状态和 `fault` 下通常为 `null`。前端不要依赖其内部结构作为稳定接口。

## 7. 路由总表

“页面已接入”指当前 `web/src/api.js` 和页面逻辑已经调用该路由，不代表当前页面门禁已完全正确。

| 方法 | 路径 | 主要前置条件 | 主要响应 | 页面已接入 |
| --- | --- | --- | --- | --- |
| GET | `/api/health` | 无 | `{ok:true}` | 否 |
| GET | `/api/status` | 无 | `RobotStatus` | 是，1 秒轮询 |
| GET | `/api/capabilities` | 无 | `RobotCapabilities` | 否，页面改用 status 内嵌值 |
| POST | `/api/startup` | `created/shutdown` | `{ok:true}` | 是 |
| POST | `/api/shutdown` | 任意；进程内幂等 | `{ok:true}` | 否 |
| POST | `/api/stop` | 可与阻塞操作并发 | `{ok:true}` | 是 |
| GET | `/api/axes` | 能力可用 | `{axes: AxisDescriptor[]}` | 否，当前页面轴名写死 |
| GET | `/api/axes/{axis}` | runtime 可查询 | `AxisState` | 是 |
| POST | `/api/axes/{axis}/move-absolute` | `execute/ready` 且能力可用 | `MotionCommandResult` | 否 |
| POST | `/api/axes/{axis}/move-relative` | `execute/ready` 且能力可用 | `MotionCommandResult` | 是，Jog |
| POST | `/api/motion/base/plan` | 非 read-only、`ready` | `BasePlanResponse` | 是 |
| POST | `/api/motion/base/execute` | 非 read-only、`ready` | `MotionResult` | 是 |
| GET | `/api/motion/base/current` | `ready` 且五轴真实状态有效 | `CurrentTcpPose` | 是 |
| POST | `/api/motion/return-to-startup` | `execute/ready` | 当前 execute backend 返回布尔值 | 是 |
| POST | `/api/joints/enable` | `execute`，`disabled/ready` | backend 相关 JSON | 否 |
| POST | `/api/joints/disable` | `execute/ready` | backend 相关 JSON | 否 |
| POST | `/api/suction` | `execute/ready` | backend 相关 JSON | 是，页面仅使用 `grip/idle` |
| POST | `/api/vision/observe` | 非 read-only、`ready` | `VisionTargetObservation` | 否 |
| POST | `/api/vision/plan` | 非 read-only、`ready`，视觉与外参门禁通过 | `VisionPlanResponse` | 否 |
| POST | `/api/pick` | 非 read-only、`ready`，能力与抓取配置通过 | `PickResult` | 否 |
| POST | `/api/scan-pick` | 非 read-only、`ready`，扫描/抓取配置通过 | `ScanAndPickResult` | 否 |

合法 `{axis}`：`slide`、`z`、`shoulder`、`elbow`、`rotation`。

## 8. 请求契约

### 8.1 Base 目标规划或执行

```json
{
  "x_mm": 300.0,
  "y_mm": 400.0,
  "z_mm": 120.0,
  "yaw_deg": 0.0
}
```

必填：`x_mm`、`y_mm`、`z_mm`。`yaw_deg` 可省略或为 `null`。

### 8.2 单轴绝对运动

```json
{
  "position": -20.0,
  "velocity": 4.0,
  "acceleration": 8.0,
  "timeout_s": 3.0
}
```

只有 `position` 必填。

### 8.3 单轴相对运动

```json
{
  "delta": -10.0,
  "velocity": null,
  "acceleration": null,
  "timeout_s": null
}
```

只有 `delta` 必填。当前页面只发送 `{ "delta": number }`。

### 8.4 吸盘

```json
{
  "action": "grip"
}
```

`action` 只允许：

- `grip`：启动吸附；
- `release`：执行释放动作；
- `idle`：关闭吸附输出并回到 idle。

STOP 成功路径还会尝试把吸附输出切换为 `idle`。命令成功不代表存在真空反馈或已经物理抓取成功。

## 9. 关键响应契约

完整类型见 `frontend-web-api.types.ts`。

### 9.1 状态

```json
{
  "state": "ready",
  "mode": "dry-run",
  "capabilities": {
    "base_frame_motion": true,
    "tray_workspace_gate": true,
    "offset_planning": true,
    "robot_motion_envelope": true,
    "joint_holding": true,
    "suction_command": true,
    "vision_gateway": "fake available",
    "vision_target_observation": true,
    "hand_eye_calibration": "validated",
    "vision_target_resolution": true,
    "pick_planning": true,
    "pick_execution": false,
    "physical_pick_verification": false,
    "axis_listing": true,
    "axis_state_query": true,
    "axis_absolute_motion": false,
    "axis_relative_motion": false
  },
  "backend_status": null,
  "fault": null
}
```

`capabilities` 是动态值，可能随着 startup、mode 和配置改变。建议每次以最新 `/api/status` 中的值为准。

### 9.2 单轴状态

```json
{
  "axis": "slide",
  "connected": true,
  "enabled": null,
  "busy": false,
  "homed": true,
  "position_valid": true,
  "current_position": 12.5,
  "position_unit": "mm",
  "faulted": false,
  "fault_code": null,
  "fault_message": null
}
```

字段可能为 `null`，不得用 JavaScript truthy/falsy 简化三态值。只有 `connected === true`、`position_valid === true`、`faulted === false` 且 `current_position` 为有限数时，才适合显示为可信真实位置。

### 9.3 当前 TCP

```json
{
  "x_mm": 250.0,
  "y_mm": 200.0,
  "z_mm": 200.0,
  "yaw_deg": 0.0,
  "frame_id": "base"
}
```

该接口读取真实五轴状态并由后端做 FK。前端不得用单轴位置自行计算 TCP。

### 9.4 Base 执行

`dry-run` 示例：

```json
{
  "executed": false,
  "plan": {},
  "message": "Dry-run plan complete; no motion command was submitted."
}
```

`execute` 成功时 `executed=true`。前端必须根据 `executed` 显示“仅规划”或“真实执行完成”，不能只看 HTTP 200。

### 9.5 规划结果

当前存在两种结构：

- `dry-run`：返回 `BaseMovePlan` 对象，阶段位于 `plan.stages`，阶段名字段为 `kind`；
- `execute`：当前 backend 返回 `DemoStage[]` 数组，阶段名字段为 `name`。

前端兼容读取方式：

```js
const stages = Array.isArray(plan) ? plan : (plan?.stages ?? []);
const stageName = (stage) => stage.kind ?? stage.name ?? "stage";
```

这是当前后端响应形态不统一，不应视为长期稳定设计。生产 UI 上线前建议后端统一为一个显式响应模型。

## 10. 错误协议

统一格式：

```json
{
  "error": {
    "type": "RobotServiceStateError",
    "message": "axis move requires READY, got executing"
  }
}
```

规划拒绝可能增加：

```json
{
  "error": {
    "type": "TargetOutsideTrayWorkspace",
    "message": "target is outside tray",
    "rejection_reason": "outside_tray_workspace"
  }
}
```

| HTTP | 含义 | 前端处理 |
| --- | --- | --- |
| 400 | 请求字段、数值或轴名非法 | 表单错误，不重试 |
| 409 | mode/state/busy/能力门禁冲突 | 刷新 status，提示当前状态，不自动重试 |
| 422 | 工作区、不可达或视觉/规划拒绝 | 展示 `rejection_reason`，允许用户修改目标 |
| 503 | Service、硬件通信或运行故障不可用 | 显示设备/服务故障，不自动重放写请求 |
| 500 | 未预期服务错误 | 显示通用错误并保留诊断时间点 |
| 网络错误 | 未收到 HTTP 响应 | 状态未知；先重新查询 status，不得假定命令未执行 |

500 和 503 不回传本地路径、串口名或堆栈。前端只展示后端 `error.message`，不要展示原始 HTML 或整段响应文本。

## 11. 前端门禁要求

建议实现统一判断，不在各按钮中分散复制：

```js
function derivePermissions(status, pending) {
  const state = status?.state;
  const mode = status?.mode;
  const cap = status?.capabilities ?? {};
  const idle = state === "ready" && !pending.anyWrite;

  return {
    startup: ["created", "shutdown"].includes(state) && !pending.anyWrite,
    planBase: idle && mode !== "read-only" && cap.base_frame_motion === true,
    executeBase: idle && mode !== "read-only" && cap.base_frame_motion === true,
    jog: idle && mode === "execute" && cap.axis_relative_motion === true,
    suction: idle && mode === "execute" && cap.suction_command === true,
    returnToStartup: idle && mode === "execute",
    observe: idle && mode !== "read-only" && cap.vision_target_observation === true,
    visionPlan: idle && mode !== "read-only" && cap.vision_target_resolution === true,
    pick: idle && mode !== "read-only" && cap.pick_planning === true,
    scanPick: idle && mode !== "read-only" && cap.pick_planning === true,
    stop: !pending.stop,
  };
}
```

注意：`POST /api/motion/base/execute` 在 `dry-run` 中允许调用，但只返回规划结果并令 `executed=false`；按钮文案应显示为“模拟执行/检查”，避免误导。

## 12. 轮询、并发与取消

当前页面策略：

- `/api/status`：每 1 秒轮询；
- 选中轴：只在 `ready/disabled` 且没有写请求时刷新，最短间隔 2 秒；
- Current TCP：进入 `ready` 或 Jog、Return、Base Execute 完成后刷新；
- 同一时间只允许一个普通写请求；
- STOP 不受其他普通 pending 限制，可并行发送。

前端实现要求：

1. 不要重叠发送 status 轮询；上一次未完成时复用或跳过。
2. 不要对任何运动、吸盘、关节或抓取 `POST` 做自动重试。
3. 请求超时、页面刷新或 `AbortController.abort()` 只表示前端停止等待，不表示机器人停止。
4. 网络状态不确定时，先重新读取 `/api/status` 和轴状态；需要停止时由操作者明确发送 `/api/stop`。
5. STOP 返回 200 后仍应刷新 `/api/status`，以最终 `state/fault` 为准。
6. 页面卸载时不要偷偷发送 shutdown 或 stop；这属于操作者动作和安全策略，不是普通前端清理逻辑。

## 13. 当前页面已知契约差异

以下是本次按当前工作树复核得到的事实，前端重写时应处理：

1. 当前 `getControlAvailability()` 只按 state/mode 判断 Jog，没有检查 `capabilities.axis_relative_motion`。因此 `dry-run/ready` 下 Jog 可能显示可用，但接口会返回 409。
2. 当前页面会在 `dry-run/ready` 尝试读取 Current TCP；离线轴状态故意标记为 `connected=false`，当前接口会返回 409 `Current five-axis positions are unavailable or invalid.`。页面应把它显示为“离线模式无真实 TCP”，而不是机器人故障。
3. 当前 Base 规划响应在 `dry-run` 和 `execute` 两种 backend 下结构不同；现有 `renderPlan()` 只兼容 `plan.stages`，不能完整展示 execute 模式的数组结果。
4. 当前页面把轴列表硬编码为 5 个轴，没有调用 `/api/axes`；如后端轴能力变化，页面不会自动更新。
5. 当前 OpenAPI 能列出路由和 4 个请求模型，但大部分成功响应 schema 是空对象 `{}`；运行时请求校验错误又被统一映射成 400，而生成的 OpenAPI 仍显示 FastAPI 默认 422。前端类型以本交接文件和实际代码为准。
6. 当前 `api.js` 没有 fetch 超时和 operation id。新增超时只能结束 UI 等待，不能被描述为取消机器人动作。

## 14. 联调验收清单

- [ ] 前端 API 地址来自环境变量，未硬编码部署机器地址。
- [ ] `npm test` 与 `npm run build` 通过。
- [ ] 后端离线测试至少覆盖 `test_robot_web_api.py`。
- [ ] 页面离线时能区分网络错误和 HTTP 业务错误。
- [ ] 所有写按钮在 pending 时禁用，STOP 保持独立可用。
- [ ] 按 `state + mode + capabilities` 三者共同控制按钮。
- [ ] `dry-run` 的 Base execute 明确显示 `executed=false`。
- [ ] 规划组件兼容对象和数组两种当前响应，或等待后端统一契约后再实现。
- [ ] 400/409/422/503/500 都有可读提示，422 展示 `rejection_reason`。
- [ ] 不自动重试任何写请求。
- [ ] 页面刷新、关闭或请求超时不显示“机器人已停止”。
- [ ] 没有把 HTTP 200、吸盘命令确认或离线测试描述成真实硬件采摘成功。
- [ ] execute 联调由设备负责人现场授权，并准备独立硬件急停。

## 15. 代码事实来源

发生冲突时，按以下顺序判断：

1. `host/application/web_api.py`：路由、请求校验、CORS、错误映射；
2. `host/application/robot_service.py`：mode/state/capabilities 和操作语义；
3. `web/src/api.js`：当前页面实际请求；
4. `web/src/control-state.js`、`web/src/main.js`：当前页面门禁与轮询；
5. `docs/interfaces/ROBOT_WEB_API.md`：说明性文档。

本交接包不包含机器专属 `host/config/local/` 配置、硬件凭据或真实设备参数。
