# Mushroom Picking Platform

基于滑轨 SCARA 机械臂的蘑菇采摘平台。

## 系统组成

- 滑轨步进电机
- Z 轴步进电机
- 两个瓴控 MG4010E-i36 CAN 关节电机
- 末端总线舵机
- 吸盘及真空检测
- Intel RealSense 深度相机
- 上位机控制程序
- STM32 步进与 IO 控制器

## 目录结构

- `docs/`：系统设计、电气连接、协议和标定文档
- `host/`：上位机控制程序；当前机械臂的正式配置统一位于 `host/config/`
- `host/tests/`：按领域分组的 Host 离线测试与共享 helper
- `firmware/`：STM32 固件
- `tools/`：电机测试、标定和调试工具
- `config/`：仓库级配置

Web 前端可通过 [Robot Web API](docs/interfaces/ROBOT_WEB_API.md) 调用统一的
`MushroomRobotService`。Service 模式默认是 `read-only`；Web API 默认绑定当前控制端的
`172.20.10.3:8000`，也可通过下面的参数显式覆盖。

## 网络配置与 Service 启动

机器人控制端同时连接两个不同的网络端点：

```text
Web 前端 ──HTTP──> 控制端 Robot Web API 172.20.10.3:8000
控制端   ──TCP───> 视觉端识别服务      172.20.10.10:9000
```

### 视觉端 TCP 地址

视觉 TCP（Transmission Control Protocol，传输控制协议）服务的地址位于
[`host/config/robot_runtime.json`](host/config/robot_runtime.json) 的
`vision_runtime` 区块：

```json
"vision_runtime": {
  "host": "172.20.10.10",
  "port": 9000
}
```

- `host`：运行视觉识别服务的设备 IP；
- `port`：视觉程序监听的 TCP 端口；
- 修改后需要重启 Robot Service；视觉端程序必须监听相同端口。

### 控制端 Web API 地址

控制端自身监听的 IP 和端口通过 `robot_web_api.py` 的启动参数设置：

```bash
cd /Users/sd/Projects/mushroom-picking-platform/host

.venv/bin/python scripts/robot_web_api.py \
  --mode execute \
  --confirm-motion \
  --vision-gateway socket \
  --host 172.20.10.3 \
  --port 8000
```

- `--host 172.20.10.3`：控制端计算机用于对外提供 Web API 的本机 IP；
- `--port 8000`：Robot Web API 的监听端口；
- `--vision-gateway socket`：使用上述 `vision_runtime.host/port` 连接视觉端；
- `--host/--port` 不会修改视觉端地址，两组地址不能混用。

前端连接控制端的地址在 `web/.env` 中设置：

```env
VITE_API_BASE_URL=http://172.20.10.3:8000
```

如果前端来源地址不在后端默认的跨源资源共享（Cross-Origin Resource Sharing, CORS）白名单中，
启动后端时还需追加，例如：

```bash
--cors-origin http://172.20.10.2:5173
```

## 当前阶段

当前优先完成：

1. CAN 关节电机驱动；
2. 总线舵机驱动；
3. 步进控制器通信；
4. 五关节统一抽象；
5. SCARA 正逆运动学；
6. 基本采摘状态机。
