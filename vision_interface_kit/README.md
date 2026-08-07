# Vision TCP Interface Kit

本目录是交付给视觉组的最小接口包。视觉程序只需要实现一个 TCP Server，并按照本文约定接收一次拍照请求、返回一次识别结果。

## 1. 连接方式

- 默认监听地址由视觉程序自行决定，示例端口为 `9000`。
- Host 每次识别都会新建一个 TCP 连接。
- 一个连接只完成一次请求和一次响应，响应完成后关闭该连接并继续监听后续连接。
- 请求和响应都是一行 UTF-8 JSON，末尾必须包含换行字节 `\n`。
- 每次响应只能返回一个目标。

## 2. 请求

Host 发送 `capture_request`，完整示例见 `examples/capture_request.json`。

| 字段 | 说明 |
| --- | --- |
| `protocol_version` | 固定为 `1` |
| `type` | 固定为 `capture_request` |
| `request_id` | 本次请求的唯一标识；响应必须原样返回 |
| `camera_frame` | 固定使用 `camera_color_optical_frame` |
| `timestamp` | Host 发送请求时的 Unix 时间，单位为秒 |

## 3. 响应

Vision 必须返回以下三种响应之一：

- `target_detection`：检测到一个目标，示例见 `examples/target_detection.json`；
- `no_target`：未检测到目标，示例见 `examples/no_target.json`；
- `error`：相机、推理或其他视觉处理失败，示例见 `examples/error.json`。

所有响应的 `request_id` 必须与请求完全一致。字段不能缺失，也不能增加协议之外的字段。`target_detection` 中的 `timestamp`、`target_id`、`confidence` 和 `orientation` 即使没有值也必须保留，并使用 JSON `null`。

### 目标位置

`position_mm` 是目标在 `camera_color_optical_frame` 中的三维坐标：

- 单位统一为毫米（mm）；
- X 向右；
- Y 向下；
- Z 向前；
- Z 必须大于 `0`。

当前允许 `orientation` 为 `null`。如果提供方向，格式必须是单位四元数，分量顺序为 `x`、`y`、`z`、`w`。

`confidence` 可以是 `null`，或者是 `[0.0, 1.0]` 范围内的数值。`timestamp` 应为视觉图像实际采集时的 Unix 时间秒；如果 Vision 与 Host 不在同一台机器，两端时钟需要同步。

## 4. 使用验收客户端

先启动视觉组实现的 TCP Server，然后在本目录的上一级执行：

```bash
python3 vision_interface_kit/mock_robot_client.py
```

默认连接 `127.0.0.1:9000`。指定其他地址、端口或超时时间：

```bash
python3 vision_interface_kit/mock_robot_client.py \
  --host 192.168.1.20 \
  --port 9000 \
  --timeout 2.0
```

客户端会发送合法的 `capture_request`，读取一行响应，并检查：

- 响应是合法的 UTF-8 JSON；
- `protocol_version` 为 `1`；
- 消息类型是 `target_detection`、`no_target` 或 `error`；
- `request_id` 与请求一致；
- 顶层字段集合和目标位置等基本数据符合协议。

验收通过时程序退出码为 `0` 并打印 `PASS`；连接或协议检查失败时退出码为 `1`，错误原因输出到标准错误流。
