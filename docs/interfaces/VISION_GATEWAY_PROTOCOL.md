# Vision Gateway Protocol v1

状态：implemented、offline-tested；真实视觉 producer 仍为 hardware-blocked/unavailable。

`VisionGateway` 只传输并验证消息，不读取轴状态、不解析 Camera→Base、不规划或执行运动。协议使用 TCP、一行一个 UTF-8 JSON、一次连接完成一次 request/response，不自动重连。

## 消息

请求固定为 `protocol_version=1`、`type=capture_request`，并包含 `request_id`、`camera_frame`、`timestamp`。响应只能是：

- `target_detection`：同一 `request_id`、`frame_id`、正深度 `position_mm`，可选 `orientation`、`confidence`、`target_id`；
- `no_target`：同一 `request_id` 和非空 `reason`；
- `error`：同一 `request_id`、`code`、`message`。

`orientation=null` 表示视觉未提供方向；上层不得据此伪造 yaw。实现严格拒绝未知/缺失字段、非有限数、非正深度、错误版本、request mismatch、非 UTF-8、malformed JSON、超长消息、timeout 和断线。

## 安全边界

拍照前必须在机器人已停止且全部轴到位时创建 `CaptureSnapshot`。响应到达后再次读取状态，只有五轴位置未变化且仍为 `STATIONARY` 才能构造 `VisionTargetObservation`。视觉目标不得直接进入执行器层。
