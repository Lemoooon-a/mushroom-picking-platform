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
- `host/`：上位机控制程序；`host/config/` 分为模型/加载器、`project/`、`examples/` 和团队共享的 `local/` 运行配置
- `host/tests/`：按领域分组的 Host 离线测试与共享 helper
- `firmware/`：STM32 固件
- `tools/`：电机测试、标定和调试工具
- `config/`：仓库级机器人配置模板

Web 前端可通过 [Robot Web API](docs/interfaces/ROBOT_WEB_API.md) 调用统一的
`MushroomRobotService`，服务默认使用只读模式并仅监听本机地址。

## 当前阶段

当前优先完成：

1. CAN 关节电机驱动；
2. 总线舵机驱动；
3. 步进控制器通信；
4. 五关节统一抽象；
5. SCARA 正逆运动学；
6. 基本采摘状态机。
