# Host 测试目录

`tests/` 下的自动发现测试默认全部离线运行，不连接真实硬件，也不得访问串口或控制器局域网（Controller Area Network, CAN）。需要真实设备、烧录、Homing、轴运动或吸盘动作的验证不得放入自动 discovery。

## 目录分类

- `config/`：配置模型、加载器和安全包络。
- `geometry/`：刚体变换和坐标链。
- `kinematics/`：运动学、工作区和规划器。
- `calibration/`：标定算法、状态模型和脚本适配。
- `protocol/`：CAN、舵机及 STM32 协议和驱动。
- `motion/`：轴抽象、统一控制器和吸盘控制。
- `application/`：应用控制器和培养槽工作区。
- `vision/`：视觉目标解析。
- `cli/`：维护与调试命令行入口。
- `hardware_adapter/`：设备发现与硬件适配。
- `integration/`：离线启动和演示编排。
- `helpers/`：共享测试辅助代码；运动 CLI helper 位于 `tests/helpers/motion_cli_test_support.py`。

## 命令格式

在 `host/` 目录执行全量离线 discovery：

```bash
.venv/bin/python -m unittest discover -s tests -q
```

按模块指定测试：

```bash
.venv/bin/python -m unittest \
  tests.kinematics.test_base_frame_solver \
  tests.motion.test_unified_controller \
  -q
```

本次目录整理按明确指令未重新运行任何测试；最近一次整理前已知结果为 524 项 Host 测试通过。
