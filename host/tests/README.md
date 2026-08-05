# Host 测试目录

`tests/` 下的自动发现测试默认全部离线运行，不连接真实硬件，也不得访问串口或控制器局域网（Controller Area Network, CAN）。需要真实设备、烧录、Homing、轴运动或吸盘动作的验证不得放入自动 discovery。

## 目录分类

- `suites/config/`：配置模型、加载器和安全包络。
- `suites/geometry/`：刚体变换和坐标链。
- `suites/kinematics/`：运动学、工作区和规划器。
- `suites/calibration/`：标定算法、状态模型和脚本适配。
- `suites/protocol/`：CAN、舵机及 STM32 协议和驱动。
- `suites/motion/`：轴抽象、统一控制器和吸盘控制。
- `suites/application/`：应用控制器、GraspProfile、PickPlanner、Workflow 和 Robot Service 状态/故障语义。
- `suites/vision/`：视觉协议、Fake/Socket gateway、拍照快照和目标解析。
- `suites/cli/`：维护与调试命令行入口。
- `suites/hardware_adapter/`：设备发现与硬件适配。
- `suites/integration/`：离线启动和演示编排。
- `helpers/`：共享测试辅助代码；运动 CLI helper 位于 `tests/helpers/motion_cli_test_support.py`。

领域目录统一放在 `suites/` 包下，避免 `unittest discover -s tests` 将
`tests/config/`、`tests/kinematics/` 等同名测试包置于生产包之前。

## 命令格式

在 `host/` 目录执行全量离线 discovery：

```bash
.venv/bin/python -m unittest discover -s tests -q
```

按模块指定测试：

```bash
.venv/bin/python -m unittest \
  tests.suites.kinematics.test_base_frame_solver \
  tests.suites.motion.test_unified_controller \
  -q
```

2026-08-06 Robot Service 集成后结果为 553 项 Host 离线测试通过；automatic discovery 不包含真实硬件测试。
