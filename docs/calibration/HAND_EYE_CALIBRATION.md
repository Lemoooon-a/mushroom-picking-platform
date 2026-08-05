# 手眼标定数据与验证门禁

## 1. 当前状态

当前 capability 为 `unavailable`。本机 `host/config/local/frame_transforms.json` 中：

```text
tool_T_camera = null
```

仓库没有手眼标定求解实现，也没有已验证的外参记录。本文件只冻结输入、输出、数据格式和验证
状态；不声称已经完成标定。

## 2. 坐标系与输出方向

采用 Eye-in-Hand：

- `T`：Tool/TCP；
- `C`：Camera；
- 输出 `tool_T_camera` 把 Camera 中的点或位姿转换到 Tool；
- 反向量为 `camera_T_tool = inverse(tool_T_camera)`。

方向必须通过标定方法和独立验证确认，不能只根据变量名猜测。

## 3. 所需输入

未来标定至少需要：

- 已确认的 Camera 光学 frame 定义；
- 相机内参和畸变参数；
- 多个静止采集姿态的五轴状态与 `base_T_tool(q_capture)`；
- 每幅图像中同一标定目标的可靠 pose；
- 标定板/目标几何与单位；
- 采集时间对应关系；
- 独立验证姿态与误差阈值；
- 方法、软件版本、操作者和原始数据来源。

如果视觉只输出像素，还必须提供深度或其他可观测三维约束及反投影流程。

## 4. 数据模型

运行时最小记录为：

```python
@dataclass(frozen=True)
class HandEyeCalibration:
    tool_T_camera: RigidTransform
    validated: bool
    source: str
    method: str
    created_at: str | None = None
```

状态严格区分：

- `missing`：没有 `tool_T_camera`；
- `provisional`：有变换但 `validated=false` 或没有独立 validation 证据；
- `validated`：存在明确 `tool_camera_validated=true` 记录。

现有配置兼容字段：

```json
{
  "tool_T_camera": {"translation_mm": [0, 0, 0], "rotation_rpy_deg": [0, 0, 0]},
  "metadata": {
    "tool_camera_source": "...",
    "tool_camera_method": "...",
    "tool_camera_set_at": "...",
    "tool_camera_validated": false
  }
}
```

上例仅说明 schema；不得把全零或 identity 当作真实外参。`metadata.validated` 是
Base–Slide-zero 状态，不能替代 `tool_camera_validated`。

## 5. Provisional 录入

现有 `scripts/set_tool_camera_transform.py` 只用于预览或人工录入候选固定变换。写入任何新值时会
明确设置：

```text
tool_camera_validated = false
```

它不会求解手眼关系，也不会自动升级为 validated。人工测量、CAD 值或临时装配测量即使可写入，
仍只能作为 provisional，不能解锁真实视觉运动。

## 6. 后续 validation 计划

后续独立任务应：

1. 确认相机 frame、内参与标定目标；
2. 在多组静止、全部轴到位的姿态采集同步数据；
3. 用明确方法求解 `tool_T_camera`；
4. 在未参与求解的姿态验证位置和旋转误差；
5. 保存原始数据、方法、版本、阈值和验证结果；
6. 经人工复核后才显式写 `tool_camera_validated=true`；
7. 再单独进行小范围、低速、可急停的视觉到 Base 只读/运动验证。

当前不实现上述标定，也不修改本机外参。
