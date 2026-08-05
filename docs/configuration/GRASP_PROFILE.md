# GraspProfile 配置

状态：schema/loader implemented、offline-tested；本机真实 profile unavailable。

字段：

| 字段 | 含义 | 约束 |
| --- | --- | --- |
| `approach_offset_mm` | object Base Z 到 pre-grasp 的偏移 | 有限、非负、不低于 contact |
| `contact_offset_mm` | object Base Z 到 contact 的偏移 | 有限，可为负 |
| `retreat_offset_mm` | object Base Z 到 retreat 的偏移 | 有限、非负、不低于 contact |
| `yaw_mode` | `fixed` / `keep_current` / `from_observation` | 枚举值 |
| `fixed_yaw_deg` | 固定 Base tool yaw | 只在 `fixed` 时必填 |
| `minimum_confidence` | 最低检测置信度 | `[0,1]` |
| `maximum_observation_age_s` | observation 最大年龄 | 有限正数 |

`from_observation` 在 orientation 缺失时明确拒绝，不回退为 0°；非零 roll/pitch 也不会被静默丢弃。

tracked 模板 `host/config/examples/grasp_profile.json` 保持 `validated=false` 且真实 offset 为 `null`。操作者应复制到 ignored 的 `host/config/local/grasp_profile.json`，只填写经独立确认的值并显式设置 `validated=true`。loader 对缺失、placeholder、NaN/Inf 或未验证配置一律 fail-closed。
