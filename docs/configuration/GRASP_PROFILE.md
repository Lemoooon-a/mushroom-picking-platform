# GraspProfile 配置

状态：schema/loader implemented、offline-tested；团队共享的已验证 profile 位于 `host/config/local/grasp_profile.json`。

字段：

| 字段 | 含义 | 约束 |
| --- | --- | --- |
| `approach_offset_mm` | object Base Z 到 pre-grasp 的偏移 | 有限、非负、不低于 contact |
| `contact_offset_mm` | object Base Z 到 contact 的偏移 | 有限，可为负 |
| `retreat_offset_mm` | object Base Z 到 retreat 的偏移 | 与 `retreat_z_mm` 二选一；有限、非负、不低于 contact |
| `retreat_z_mm` | retreat 的 Base 绝对 Z | 与 `retreat_offset_mm` 二选一；有限且不得低于 contact Base Z |
| `minimum_transit_z_mm` | pre-grasp/retreat 的最低 Base Z 门限 | 可选、有限；两个高位目标都必须严格高于此值，contact 豁免 |
| `yaw_mode` | `fixed` / `keep_current` / `from_observation` | 枚举值 |
| `fixed_yaw_deg` | 固定 Base tool yaw | 只在 `fixed` 时必填 |
| `minimum_confidence` | 最低检测置信度 | `[0,1]` |
| `maximum_observation_age_s` | observation 最大年龄 | 有限正数 |

`from_observation` 在 orientation 缺失时明确拒绝，不回退为 0°；非零 roll/pitch 也不会被静默丢弃。

模板 `host/config/examples/grasp_profile.json` 保持 `validated=false` 且真实 offset 为 `null`。项目运行默认使用 tracked 的 `host/config/local/grasp_profile.json`；修改其中参数前必须独立确认并保持 `validated=true`。loader 对缺失、placeholder、NaN/Inf 或未验证配置一律 fail-closed。
