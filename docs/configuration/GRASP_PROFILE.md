# GraspProfile 配置

状态：schema v2/loader implemented、offline-tested；已验证 profile 位于
`host/config/robot_runtime.json` 的 `grasp_profile` 区块。

字段：

| 字段 | 含义 | 约束 |
| --- | --- | --- |
| `contact_offset_mm` | object Base Z 到 contact 的偏移 | 有限，可为负 |
| `suction_settle_time_s` | 开启吸盘后、抬升前的稳定等待 | 有限、非负；缺省为 2 秒 |
| `yaw_mode` | `fixed` / `keep_current` / `from_observation` | 枚举值 |
| `fixed_yaw_deg` | 固定 Base tool yaw | 只在 `fixed` 时必填 |
| `minimum_confidence` | 最低检测置信度 | `[0,1]` |
| `maximum_observation_age_s` | observation 与 Host 当前时间允许的最大绝对偏差 | 有限正数；接受 `[-阈值, +阈值]` 内的时钟偏差 |

`from_observation` 在 orientation 缺失时明确拒绝，不回退为 0°；非零 roll/pitch 也不会被静默丢弃。

抓取的 overhead/lift 与扫描、arm-local 工作区进入过渡共用绝对 Base Z=200 mm。contact 为视觉目标经手眼标定和目标补偿后的 Base Z，再叠加 `contact_offset_mm`，且必须严格低于 200 mm。

项目运行固定使用 Git 跟踪的 `host/config/robot_runtime.json`；修改 `grasp_profile` 区块前必须
独立确认并保持 `validated=true`。loader 对旧 schema、缺失、placeholder、NaN/Inf 或未验证
配置一律 fail-closed，仓库不再提供 example 配置。
