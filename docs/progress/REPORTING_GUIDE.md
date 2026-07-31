# Mushroom Picking Platform — Project Progress Reporting Guide

## 1. Purpose

This document defines how Codex must inspect, evaluate, and report the current
state of the complete mushroom-picking platform repository.

The project is not limited to STM32 firmware. A complete progress report must
consider all relevant subsystems, including:

- STM32 motion and end-effector firmware;
- Slide and Z-axis motion;
- TMC5160 configuration, diagnostics, and homing;
- UART logging and machine-control protocol;
- vacuum pump and release-valve control;
- MG4010E CAN communication and motor control;
- shoulder and elbow joint abstraction and calibration;
- forward and inverse kinematics;
- multi-joint coordination;
- host-side task control;
- coordinate transforms;
- complete mushroom-picking task integration.

The report is intended for:

- continuing work in another GPT or Codex conversation;
- handing the project to another developer;
- determining what has actually been implemented;
- separating software completion from hardware validation;
- identifying the smallest safe next milestone.

All conclusions must be based on repository evidence.

---

## 2. Canonical Output

Unless the user explicitly requests a chat-only summary, update:

`docs/progress/CURRENT_STATUS.md`

This file represents the current project-wide status.

Do not repeatedly append contradictory status entries. Replace obsolete
statements and clearly mark superseded designs.

Do not modify business source code while performing a status-only task unless
the user explicitly requests implementation changes.

Optional historical snapshots may be stored under:

`docs/progress/history/`

The current report remains the primary source for ongoing work.

---

## 3. Repository Topology

Before reporting progress, determine the actual repository structure.

The project may contain:

- a root Git repository;
- nested Git repositories;
- Git submodules;
- untracked subsystem directories;
- generated files;
- external libraries;
- calibration records;
- build artifacts.

Inspect each Git repository separately.

For every repository found, report:

- repository path;
- current branch;
- upstream tracking branch;
- clean or dirty worktree;
- staged changes;
- unstaged changes;
- untracked files;
- recent relevant commits;
- whether another repository tracks it as a submodule, subtree, or ordinary
  directory.

Do not describe the whole project as clean merely because one nested repository
is clean.

Pay particular attention to:

`firmware/stm32_motion_controller`

and determine whether it is:

- an independent nested repository;
- a Git submodule;
- a subtree;
- or an accidentally nested repository.

---

## 4. Required Inspection

Before updating the project report, inspect at least the following.

### 4.1 Project-level sources

- root `AGENTS.md`;
- nested `AGENTS.md` files;
- root `README.md`;
- project architecture documents;
- planning documents;
- calibration documents;
- current progress documents;
- root Git state;
- recent root commits;
- uncommitted root changes.

### 4.2 STM32 firmware

Inspect where present:

- `.ioc`;
- `Core/`;
- `Drivers/`;
- `Middlewares/`;
- `Libraries/`;
- `App/`;
- CMake, Makefile, STM32CubeIDE, linker, and build configuration;
- generated GPIO, DMA, UART, SPI, and timer initialization;
- UART logging and command handling;
- STEP/DIR motion;
- TMC5160 SPI configuration;
- Slide and Z homing;
- fault polling and DIAG handling;
- vacuum pump and release-valve control;
- firmware build output;
- hardware-validation logs.

### 4.3 MG4010E and host control

Inspect where present:

- CAN transport;
- protocol codecs;
- motor driver;
- joint abstraction;
- command-line tools;
- configuration files;
- timeout and retry behavior;
- stop behavior;
- position interpretation;
- soft limits;
- unit tests;
- hardware-validation notes.

### 4.4 Kinematics and calibration

Inspect:

- shoulder and elbow zero configuration;
- direction conventions;
- reduction ratios;
- joint limits;
- measured absolute angles;
- calibration records;
- Planar 2R forward kinematics;
- inverse kinematics;
- singularity and reachability handling;
- actual link-length configuration;
- joint-limit filtering;
- integration with the motor command layer.

### 4.5 System integration

Inspect:

- `host/motion/`;
- `host/tasks/`;
- common actuator abstractions;
- multi-joint coordination;
- arrival detection;
- motion timeout;
- fault propagation;
- unified stop behavior;
- coordinate transformations;
- end-effector rotation;
- vacuum confirmation;
- complete harvesting state machine.

Do not treat an empty directory or design document as an implemented feature.

---

## 5. Evidence Priority

Use evidence in the following order:

1. current source code;
2. current configuration files;
3. successful and reproducible build/test output;
4. hardware logs, measurements, photos, or explicit test records;
5. current documentation;
6. commit messages;
7. historical plans.

A README or commit message is not sufficient proof when current code disagrees.

A successful build proves compilation and linking only. It does not prove:

- correct GPIO output;
- correct PWM waveform;
- correct motor direction;
- successful homing;
- mechanical repeatability;
- reliable CAN communication;
- successful vacuum pickup;
- complete system integration.

---

## 6. Status Definitions

Use the following standard statuses.

| Status | Meaning |
| --- | --- |
| Planned | Design or task description exists, but implementation has not started |
| Implemented | Relevant source code exists |
| Compiles | Included in a successful build |
| Offline tested | Unit, mock, simulation, or mathematical tests pass |
| Bench tested | Electrical signals, protocol, driver, or actuator tested on hardware |
| Mechanically tested | Tested with the actual axis, arm, gripper, or mechanism |
| Integrated | Works together with adjacent subsystems |
| System validated | Demonstrated in the complete mushroom-picking workflow |
| Not verified | Evidence is insufficient |
| Blocked | Missing hardware, data, configuration, or decision prevents progress |

A subsystem may have multiple statuses simultaneously. For example:

```text
Implemented: yes
Compiles: yes
Bench tested: yes
Mechanically tested: no
Integrated: no
```

Do not compress these into a misleading single “completed” label.

---

## 7. Required Report Structure

### 7.1 推荐的报告层级

```text
蘑菇采摘平台
├── STM32 底层执行器
│   ├── Slide
│   ├── Z
│   ├── TMC5160
│   ├── 串口协议
│   └── 吸盘
│
├── MG4010E 关节控制
│   ├── CAN 总线
│   ├── 协议层
│   ├── 电机驱动层
│   └── 关节抽象
│
├── 机器人模型与算法
│   ├── 肩肘标定
│   ├── 2R 正逆运动学
│   ├── 坐标系
│   └── 解筛选
│
├── 系统协调层
│   ├── 多关节控制
│   ├── 到位与超时
│   ├── 统一停止
│   └── 故障传播
│
└── 整机任务层
    ├── 视觉目标
    ├── 接近与下探
    ├── 吸附确认
    ├── 搬运
    └── 释放与恢复
```

### 7.2 固定章节顺序

Use the following sections in this order.

# 蘑菇采摘平台当前进度

## 1. 技术结论

Summarize:

- the current overall project stage;
- the most mature subsystem;
- the largest integration gap;
- the next major milestone;
- whether the project is still at component level, subsystem integration level,
  or complete-task validation level.

This section should describe the complete platform, not only STM32.

## 2. 总体进度矩阵

Include a project-wide table covering at least:

- STM32 Slide axis;
- STM32 Z axis;
- STM32 machine protocol;
- vacuum pump and release valve;
- TMC5160 diagnostics and homing;
- MG4010E CAN transport and protocol;
- MG4010E single-joint control;
- shoulder and elbow calibration;
- kinematics;
- multi-joint coordination;
- coordinate transformation;
- complete harvesting task.

For each item include:

- current status;
- implemented capability;
- main gap;
- verification level.

## 3. 仓库与版本状态

Report every Git repository separately.

Include:

- repository path;
- branch;
- upstream;
- commits;
- dirty files;
- untracked files;
- nested repository relationship;
- reproducibility and traceability risks.

Explicitly identify code that has not been committed.

## 4. 系统架构

Describe the current dependency chain, for example:

```text
STM32 actuator firmware
        ↑ serial protocol
Host motion coordination
        ↑
MG4010E joint layer
        ↑
Kinematics and task planning
        ↑
Harvest task state machine
```

Also describe the repository directories and their responsibilities.

Identify:

- reusable libraries;
- platform-specific code;
- generated code;
- host-side code;
- algorithm-only code;
- unfinished integration layers.

## 5. STM32 固件进度

Cover only the STM32 subsystem in this section.

Recommended subsections:

### 5.1 Slide and Z Motion

### 5.2 TMC5160 Configuration and Diagnostics

### 5.3 Homing and Position Validity

### 5.4 UART Logging and Machine Protocol

### 5.5 Vacuum Pump and Release Valve

### 5.6 Fault, Limit, and Emergency Handling

For each subsection report:

- files;
- APIs;
- configuration;
- startup behavior;
- build status;
- hardware-test status;
- known limitations.

## 6. MG4010E 关节控制进度

Cover:

- CAN transport;
- protocol;
- driver;
- joint abstraction;
- position source;
- zero and direction conventions;
- soft limits;
- command tools;
- test coverage;
- real-hardware evidence;
- missing arrival, timeout, and coordination behavior.

## 7. 标定与运动学进度

Cover:

- shoulder and elbow calibration values;
- source and quality of calibration evidence;
- forward kinematics;
- inverse kinematics;
- singularity handling;
- reachability;
- actual link parameters;
- joint-limit filtering;
- connection to actuator commands.

Clearly distinguish mathematical correctness from robot execution.

## 8. 系统协调与采摘任务

Report the status of:

- common actuator interface;
- Slide, shoulder, elbow, Z, rotation, and vacuum coordination;
- point-to-point joint coordination;
- arrival detection;
- timeouts;
- unified stop;
- fault propagation;
- coordinate transforms;
- camera-to-robot mapping;
- harvesting task state machine;
- vacuum confirmation;
- placement and release.

If only architecture documents or placeholder directories exist, mark them as
planned or pending.

## 9. 验证结果

Report separately:

### 9.1 Firmware Builds

### 9.2 Host Unit Tests

### 9.3 Mathematical Tests

### 9.4 Electrical Bench Tests

### 9.5 Mechanical Tests

### 9.6 Integrated System Tests

For each command record:

- working directory;
- exact command;
- exit status;
- relevant result;
- whether hardware was involved.

## 10. 资源与性能

Where applicable report:

- firmware FLASH and RAM;
- UART buffer sizes;
- task or ISR timing risks;
- CAN timeout and retry behavior;
- test duration;
- communication rates;
- known performance bottlenecks.

Do not require every subsystem to have the same resource metrics.

## 11. 安全默认行为

Verify from current source code:

- whether STM32 moves at startup;
- whether axes are enabled at startup;
- whether homing runs automatically;
- whether the vacuum pump starts;
- whether the release valve opens;
- whether MG4010E commands are sent automatically;
- whether host tools default to dry-run or real hardware;
- what happens on communication failure;
- what happens on emergency stop;
- whether position remains valid after an abrupt stop.

List all boot-test and real-motion enabling macros and their defaults.

## 12. 已知问题和风险

Separate:

### Confirmed issues

Problems supported by evidence.

### Risks

Conditions that may become problems but have not yet been reproduced.

Include where applicable:

- uncommitted code;
- nested Git repository ambiguity;
- temporary soft limits;
- missing full-travel calibration;
- sensorless-homing uncertainty;
- lack of vacuum feedback;
- lack of arrival detection;
- lack of motion timeout;
- missing multi-joint coordination;
- missing coordinate transforms;
- missing complete-task recovery;
- documentation/code mismatch.

## 13. 开放决策

List values or architecture choices requiring user input or hardware tests.

Examples:

- final Slide and Z travel;
- final homing parameters;
- final vacuum PWM and timing;
- vacuum sensor selection;
- emergency release behavior;
- shoulder and elbow link lengths;
- joint coordination strategy;
- inverse-kinematics branch selection;
- stop and fault-propagation policy;
- root/nested Git repository strategy.

Do not make these decisions without evidence.

## 14. 下一阶段建议

Use priority levels such as:

```text
P0 — reproducible repository baseline
P1 — subsystem hardware validation
P2 — joint and actuator coordination
P3 — complete harvesting workflow
```

Each next step must include:

- goal;
- affected subsystem;
- expected files;
- required hardware or evidence;
- acceptance criteria;
- safety precautions.

Prefer small, independently verifiable milestones.

## 15. 交接信息

End with a compact handoff section containing:

- current overall stage;
- current active task;
- files to read first;
- commands to run first;
- confirmed hardware parameters;
- values that must not be guessed;
- uncommitted changes;
- next intended milestone.

This section should be sufficient to initialize a new GPT or Codex session.

---

## 8. Reporting Rules

- Write the report in Chinese.
- Preserve source identifiers, APIs, paths, register names, message names, and
  protocol names in English.
- Use Chinese and English technical-term correspondence naturally where useful.
- Do not insert unnecessary translations into code identifiers or paths.
- Use tables for subsystem comparisons and verification matrices.
- Do not copy large source-code blocks.
- Do not invent hardware-test evidence.
- Do not infer implementation from directory names.
- Do not infer hardware success from compilation.
- Do not silently modify source code during a report-only task.
- Clearly separate current implementation from planned implementation.
- Clearly identify uncommitted work.
- Clearly identify generated files and hand-written files.
- Clearly identify algorithm-only functionality that is not connected to real
  actuators.
- Clearly distinguish component validation, subsystem integration, and full
  system validation.
- An optional external reporting skill may be used as assistance, but this
  repository guide remains the project source of truth.
- If an external skill or referenced template is missing, continue using this
  guide and record the missing optional dependency briefly.
