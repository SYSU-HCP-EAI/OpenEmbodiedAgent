# Communication Architecture / 通信架构说明

This document explains how Physical Agent Operating System components communicate at runtime.
It is a bilingual architectural guide, not a live protocol bus by itself.

本文说明 Physical Agent Operating System 在运行时如何通信。
它是一份中英双语的架构说明书，不是实际承载通信的运行态总线。

## 1. Core Principle / 核心原则

Physical Agent Operating System follows a Markdown-first design:

- Track A (Agent side) plans, reasons, and validates.
- Track B (HAL side) executes through drivers and watchdogs.
- Shared state is exposed through Markdown files instead of direct cross-layer Python calls.

Physical Agent Operating System 采用 Markdown-first 设计：

- Track A（Agent 侧）负责理解、规划、校验。
- Track B（HAL 侧）负责通过 driver 和 watchdog 执行。
- 跨层共享状态优先通过 Markdown 文件暴露，而不是直接跨层 Python 调用。

## 2. Workspaces / 工作区拓扑

### Single mode

- One workspace, usually `~/.PhyAgentOS/workspace`
- Agent and watchdog both operate around the same runtime directory

### Fleet mode

- One shared workspace, usually `~/.PhyAgentOS/workspaces/shared`
- One robot workspace per embodied instance, for example:
  - `~/.PhyAgentOS/workspaces/go2_edu_001`
  - `~/.PhyAgentOS/workspaces/desktop_pet_001`

单实例模式：

- 只有一个 workspace，通常是 `~/.PhyAgentOS/workspace`
- Agent 和 watchdog 围绕同一个运行目录工作

Fleet 模式：

- 一个 shared workspace，通常是 `~/.PhyAgentOS/workspaces/shared`
- 每个机器人实例一个 robot workspace，例如：
  - `~/.PhyAgentOS/workspaces/go2_edu_001`
  - `~/.PhyAgentOS/workspaces/desktop_pet_001`

## 3. File Responsibilities / 文件职责

### Shared workspace files

- `ENVIRONMENT.md`
  - Global environment truth source
  - Scene graph, map, TF, and per-robot runtime state
- `ROBOTS.md`
  - Auto-generated fleet directory
  - Summarizes robot id, driver, type, concise capability summary, workspace, enablement, connection state, and nav state
- `LESSONS.md`
  - Shared failure memory and action rejection notes
- `TASK.md`
  - Multi-step task decomposition state
- `ORCHESTRATOR.md`
  - Global supervision and coordination state

Shared workspace 文件：

- `ENVIRONMENT.md`
  - 全局环境真相源
  - 保存 scene graph、map、TF 和各机器人的运行态
- `ROBOTS.md`
  - 自动生成的机器人目录
  - 摘要记录 robot id、driver、类型、简要能力、workspace、启用状态、连接状态、导航状态
- `LESSONS.md`
  - 共享失败经验和动作拒绝记录
- `TASK.md`
  - 多步骤任务拆解状态
- `ORCHESTRATOR.md`
  - 全局监督与协调状态

### Robot workspace files

- `ACTION.md`
  - Action queue for one robot instance only
- `EMBODIED.md`
  - Runtime robot profile copied from `hal/profiles/*.md`
  - Used by Critic validation for that specific robot

Robot workspace 文件：

- `ACTION.md`
  - 单个机器人实例自己的动作队列
- `EMBODIED.md`
  - 从 `hal/profiles/*.md` 复制来的运行时机器人 profile
  - 被 Critic 用来校验这台机器人的具体动作

## 4. Template vs Profile / 模板与 Profile 的区别

`PhyAgentOS/templates/EMBODIED.md` is only a structural template.
It explains:

- what sections `EMBODIED.md` should contain
- what each section means
- what belongs in static profile data
- what belongs in runtime state instead

Concrete robot values must live in `hal/profiles/*.md`.

`PhyAgentOS/templates/EMBODIED.md` 只是结构模板。
它用于说明：

- `EMBODIED.md` 应包含哪些 section
- 每个 section 的作用是什么
- 哪些信息属于静态 profile
- 哪些信息应该写进运行态文件

具体机器人参数必须写在 `hal/profiles/*.md`。

## 5. Who Reads What / 谁读取什么

### Planner / main Agent

Usually reads the shared workspace:

- `ENVIRONMENT.md`
- `ROBOTS.md`
- `LESSONS.md`
- `TASK.md`
- `ORCHESTRATOR.md`

The main Agent does not automatically ingest every robot profile in fleet mode.

### Critic via `EmbodiedActionTool`

When validating one action for one robot, it reads:

- shared `ENVIRONMENT.md`
- target robot's runtime `EMBODIED.md`
- action draft and reasoning

This means capability-specific validation happens at dispatch time.

### Watchdog

Each watchdog instance reads:

- its own robot workspace `ACTION.md`
- shared `ENVIRONMENT.md`

Each watchdog writes:

- runtime profile copy into its robot workspace `EMBODIED.md`
- robot runtime state back into shared `ENVIRONMENT.md`
- refreshed summary into shared `ROBOTS.md`

Planner / 主 Agent：

- 默认主要读取 shared workspace：
  - `ENVIRONMENT.md`
  - `ROBOTS.md`
  - `LESSONS.md`
  - `TASK.md`
  - `ORCHESTRATOR.md`
- 在 fleet 模式下，不会默认把每台机器人的完整 profile 全量注入上下文

Critic（通过 `EmbodiedActionTool`）：

- 对某个机器人做动作校验时，会读取：
  - shared `ENVIRONMENT.md`
  - 目标机器人的 runtime `EMBODIED.md`
  - 当前动作草案与 reasoning
- 也就是说，针对具体机器人能力的精确判断发生在动作派发阶段

Watchdog：

- 每个 watchdog 实例读取：
  - 自己 robot workspace 里的 `ACTION.md`
  - shared `ENVIRONMENT.md`
- 每个 watchdog 写入：
  - 自己 robot workspace 里的 runtime `EMBODIED.md`
  - shared `ENVIRONMENT.md` 里的本机器人运行态
  - shared `ROBOTS.md` 里的摘要目录

## 6. Typical Runtime Pipeline / 典型运行流程

1. `paos onboard` prepares workspaces.
2. Fleet config defines which robots exist.
3. Shared `ROBOTS.md` is generated from config plus runtime summary.
4. User starts one watchdog per robot instance.
5. Watchdog installs that robot's profile as runtime `EMBODIED.md`.
6. User starts `paos agent`.
7. Agent plans from shared state and chooses a robot.
8. `EmbodiedActionTool` validates against the target robot's runtime `EMBODIED.md`.
9. The action is written to that robot workspace's `ACTION.md`.
10. The matching watchdog executes it and writes runtime updates back to shared files.

1. `paos onboard` 准备工作区。
2. Fleet 配置定义有哪些机器人实例。
3. shared `ROBOTS.md` 根据配置和运行摘要自动生成。
4. 用户为每个机器人实例启动一个 watchdog。
5. watchdog 将该机器人的 profile 安装为 runtime `EMBODIED.md`。
6. 用户启动 `paos agent`。
7. Agent 基于 shared 状态规划并选择机器人。
8. `EmbodiedActionTool` 使用目标机器人的 runtime `EMBODIED.md` 做校验。
9. 动作被写入该机器人 workspace 的 `ACTION.md`。
10. 对应 watchdog 执行动作，并把运行结果回写到 shared 文件。

## 7. Design Intent / 设计意图

- Keep shared context concise enough for planning
- Keep robot-specific validation precise
- Keep runtime state visible and inspectable
- Avoid hiding hardware facts inside opaque code paths

- 让 shared 上下文足够简洁，便于规划
- 让机器人级校验保持精确
- 让运行态保持可见、可检查
- 避免把硬件事实藏在不可见的黑盒代码路径里
