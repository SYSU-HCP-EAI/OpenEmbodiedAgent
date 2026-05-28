# Communication Architecture / 通信架构说明

This document explains how Physical Agent Operating System components communicate at runtime.
It is a bilingual architectural guide, not a live protocol bus by itself.

本文说明 Physical Agent Operating System 在运行时如何通信。
它是一份中英双语的架构说明书，不是实际承载通信的运行态总线。

## 1. Core Principle / 核心原则

Physical Agent Operating System follows a Markdown-first design:

- Track A (Agent side) plans, reasons, and validates.
- Track B (Runtime / HAL side) executes through watchdog-supervised targets and skills.
- Shared state is exposed through Markdown files instead of direct cross-layer Python calls.

Physical Agent Operating System 采用 Markdown-first 设计：

- Track A（Agent 侧）负责理解、规划、校验。
- Track B（Runtime / HAL 侧）负责通过 watchdog 监督的 target 与 skill 执行。
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
- `TARGETS.md`, `SKILLS.md`, `SESSIONS.md`
  - Runtime target registry, skill registry, and session queue
  - Used by the session-centered runtime instead of direct Agent-to-target calls
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
- `TARGETS.md`、`SKILLS.md`、`SESSIONS.md`
  - Runtime target registry、skill registry 与 session 队列
  - 被 session-centered runtime 使用，避免 Agent 直接调用 target
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

The session-centered `WatchdogSupervisor` reads:

- `TARGETS.md`
- `SKILLS.md`
- `SESSIONS.md`
- external runtime YAML under `configs/runtime/`

It writes:

- session status and result back to `SESSIONS.md`
- perception deltas to `ENVIRONMENT.md`
- reusable preflight failures to `LESSONS.md`
- transient runtime history to `LOG.md`
- episode artifacts under `artifacts/runtime/<session_id>/`

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

- session-centered `WatchdogSupervisor` 读取：
  - `TARGETS.md`
  - `SKILLS.md`
  - `SESSIONS.md`
  - `configs/runtime/` 下的外部 runtime YAML
- 它写入：
  - `SESSIONS.md` 中的 session 状态与结果
  - `ENVIRONMENT.md` 中的感知增量
  - `LESSONS.md` 中可复用的 preflight 失败经验
  - `LOG.md` 中的临时 runtime history
  - `artifacts/runtime/<session_id>/` 下的 episode artifacts

## 6. Runtime Session Protocol / Runtime Session 协议

The runtime protocol keeps the upper/lower boundary file-based while moving execution to sessions:

- `TARGETS.md` answers which targets exist, whether they are enabled, which skills they support, and which target class/kind, runtime endpoint, target adapter, sensor config, perception config, and runtime contract they use.
- `SKILLS.md` declares `runtime_kind`, loop mode, agent exposure, supported target kinds, policy requirements, observation contract, required sensors/environment outputs, output action contract, target-tool policy, and allowed deterministic bridges.
- `SESSIONS.md` declares a task, target, skill, timeout, priority, and routing hints. It does not bind pair adapters.
- `configs/runtime/contracts/<target_id>.runtime.yaml` declares target action contract and safety limits.
- Adapter and bridge references use explicit URI namespaces such as `target_adapter://`, `policy_adapter://`, and `bridge://`.

Runtime status flow:

```text
pending -> claimed -> preflight_checking -> running -> succeeded / failed / timed_out / cancelled
preflight_checking -> rejected
```

`RuntimeCompatibilityPreflight` resolves an `AdapterPlan` before execution. It validates protocol files, target/skill compatibility, adapter/bridge availability, sensor config declarations, perception config declarations, and runtime contracts. Actual target observation channels are checked when runtime reads an observation for environment refresh or skill execution. After preflight, `WatchdogSupervisor` creates a `SessionRunner`; the runner owns target lifecycle and exposes the target to policy or builtin skills only through `TargetSessionHandle`. Target runtimes and policy servers do not call each other directly.

Runtime 协议继续保持文件边界，但执行单位变成 session：

- `TARGETS.md` 描述有哪些 target、是否启用、支持哪些 skill，以及 target class/kind、runtime endpoint、target adapter、sensor config、perception config、runtime contract。
- `SKILLS.md` 声明 `runtime_kind`、loop mode、agent exposure、支持的 target kinds、policy 需求、observation contract、所需 sensors/environment outputs、输出动作契约、target-tool policy 和允许的确定性 bridge。
- `SESSIONS.md` 声明任务、target、skill、timeout、priority 和 routing hints，不绑定 pair adapter。
- `configs/runtime/contracts/<target_id>.runtime.yaml` 声明 target action contract 与安全限制。
- Adapter 与 bridge 引用使用 `target_adapter://`、`policy_adapter://`、`bridge://` 等显式 URI 命名空间。

运行状态流：

```text
pending -> claimed -> preflight_checking -> running -> succeeded / failed / timed_out / cancelled
preflight_checking -> rejected
```

`RuntimeCompatibilityPreflight` 会在执行前解析 `AdapterPlan`，并校验协议文件、target/skill 兼容性、adapter/bridge 可用性、sensor config 声明、perception config 声明和 runtime contract。真实 target observation 的 channel 会在 runtime 为环境刷新或技能执行读取 observation 时校验。Preflight 通过后，`WatchdogSupervisor` 创建 `SessionRunner`；runner 负责 target lifecycle，并且只通过 `TargetSessionHandle` 把 target 暴露给 policy 或 builtin skill。Target runtime 与 policy server 不直接互相调用。

## 7. Typical Runtime Pipeline / 典型运行流程

1. `paos onboard` prepares workspaces.
2. Runtime protocol files define targets, skills, sessions, and external configs.
3. User starts the runtime watchdog.
4. User starts `paos agent`.
5. Agent plans from shared state and writes or updates a session.
6. Watchdog claims a pending session and runs compatibility preflight.
7. If accepted, watchdog creates a `SessionRunner`; the runner enters `running`, configures and starts the target session, then runs the selected skill runtime through `TargetSessionHandle`.
8. Runtime writes session results, environment deltas, lessons, and artifacts.

1. `paos onboard` 准备工作区。
2. Runtime 协议文件定义 targets、skills、sessions 与外部配置。
3. 用户启动 runtime watchdog。
4. 用户启动 `paos agent`。
5. Agent 基于 shared state 规划并写入或更新 session。
6. Watchdog claim pending session 并执行 compatibility preflight。
7. 如通过，watchdog 创建 `SessionRunner`；runner 进入 `running`，配置并启动 target session，然后通过 `TargetSessionHandle` 运行选定的 skill runtime。
8. Runtime 写回 session result、environment delta、lessons 与 artifacts。

## 8. Design Intent / 设计意图

- Keep shared context concise enough for planning
- Keep robot-specific validation precise
- Keep runtime state visible and inspectable
- Avoid hiding hardware facts inside opaque code paths

- 让 shared 上下文足够简洁，便于规划
- 让机器人级校验保持精确
- 让运行态保持可见、可检查
- 避免把硬件事实藏在不可见的黑盒代码路径里
