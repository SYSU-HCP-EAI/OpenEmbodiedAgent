<div align="center">
  <img src="docs/imgs/logo_en.png" alt="PhyAgentOS" width="560">

  <h3>认知与物理解耦 —— 面向具身智能的 Session-Centered 运行时</h3>

  <p>
    <a href="https://github.com/PhyAgentOS/PhyAgentOS/stargazers">
      <img src="https://img.shields.io/github/stars/PhyAgentOS/PhyAgentOS?style=social" alt="Stars">
    </a>
    <a href="https://github.com/PhyAgentOS/PhyAgentOS/network/members">
      <img src="https://img.shields.io/github/forks/PhyAgentOS/PhyAgentOS?style=social" alt="Forks">
    </a>
  </p>
  <p>
    <img src="https://img.shields.io/badge/Python-≥3.11-3776AB?logo=python&logoColor=white" alt="Python">
    <img src="https://img.shields.io/badge/License-MIT-3DA639" alt="License">
    <a href="https://sysu-hcp-eai.github.io/PhyAgentOS-website/">
      <img src="https://img.shields.io/badge/🌐_Website-online-FF6B35" alt="Website">
    </a>
    <a href="https://github.com/PhyAgentOS/PhyAgentOS">
      <img src="https://img.shields.io/badge/PRs-Welcome-2EA44F" alt="PRs">
    </a>
  </p>
  <p>
    <sub><a href="./README.md">English</a> · <a href="./README_zh.md">中文</a></sub>
  </p>
</div>

---

## 📢 更新日志

| 版本 | 日期 | 更新内容 |
|:-----|:-----|:---------|
| ![v0.2.1](https://img.shields.io/badge/v0.2.1-FF574F) | 2026-05-29 | 基于 ![v0.1.3](https://img.shields.io/badge/v0.1.3-47A882) 的MineCraft 就绪，以云端agent接入用户的本地服务器 |
| ![v0.1.3](https://img.shields.io/badge/v0.1.3-47A882) | 2026-05-25 | `PolicySkillRuntime` / `BuiltinSkillRuntime` 边界严格分离，Game Agent & Benchmarking 就绪 |
| ![v0.1.2](https://img.shields.io/badge/v0.1.2-11648A) | 2026-05-20 | 感知插件体系：`SensorConfig` / `PerceptionConfig` YAML + `EnvironmentWriter` 可审计写回 |
| ![v0.1.1](https://img.shields.io/badge/v0.1.1-11648A) | 2026-05-18 | Session-Centered Runtime MVP：`DummySimTarget` + `DummyAdapter` + `DummyClient` 串行链路 |
| ![v0.1.0](https://img.shields.io/badge/v0.1.0-11648A) | 2026-04-29 | Hackathon 基线：插件化 HAL，ReKep / SAM3 真机抓取与 VLN 全链路 |

---

## 🤔 为什么选择 PhyAgentOS？

传统的"大模型直连硬件"方案高度耦合，换一个机器人就要重写整个执行链路。PhyAgentOS 通过 **认知-物理解耦 + Session-Centered Runtime** 彻底改变了这一点：

<table>
<tr><td width="32">🔌</td><td><b>同代码，万硬件</b> — 新增机器人只需实现一个 Target Adapter（~100 行），调度层零改动。</td></tr>
<tr><td>🛡️</td><td><b>三道安全防线</b> — Critic 校验 → Strict Preflight → Target-side SafetyGuard，真机场景不可绕过。</td></tr>
<tr><td>📋</td><td><b>全程可审计</b> — 状态、动作、感知结果以 Markdown + YAML 落盘，每一步可追溯复现。</td></tr>
<tr><td>🔄</td><td><b>零摩擦迁移</b> — 同一套 Session 协议在 sim / real / game 三类 target 上无差别运行。</td></tr>
</table>

<br>

<div align="center">
  <img src="docs/imgs/framework.png" alt="架构图" width="960">
  <p><sub>▲ Session-Centered Runtime 架构全览</sub></p>
</div>

---

## ✨ 核心特性

<table>
<tr>
  <td width="32">🔄</td>
  <td width="160"><b>Session-Centered Runtime</b></td>
  <td><code>WatchdogSupervisor</code> → <code>SessionRunner</code> → <code>SkillRuntime</code> → <code>TargetSessionHandle</code> 执行链路，抛弃 Driver-Center 旧架构</td>
</tr>
<tr>
  <td>🎯</td>
  <td><b>Target-Configured</b></td>
  <td><code>game</code> / <code>debug</code> / <code>simulation</code> / <code>real_robot</code> 四类 target，<code>TARGETS.md</code> 统一注册，adapter 按需挂载</td>
</tr>
<tr>
  <td>🧩</td>
  <td><b>Adapter + Bridge</b></td>
  <td><code>TargetAdapter</code> + <code>PolicyAdapter</code> + <code>ActionBridge</code> 三段解耦，<code>AdapterPlan</code> 自动编排，消灭 target×skill 组合爆炸</td>
</tr>
<tr>
  <td>⚡</td>
  <td><b>双轨 Skill 运行时</b></td>
  <td><code>PolicySkillRuntime</code> 维护 policy 闭环 + <code>BuiltinSkillRuntime</code> 管理 agent 交互闭环</td>
</tr>
<tr>
  <td>🛡️</td>
  <td><b>Strict Preflight</b></td>
  <td>10 项前置校验（target / sensor / perception / contract / tool），不合格直接 <code>rejected</code></td>
</tr>
<tr>
  <td>📝</td>
  <td><b>文件协议矩阵</b></td>
  <td><code>TARGETS.md</code> · <code>SKILLS.md</code> · <code>SESSIONS.md</code> · <code>ENVIRONMENT.md</code> · <code>LESSONS.md</code> + 外部 YAML</td>
</tr>
<tr>
  <td>🔐</td>
  <td><b>多层安全</b></td>
  <td>Critic 校验 → Preflight 契约检查 → Target-side SafetyGuard → Operator Override</td>
</tr>
<tr>
  <td>🌐</td>
  <td><b>Fleet 模式</b></td>
  <td>多机器人协同，shared + per-robot 工作区，优先级串行调度</td>
</tr>
</table>

---

## 🚀 5 分钟快速开始

<table>
<tr>
<td width="28" align="center">1</td>
<td>

**安装**

```bash
git clone https://github.com/PhyAgentOS/PhyAgentOS.git && cd PhyAgentOS
pip install -e .            # Python ≥ 3.11
pip install -e ".[dev]"     # 开发依赖
```
</td>
</tr>
<tr>
<td align="center">2</td>
<td>

**初始化工作区**

```bash
paos onboard
```
</td>
</tr>
<tr>
<td align="center">3</td>
<td>

**终端 1：启动 Runtime（Track B）**

```bash
python -m PhyAgentOS.runtime.watchdog
```
</td>
</tr>
<tr>
<td align="center">4</td>
<td>

**终端 2：启动 Agent（Track A）**

```bash
paos agent
```
</td>
</tr>
</table>

在 Agent CLI 中输入自然语言指令即可驱动硬件。无需硬件？运行 Smoke Test 验证全链路：

```bash
python scripts/init_runtime_workspace.py --workspace /tmp/paos_runtime_smoke
python scripts/run_runtime_watchdog.py --workspace /tmp/paos_runtime_smoke --once
# → session 标记 succeeded，结果写入 artifacts/
```

---

## 📦 项目结构

```
PhyAgentOS/
│
├── PhyAgentOS/agent/          # Track A  ─  Planner / Critic / Memory
│
├── PhyAgentOS/runtime/        # Track B  ─  执行平面
│   ├── watchdog/              #   WatchdogSupervisor
│   ├── sessions/              #   SessionRunner / TargetSessionHandle
│   ├── targets/               #   RolloutTarget (game·debug·sim·real)
│   ├── skills/                #   PolicySkillRuntime / BuiltinSkillRuntime
│   ├── adapters/              #   TargetAdapter / PolicyAdapter / Bridge
│   ├── perception/            #   感知运行时 / EnvironmentWriter
│   ├── preflight/             #   RuntimeCompatibilityPreflight
│   └── schemas/               #   Pydantic Schema
│
├── configs/runtime/           # Sensor / Perception / Contract YAML
├── scripts/                   # 工具脚本
├── workspace/                 # 运行时工作区
├── docs/                      # 文档
└── tests/                     # 测试
```

---

## 🏷️ 支持目标

| | Kind | 位置 | 示例 |
|:--|:-----|:-----|:-----|
| 🎮 | `game` | Local | Minecraft、星露谷物语 —— 低成本验证长期决策与记忆 |
| 🐛 | `debug` | Local | echo / mock / dry-run —— 零硬件验证协议链路 |
| 🧪 | `simulation` | Remote | RoboCasa、LIBERO —— Benchmark 评测与批量经验挖掘 |
| 🤖 | `real_robot` | Remote | Franka、Go2、XLeRobot、AgileX PIPER —— 真实运行 |

> 全部 target 通过 `TARGETS.md` 统一注册，`target_adapter://` URI 标识 adapter。
> 更多实例与演示 → [项目网站](https://phy-agent-os.net/)

---

## 📖 文档

| 文档 | 面向 | 说明 |
|:-----|:-----|:-----|
| [🌐 项目网站](https://phy-agent-os.net/docs/en/architecture.html) | 所有人 | 完整文档、架构详解、Demo 演示 |
| [📘 用户手册](https://phy-agent-os.net/docs/en/api-reference.html) | 使用者 | 安装部署、运行操作指南 |
| [📙 开发指南](https://phy-agent-os.net/docs/en/developer-guide.html) | 开发者 | 二次开发、硬件接入、插件编写 |

---

## 🤝 参与贡献

欢迎提交 PR 和 Issue，我们的开发计划可以在此处查看👉 [开发计划](https://phy-agent-os.net/docs/en/developer-guide.html)。

---

<div align="center">

本项目基于 **[nanobot](https://github.com/HKUDS/nanobot)** 构建

由 **中山大学 HCP 实验室** 与 **鹏城实验室** 联合开发

<br>

<img src="docs/imgs/SYSU.png" alt="SYSU" height="128">
&nbsp;&nbsp;&nbsp;
<img src="docs/imgs/Pengcheng.png" alt="Pengcheng" height="128">
&nbsp;&nbsp;&nbsp;
<img src="docs/imgs/HCP.jpg" alt="HCP" height="128">

<br>
<sub>MIT License · Copyright © 2025-2026 PhyAgentOS</sub>

</div>
