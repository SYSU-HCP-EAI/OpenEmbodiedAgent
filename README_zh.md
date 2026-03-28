<div align="center">
  <img src="docs/imgs/logo.png" alt="OpenEmbodiedAgent" width="500">
  <h1>OpenEmbodiedAgent (OEA)</h1>
  <p><b>一种基于协议解耦与多智能体协同的自进化具身框架</b></p>
  <p>
    <a href="./README.md">English</a> | <a href="./README_zh.md">中文</a>
  </p>
  <p>
    <img src="https://img.shields.io/badge/version-0.05-blue" alt="Version">
    <img src="https://img.shields.io/badge/python-≥3.10-blue" alt="Python">
    <img src="https://img.shields.io/badge/license-MIT-green" alt="License">
  </p>
</div>

## 📖 简介

**OpenEmbodiedAgent (OEA)** 是一种基于 Agentic 工作流的自进化具身智能框架。它摒弃了传统**大模型直接控制硬件**的黑盒模式，首创了**认知-物理解耦**的架构范式，通过构建语言-动作接口（Language-Action Interface），将动作表示与本体形态彻底解耦，实现了从强推理云端模型到边缘物理执行层的标准化映射。

OEA 采用**万物皆 Markdown** (State-as-a-File)的协议矩阵，原生支持跨硬件平台的零代码迁移、基于沙盒的代码工具自生成，以及基于多智能体验证（Multi-Agent Critic）的安全纠偏机制。

## ✨ 核心特性

*   📝 **万物皆 Markdown (State-as-a-File)**: 软硬件通过读写本地 Markdown 文件（如 `ENVIRONMENT.md`、`ACTION.md`）进行通信，彻底解耦，极度透明。
*   🧠 **双轨多体系统**:
    *   **Track A (大脑)**: 包含 Planner (规划) 与 Critic (校验) 机制。大模型不直接下发指令，必须经过 Critic 对照当前机器人运行时 `EMBODIED.md`（由 profile 复制而来）的能力约束校验后才落盘。
    *   **Track B (物理执行)**: 独立的硬件看门狗 (`hal_watchdog.py`) 监听指令并执行。支持单实例模式和多机器人协同的 **Fleet 模式**。
*   🔌 **动态插件机制**: 支持通过 `hal/drivers/` 动态加载外部硬件驱动，无需修改核心代码即可扩展新硬件支持。
*   🛡️ **安全纠偏机制**: 严格的动作校验与 `LESSONS.md` 经验避坑库，防止 Agent 工作流失控。
*   🎮 **仿真环境闭环**: 内置轻量级仿真支持，无需真实硬件即可验证从自然语言指令到物理状态改变的全链路。
*   🗺️ **语义导航与感知**: 内置 `SemanticNavigationTool` 和 `PerceptionService`，支持将高层语义目标解析为物理坐标，并融合几何与语义信息构建场景图。

## 🦾 样例演示

<div align="center">
  <img src="docs/imgs/setup.gif" alt="rekep" width="900">
  <br>
  OEA一键部署机械臂，无需写代码 （松灵 PIPER）
</div>


<div align="center">
  <img src="docs/imgs/SAM3.gif" alt="rekep" width="900">
  <br>
  OEA通过SAM3实现自然语言驱动的抓取任务 （松灵 PIPER）
</div>


<div align="center">
  <img src="docs/imgs/ReKep.gif" alt="rekep" width="900">
  <br>
  OEA通过ReKep实现自然语言驱动的抓取任务 (越疆 DBot)
</div>

## 🏗️ 架构设计

OEA 的核心是一个本地工作区（Workspace），软硬件作为独立的守护进程对文件进行读写：

<div align="center">
  <img src="docs/imgs/oea_zh.png" alt="OpenEmbodiedAgent" width="900">
</div>

## 🚀 Quick Start

### 1. 安装依赖

```bash
git clone https://github.com/your-repo/OpenEmbodiedAgent.git
cd OpenEmbodiedAgent
pip install -e .
# 安装仿真环境依赖 (如 watchdog)
pip install watchdog

# 可选：安装外部 ReKep 真机插件
python scripts/deploy_rekep_real_plugin.py \
  --repo-url https://github.com/baiyu858/oea-rekep-real-plugin.git
```

### 2. 初始化工作区

```bash
OEA onboard
```
这会在当前工作区生成核心 Markdown 协议文件。
单实例模式默认使用 `~/.OEA/workspace/`；fleet 模式会使用 `~/.OEA/workspaces/` 下的 shared 工作区和多个机器人工作区。

### 3. 启动系统

需要开启两个终端：

**终端 1: 启动硬件看门狗与仿真环境 (Track B)**
```bash
python hal/hal_watchdog.py
```

如果要使用真机 ReKep 而不是仿真，请先安装插件，再执行：

```bash
python hal/hal_watchdog.py --driver rekep_real
```

**终端 2: 启动大脑 Agent (Track A)**
```bash
OEA agent
```

### 4. 交互示例

在 `OEA agent` 的 CLI 中输入：
> "看看桌子上有什么，然后把那个苹果抓起来给我。"

你将在终端 1 的仿真日志中看到动作的执行，并在终端 2 收到 Agent 的完成确认。

## 📁 Project Structure

```text
OpenEmbodiedAgent/
├── OEA/                # Track A: 软件大脑核心 (基于 OEA 扩展)
│   ├── agent/              # Agent 逻辑 (Planner, Critic)
│   ├── templates/          # Workspace Markdown 模板（只定义协议结构）
│   └── ...
├── hal/                    # Track B: 硬件小脑与仿真 (新增)
│   ├── hal_watchdog.py     # 硬件看门狗守护进程
│   └── simulation/         # 仿真环境相关代码
├── scripts/                # 外部 HAL 插件部署脚本
│   └── deploy_rekep_real_plugin.py
├── workspace/              # 单实例运行时工作区（兼容默认模式）
│   ├── EMBODIED.md         # 从 hal/profiles/ 复制来的运行时机器人 profile
│   ├── ENVIRONMENT.md      # 当前环境 Scene-Graph
│   ├── ACTION.md           # 待执行的动作指令
│   ├── LESSONS.md          # 失败经验记录
│   └── SKILL.md            # 成功工作流 SOP
├── workspaces/             # fleet 模式拓扑
│   ├── shared/             # Agent 工作区与全局 ENVIRONMENT.md
│   ├── go2_edu_001/        # 机器人本地 ACTION.md / EMBODIED.md
│   └── ...
├── docs/                   # 项目文档
│   ├── PLAN.md             # 详细实施方案
│   └── PROJ.md             # 项目白皮书与架构设计
├── README.md               # 英文说明
└── README_zh.md            # 中文说明
```

## 🗺️ 演进路线图

- **Phase 1**: 桌面闭环与 Markdown 协议确立。
    - [x] v0.0.1: 完成框架设计与初始化
    - [x] v0.0.2: 完成插件形式的embodied skill部署与调用的设计
    - [x] v0.0.3: 完成视觉解耦+抓取的通路（SAM3和ReKep）
    - [x] v0.0.4: 完成基于原子动作的VLN通路（SAM3）
    - [x] v0.0.5: 多智能体协议的初步设计
    - [ ] v0.0.6: 长程任务的拆解、编排与执行
    - [ ] v0.0.7: 对小智等IoT设备的接入
- **Phase 2**: 多本体协同与多模态记忆。
- **Phase 3**: 约束求解与高阶异构协同。

## 🛠️ 支持设备

OEA 通过 HAL (Hardware Abstraction Layer) 协议支持多种具身本体。

| 本体类型 | 具体型号 | 适配状态 | 备注 |
| :--- | :--- | :--- | :--- |
| **桌面级机械臂** | 松灵 PIPER | 🟢 可支持 | 已验证 ReKep & SAM3 全链路 |
| **复合协作机器人** | 松灵 PIPER + Unitree Go2 | 🟡 部分支持 | locomotion适配中 |
| **桌面级机械臂** | 越疆 DoBot Nova 2 | 🟢 可支持 | 已验证 ReKep 部署 |
| **四足机器人** | Unitree Go2 | 🟡 部分支持 | 目前仅适配移动与语义导航 |
| **双臂操纵** | XLeRobot | 🟡 部分支持 | 目前仅支持双臂操纵协议 |
| **IoT 设备** | 小智 (ESP32) | 🟡 部分支持 | 目前仅实现语音对话交互 |
| **工业级机器人** | Franka Research 3 | ⚪ 未测试 | 驱动协议对接中 |
| **教育机器人** | 幻尔系列 | 🔴 未适配 | 待开发驱动插件 |
| **通用环境** | 内置仿真器 | 🟢 可支持 | 基于本地磁盘映射的轻量仿真 |

> **说明**: OEA 采用插件化设计，任何支持 Python 控制接口的硬件均可通过 `hal/drivers/` 快速接入。

## 🤝 参与贡献

欢迎提交 PR 或 Issue！请参考 `docs/USER_DEVELOPMENT_GUIDE.md` 了解详细的架构设计与开发指南。

---

**特别鸣谢**：本项目基于 [nanobot](https://github.com/HKUDS/nanobot) 开发，感谢其提供的超轻量级 Agent 框架。欢迎大家前往 [nanobot](https://github.com/HKUDS/nanobot) 点赞支持！
