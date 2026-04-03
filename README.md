<div align="center">
  <img src="docs/imgs/PhyAgentOS.png" alt="PhyAgentOS" width="500">
  <h1>Physical Agent Operation System (PhyAgentOS)</h1>
  <p><b>A Decoupled Protocol-Based Framework for Self-Evolving and Cross-Embodiment Agents</b></p>
  <p>
    <a href="./README.md">English</a> | <a href="./README_zh.md">中文</a>
  </p>
  <p>
    <img src="https://img.shields.io/badge/version-0.0.5-blue" alt="Version">
    <img src="https://img.shields.io/badge/python-≥3.10-blue" alt="Python">
    <img src="https://img.shields.io/badge/license-MIT-green" alt="License">
    <a href="https://sysu-hcp-eai.github.io/PhyAgentOS-website/">
        <img src="https://img.shields.io/badge/🔗_Website-online-orange" alt="Website">
    </a>
  </p>
</div>

## Long Demo    
[![Watch the video](https://img.youtube.com/vi/LtUWamZRyhM/maxresdefault.jpg)](https://youtu.be/LtUWamZRyhM?si=UjKNdqFnO1knfWbX)

## 📖 Introduction

**Physical Agent Operation System (PhyAgentOS)** is a self-evolving embodied AI framework based on Agentic workflows. Moving away from the "black-box" model of traditional "large models directly controlling hardware," PhyAgentOS pioneers a **"Cognitive-Physical Decoupling"** architectural paradigm. By constructing a Language-Action Interface, it completely decouples action representation from embodiment morphology, enabling standardized mapping from high-reasoning cloud models to edge physical execution layers.

PhyAgentOS utilizes a **"State-as-a-File"** protocol matrix, natively supporting zero-code migration across hardware platforms, sandbox-driven tool self-generation, and safety correction mechanisms based on Multi-Agent Critic verification.

## ✨ Core Features

*   📝 **State-as-a-File**: Software and hardware communicate by reading/writing local Markdown files (e.g., `ENVIRONMENT.md`, `ACTION.md`), ensuring complete decoupling and extreme transparency.
*   🧠 **Dual-Track Multi-Agent System**:
    *   **Track A (Cognitive Core)**: Includes Planner and Critic mechanisms. Large models do not issue commands directly; they must be verified by the Critic against the current robot's runtime `EMBODIED.md` (copied from profiles) before being committed.
    *   **Track B (Physical Execution)**: An independent hardware watchdog (`hal_watchdog.py`) monitors and executes commands. Supports both single-instance mode and **Fleet mode** for multi-robot coordination.
*   🔌 **Dynamic Plugin Mechanism**: Supports dynamic loading of external hardware drivers via `hal/drivers/`, allowing for new hardware support without modifying core code.
*   🛡️ **Safety Correction Mechanism**: Strict action verification and `LESSONS.md` experience library prevent Agent workflows from going out of control.
*   🎮 **Simulation Loop**: Built-in lightweight simulation support allows verification of the full chain from natural language instructions to physical state changes without real hardware.
*   🗺️ **Semantic Navigation & Perception**: Built-in `SemanticNavigationTool` and `PerceptionService` support resolving high-level semantic goals into physical coordinates and constructing scene graphs by fusing geometric and semantic information.

## 🦾 Showcase

<div align="center">
  <img src="docs/imgs/setup.gif" alt="rekep" width="900">
  <br>
  PhyAgentOS deploys robot arms with one click, no coding required (AgileX PIPER).
</div>

<div align="center">
  <img src="docs/imgs/SAM3.gif" alt="rekep" width="900">
  <br>
  PhyAgentOS achieves natural language-driven grasping tasks through SAM3 (AgileX PIPER).
</div>

<div align="center">
  <img src="docs/imgs/ReKep.gif" alt="rekep" width="900">
  <br>
  PhyAgentOS achieves natural language-driven grasping tasks through ReKep (Dobot Nova 2).
</div>

<div align="center">
  <img src="docs/imgs/Franka_QA_Pick&Up.gif" alt="rekep" width="900">
  <br>
  PhyAgentOS achieves realtime dialog and natural language-driven pick&up task through ReKep (Franka Research 3).
</div>

## 🏗️ Architecture

PhyAgentOS's core is a local workspace where software and hardware operate as independent daemons reading/writing files:

<div align="center">
  <img src="docs/imgs/PhyAgentOS_en.png" alt="PhyAgentOS" width="900">
</div>

## 🚀 Quick Start

### 1. Install Dependencies
```bash
git clone https://github.com/your-repo/Physical Agent Operating System.git
cd Physical Agent Operating System
pip install -e .
# Install simulation dependencies (e.g., watchdog)
pip install watchdog

# Optional: Install external ReKep real-world plugin
python scripts/deploy_rekep_real_plugin.py \
  --repo-url https://github.com/baiyu858/PhyAgentOS-rekep-real-plugin.git
```

### 2. Initialize Workspace
```bash
paos onboard
```
This generates core Markdown protocol files in the current workspace. Single-instance mode defaults to `~/.PhyAgentOS/workspace/`; Fleet mode uses a shared workspace and multiple robot workspaces under `~/.PhyAgentOS/workspaces/`.

### 3. Start the System
Open two terminals:

**Terminal 1: Start Hardware Watchdog & Simulation (Track B)**
```bash
python hal/hal_watchdog.py
```
To pass driver-specific runtime configuration without specializing the watchdog CLI, use:
```bash
python hal/hal_watchdog.py --driver <driver_name> --driver-config path/to/driver.json
```
The config file must be a JSON object and its keys are passed through to the selected driver constructor unchanged.

To use real-world ReKep instead of simulation, install the plugin and run:
```bash
python hal/hal_watchdog.py --driver rekep_real
```

**Terminal 2: Start Brain Agent (Track A)**
```bash
paos agent
```

### 4. Interaction Example
In the `paos agent` CLI, input:
> "Look at what is on the table, then grasp that apple for me."

You will see the action execution in the simulation logs in Terminal 1, and receive completion confirmation from the Agent in Terminal 2.

5. More Information

[User Manual](docs/user_manual/README.md): Operations guide for end users, integrators, and demo operators.

[Development Guide](docs/user_development_guide/README.md): Layered guide for secondary developers, hardware integrators, plugin authors, and maintainers.

## 📁 Project Structure

```text
Physical Agent Operating System/
├── PhyAgentOS/                # Track A: Software Brain Core
│   ├── agent/              # Agent Logic (Planner, Critic)
│   ├── templates/          # Workspace Markdown Templates
│   └── ...
├── hal/                    # Track B: Hardware HAL & Simulation
│   ├── hal_watchdog.py     # Hardware Watchdog Daemon
│   └── simulation/         # Simulation Environment Code
├── scripts/                # External HAL Plugin Deployment
│   └── deploy_rekep_real_plugin.py
├── workspace/              # Single-instance Runtime Workspace
│   ├── EMBODIED.md         # Runtime Robot Profile
│   ├── ENVIRONMENT.md      # Current Scene-Graph
│   ├── ACTION.md           # Pending Action Commands
│   ├── LESSONS.md          # Failure Experience Records
│   └── SKILL.md            # Successful Workflow SOP
├── workspaces/             # Fleet Topology
│   ├── shared/             # Agent Workspace & Global ENVIRONMENT.md
│   ├── go2_edu_001/        # Robot-local ACTION.md / EMBODIED.md
│   └── ...
├── docs/                   # Project Documentation
│   ├── PLAN.md             # Detailed Implementation Plan
│   └── PROJ.md             # Project Whitepaper & Architecture
├── README.md               # English Documentation
└── README_zh.md            # Chinese Documentation
```

## 🗺️ Roadmap

- **Phase 1**: Desktop Loop & Markdown Protocol Establishment.
    - [x] v0.0.1: Framework Design & Initialization
    - [x] v0.0.2: Embodied Skill Plugin Deployment & Invocation Design
    - [x] v0.0.3: Visual Decoupling + Grasping Pipeline (SAM3 & ReKep)
    - [x] v0.0.4: Atomic Action-based VLN Pipeline (SAM3)
    - [x] v0.0.5: Multi-Agent Protocol Design
    - [ ] v0.0.6: Long-horizon Task Decomposition, Orchestration & Execution
    - [ ] v0.0.7: IoT Device Integration (e.g., XiaoZhi)
- **Phase 2**: Multi-Embodiment Coordination & Multi-modal Memory.
- **Phase 3**: Constraint Solving & High-level Heterogeneous Coordination.

## 🛠️ Supported Devices

PhyAgentOS supports various embodiment types through the HAL (Hardware Abstraction Layer) protocol.

| Embodiment Type | Robot | Status | Remarks |
| :--- | :--- | :--- | :--- |
| **Desktop Robot Arm** | AgileX PIPER | 🟢 Verified | Full-chain verified with ReKep & SAM3 |
| **Composite Robot** | AgileX PIPER + Unitree Go2 | 🟡 Partial |  locomotion adaptation in progress |
| **Desktop Robot Arm** | Dobot Nova 2 | 🟢 Verified | ReKep deployment verified |
| **Quadruped Robot** | Unitree Go2 | 🟡 Partial | Currently supports mobility and semantic navigation |
| **Dual-Arm Control** | XLeRobot | 🟢 Verified |  Deployment and dual-arm capture have now been achieved |
| **IoT Device** | XiaoZhi (ESP32) | 🟡 Partial | Currently supports voice dialogue interaction |
| **Industrial Robot** | Franka Research 3 | 🟢 Verified | Visual Reasoning and capture have been achieved |
| **Edu Robot** | Hiwonder Series | 🔴 Unsupported | Awaiting driver plugin development |
| **General Environment** | Built-in Simulator | 🟢 Verified | Lightweight simulation based on disk mapping |

> **Note**: PhyAgentOS is designed with a plugin architecture. Any hardware that supports a Python control interface can be quickly integrated via `hal/drivers/`. A community plugin template is available at `docs/user_development_guide/PLUGIN_DEVELOPMENT_GUIDE.md`, with the Chinese version at `docs/user_development_guide/PLUGIN_DEVELOPMENT_GUIDE_zh.md`.

## 🤝 Contribute

PRs and Issues are welcome! Please refer to `docs/user_development_guide/README.md` for detailed architecture design and development guidelines.

---

**Special Thanks**: This project is developed based on [nanobot](https://github.com/HKUDS/nanobot), thanks for providing the lightweight Agent framework. Everyone is welcome to go to the [nanobot](https://github.com/HKUDS/nanobot) repository and give it a star!

## Affiliations

<p align="center">
   <img src="docs/imgs/SYSU.png" alt="SYSU" width="150">
   <img src="docs/imgs/Pengcheng.png" alt="HCP" width="150">
   <img src="docs/imgs/HCP.jpg" alt="HCP" width="150">
</p>

We welcome any individual or team to join as a contributor！
