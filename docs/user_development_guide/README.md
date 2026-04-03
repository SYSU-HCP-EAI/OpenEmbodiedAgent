# PhyAgentOS 用户开发指南

> 面向二次开发者、硬件接入者、插件作者与维护者的分层手册。本文档重点解释仓库结构、运行入口、工作区协议、驱动扩展、插件机制、导航/感知相关模块，以及推荐的测试与文档维护方式。

## 目录

- [1. 指南定位与推荐阅读顺序](#1-指南定位与推荐阅读顺序)
- [2. 从开发视角理解系统架构](#2-从开发视角理解系统架构)
- [3. 仓库结构与模块职责](#3-仓库结构与模块职责)
- [4. 配置系统与工作区模型](#4-配置系统与工作区模型)
- [5. 运行入口与开发期调试方式](#5-运行入口与开发期调试方式)
- [6. HAL 驱动开发与内置驱动扩展](#6-hal-驱动开发与内置驱动扩展)
- [7. 动作校验、分发与运行时文件流转](#7-动作校验分发与运行时文件流转)
- [8. 导航、感知与 ROS2 相关模块](#8-导航感知与-ros2-相关模块)
- [9. 外部插件机制与 ReKep 参考路径](#9-外部插件机制与-rekep-参考路径)
- [10. 测试、验证与质量门禁](#10-测试验证与质量门禁)
- [11. 文档维护与贡献建议](#11-文档维护与贡献建议)
- [12. 后续扩展主题（开发中）](#12-后续扩展主题开发中)

## 1. 指南定位与推荐阅读顺序

### 1.1 这份文档解决什么问题

如果你的目标已经不是“把系统跑起来”，而是以下这些方向之一，那么你应该以本文档为主：

- 理解仓库内各模块分工
- 新增或修改内置驱动
- 基于 HAL 接入新机器人
- 开发一个独立插件仓库
- 接入感知、导航、远程控制或 ROS2 相关能力
- 为项目补测试、补文档、补部署说明

如果你只是想先完成安装和基本运行，请先阅读 [USER_MANUAL.md](../user_manual/README.md)。

### 1.2 推荐阅读路径

| 目标 | 建议先读 | 然后继续 |
| --- | --- | --- |
| 想理解运行时通信 | [COMMUNICATION.md](COMMUNICATION.md) | 本文第 2、4、7 章 |
| 想接一个真实机器人插件 | [phyagentos-rekep-plugin-blog-zh.md](REKEP_PLUGIN_DEPLOYMENT_zh.md) | 本文第 9 章 |
| 想按模板开发外部插件 | [PLUGIN_DEVELOPMENT_GUIDE_zh.md](PLUGIN_DEVELOPMENT_GUIDE_zh.md) | 本文第 6、9、10 章 |
| 想快速了解项目整体定位 | [../README_zh.md](../../README_zh.md) | 本文第 2、3 章 |
| 想了解阶段性规划 | [plans/Report.md](../plans/Report.md) | 本文第 12 章 |

## 2. 从开发视角理解系统架构

### 2.1 核心设计：认知与执行解耦

PhyAgentOS 的核心价值不只是“有一个 Agent”，而是将认知层与执行层通过显式协议解耦：

- **Track A：Agent / Planner / Critic / Tooling**
  - 负责自然语言理解、规划、工具调用、动作校验、记忆与编排
- **Track B：HAL / Driver / Watchdog**
  - 负责把动作意图映射到具体机器人或仿真环境，并将运行结果回写
- **中间层协议**
  - 不是 RPC-first，也不是把所有状态藏在对象实例里
  - 而是优先通过工作区中的 Markdown 文件暴露共享状态

这一点是理解整个项目的前提：**很多“接口”本质上不是 Python 函数，而是文件协议与运行时约定。**

### 2.2 为什么工作区文件很重要

在 PhyAgentOS 中，下列文件通常比类图更重要：

- `ENVIRONMENT.md`
- `EMBODIED.md`
- `ACTION.md`
- `LESSONS.md`
- `TASK.md`
- `ORCHESTRATOR.md`
- `ROBOTS.md`（Fleet 模式）

它们共同构成运行时“真实状态面”。如果你只看代码、不看这些文件，很容易误解系统是怎么工作的。文件职责详见 [COMMUNICATION.md](COMMUNICATION.md) 与本文第 4、7 章。

### 2.3 单机模式与 Fleet 模式的开发含义

开发者需要特别清楚两种拓扑的差异：

- **single 模式**
  - 一个工作区即可跑通全链路
  - 适合开发期 smoke test
- **fleet 模式**
  - 一个共享工作区 + 多个机器人工作区
  - Agent 在共享工作区上做规划
  - Watchdog 在机器人工作区中消费本机 `ACTION.md`
  - Critic 会针对目标机器人的运行时 `EMBODIED.md` 做校验

这意味着：任何涉及具身动作、导航、连接状态或多机器人任务的功能，都应该显式考虑 single / fleet 两种运行语义。

## 3. 仓库结构与模块职责

### 3.1 顶层目录速览

| 路径 | 作用 |
| --- | --- |
| [../PhyAgentOS](../../PhyAgentOS) | Track A 主体：Agent、CLI、Provider、Channel、Template、Session、Heartbeat 等 |
| [../hal](../../hal) | Track B 主体：驱动、Watchdog、感知、导航、ROS2、仿真 |
| [../bridge](../../bridge) | 网桥相关代码，偏服务化/渠道接入配套 |
| [docs](..) | 项目文档与说明 |
| [../examples](../../examples) | 驱动配置样例 |
| [../scripts](../../scripts) | 部署脚本，如插件安装脚本 |
| [../tests](../../tests) | 自动化测试 |

### 3.2 Track A 的关键模块

| 路径 | 说明 |
| --- | --- |
| [../PhyAgentOS/cli/commands.py](../../PhyAgentOS/cli/commands.py) | CLI 入口：`paos onboard`、`paos agent`、`paos gateway` |
| [../PhyAgentOS/agent](../../PhyAgentOS/agent) | AgentLoop、上下文、记忆、子代理与工具注册 |
| [../PhyAgentOS/providers](../../PhyAgentOS/providers) | LLM provider 适配层 |
| [../PhyAgentOS/channels](../../PhyAgentOS/channels) | 外部渠道接入，如 Telegram、Feishu、Slack 等 |
| [../PhyAgentOS/heartbeat](../../PhyAgentOS/heartbeat) | 心跳/周期任务调度入口 |
| [../PhyAgentOS/cron](../../PhyAgentOS/cron) | 定时任务服务 |
| [../PhyAgentOS/templates](../../PhyAgentOS/templates) | 工作区模板文件 |

### 3.3 Track B 的关键模块

| 路径 | 说明 |
| --- | --- |
| [../hal/hal_watchdog.py](../../hal/hal_watchdog.py) | HAL Watchdog 主入口 |
| [../hal/base_driver.py](../../hal/base_driver.py) | 所有驱动都要遵守的最小抽象接口 |
| [../hal/drivers](../../hal/drivers) | 内置 driver 注册与实现 |
| [../hal/plugins.py](../../hal/plugins.py) | 外部插件注册、解析与激活 |
| [../hal/profiles](../../hal/profiles) | 机器人/仿真实体 profile 来源 |
| [../hal/navigation](../../hal/navigation) | 导航后端与目标导航相关逻辑 |
| [../hal/perception](../../hal/perception) | 感知、几何/语义融合与环境写回 |
| [../hal/ros2](../../hal/ros2) | ROS2 bridge 与 adapter |
| [../hal/simulation](../../hal/simulation) | 仿真场景与场景文档读写 |

### 3.4 模板、Profile 与运行时文件的区别

开发中最容易混淆的三个概念是：

1. **模板（templates）**
   - 用来定义文件结构与建议字段
   - 例如 [../PhyAgentOS/templates/EMBODIED.md](../../PhyAgentOS/templates/EMBODIED.md)、[../PhyAgentOS/templates/ENVIRONMENT.md](../../PhyAgentOS/templates/ENVIRONMENT.md)
2. **Profile（hal/profiles）**
   - 某类机器人的静态能力声明
   - 例如 [../hal/profiles/go2_edu.md](../../hal/profiles/go2_edu.md)、[../hal/profiles/simulation.md](../../hal/profiles/simulation.md)
3. **运行时文件（workspace / workspaces）**
   - 真正被 Agent、Critic、Watchdog 读写的状态面
   - 例如实际运行中的 `EMBODIED.md`、`ENVIRONMENT.md`

简而言之：**模板定义结构，Profile 提供实例类型说明，运行时文件承载真实状态。**

## 4. 配置系统与工作区模型

### 4.1 配置根对象的关注重点

Phy's 配置根对象定义在 [../PhyAgentOS/config/schema.py](../../PhyAgentOS/config/schema.py)。开发时最值得关注的配置域包括：

- `agents.defaults.*`
- `providers.*`
- `gateway.*`
- `tools.*`
- `embodiments.*`

其中 `embodiments.mode` 决定 single / fleet 模式，而 `embodiments.instances` 描述 fleet 中的机器人实例清单。

### 4.2 single 与 fleet 的最小配置差异

#### single 模式示意

```json
{
  "agents": {
    "defaults": {
      "workspace": "~/.PhyAgentOS/workspace"
    }
  },
  "embodiments": {
    "mode": "single"
  }
}
```

#### fleet 模式示意

```json
{
  "embodiments": {
    "mode": "fleet",
    "shared_workspace": "~/.PhyAgentOS/workspaces/shared",
    "instances": [
      {
        "robot_id": "go2_edu_001",
        "driver": "go2_edu",
        "workspace": "~/.PhyAgentOS/workspaces/go2_edu_001"
      }
    ]
  }
}
```

### 4.3 `paos onboard` 对开发者意味着什么

`paos onboard` 不只是“首次安装命令”，它还是：

- 模板同步入口
- 配置升级入口
- fleet 布局准备入口
- 本地开发环境纠偏入口

当你修改了模板、升级了配置结构、切换了运行模式时，都应该重新执行一次：

```bash
paos onboard
```

### 4.4 工作区文件生命周期

一个开发者应当掌握如下生命周期：

1. `paos onboard` 准备模板/目录
2. Watchdog 启动时把 profile 安装为运行时 `EMBODIED.md`
3. Watchdog 根据驱动健康状态回写 `ENVIRONMENT.md`
4. Agent 根据环境状态规划动作
5. Critic 根据 `EMBODIED.md` 与 `ENVIRONMENT.md` 验证动作
6. 通过验证后将动作写入 `ACTION.md`
7. Watchdog 消费动作并再次更新环境
8. 如果动作被拒绝，则将经验写入 `LESSONS.md`

## 5. 运行入口与开发期调试方式

### 5.1 CLI 的三个核心入口

#### `paos onboard`

用途：初始化配置与工作区。

#### `paos agent`

用途：本地交互、单轮消息调用、CLI 调试。

常用方式：

```bash
paos agent
paos agent -m "look around the room"
```

#### `paos gateway`

用途：长期在线服务、渠道接入、心跳与定时任务联动。

```bash
paos gateway
```

### 5.2 HAL Watchdog 是如何工作的

入口位于 [../hal/hal_watchdog.py](../../hal/hal_watchdog.py)。从开发角度，它做了这些关键动作：

- 解析 `--driver` / `--robot-id` / `--workspace` / `--driver-config`
- 加载 driver
- 安装 profile 到运行时 `EMBODIED.md`
- 刷新连接与健康状态
- 轮询 `ACTION.md`
- 调用 `driver.execute_action()`
- 用 `driver.get_scene()` 与 `get_runtime_state()` 回写 `ENVIRONMENT.md`

### 5.3 开发期最常用的调试组合

#### 调试 Agent + 仿真

```bash
python hal/hal_watchdog.py --driver simulation
paos agent
```

#### 调试指定驱动配置

```bash
python hal/hal_watchdog.py \
  --driver go2_edu \
  --driver-config examples/go2_driver_config.json
```

#### 调试 Fleet 某台机器人

```bash
python hal/hal_watchdog.py \
  --robot-id go2_edu_001 \
  --driver-config examples/go2_driver_config.json
```

### 5.4 开发中该看哪些日志与文件

排障时建议同时观察：

- Watchdog 终端输出
- Agent 终端输出
- `ACTION.md`
- `ENVIRONMENT.md`
- `LESSONS.md`
- Fleet 模式下的 `ROBOTS.md`

## 6. HAL 驱动开发与内置驱动扩展

### 6.1 所有驱动都必须满足的最小接口

驱动基类定义在 [../hal/base_driver.py](../../hal/base_driver.py)。最小实现必须覆盖四个抽象方法：

- `get_profile_path()`
- `load_scene(scene)`
- `execute_action(action_type, params)`
- `get_scene()`

此外，以下方法虽然不是抽象方法，但在真实机器人场景中通常非常重要：

- `connect()`
- `disconnect()`
- `is_connected()`
- `health_check()`
- `get_runtime_state()`

### 6.2 一个内置 driver 开发的最小流程

1. 在 [../hal/drivers](../../hal/drivers) 中新增 driver 实现文件
2. 继承 `BaseDriver`
3. 在 [../hal/profiles](../../hal/profiles) 中新增 profile
4. 在 [../hal/drivers/__init__.py](../../hal/drivers/__init__.py) 的 `DRIVER_REGISTRY` 中注册
5. 用 `hal/hal_watchdog.py` 直接启动验证
6. 用 `paos agent` 做全链路联调

### 6.3 什么时候应该改内置 driver，什么时候应该做插件

适合直接修改主仓库的场景：

- 修复现有驱动的 bug
- 增强内置仿真或已有支持设备
- 改动对主仓库用户具有普适性

更适合做外部插件的场景：

- 依赖较重的第三方 SDK
- 厂商私有运行时
- 真实机器人部署逻辑复杂
- 希望独立发版、独立维护依赖

### 6.4 profile 应该写什么，不该写什么

建议把 profile 当成“给 Critic 和 Agent 看的能力说明书”，而不只是给开发者看的备注。一般应包含：

- 身份与类型
- 传感器能力
- 支持动作表
- 物理约束
- 连接方式
- 运行时协议映射

可参考：

- [../hal/profiles/simulation.md](../../hal/profiles/simulation.md)
- [../hal/profiles/go2_edu.md](../../hal/profiles/go2_edu.md)
- [../hal/profiles/xlerobot_2wheels_remote.md](../../hal/profiles/xlerobot_2wheels_remote.md)
- [../hal/profiles/desktop_pet.md](../../hal/profiles/desktop_pet.md)

### 6.5 `driver-config` 的推荐用法

`hal_watchdog.py` 支持通过 `--driver-config` 传入一个 JSON object，并原样透传给目标 driver 构造器。这是一种非常实用的扩展机制，因为它可以：

- 避免频繁改 Watchdog CLI
- 保持每个 driver 自己定义初始化参数
- 让配置示例以文件形式沉淀在仓库中

参考示例：

- [../examples/go2_driver_config.json](../../examples/go2_driver_config.json)
- [../examples/xlerobot_2wheels_remote.driver.json](../../examples/xlerobot_2wheels_remote.driver.json)

## 7. 动作校验、分发与运行时文件流转

### 7.1 具身动作并不是“直接执行”

在 PhyAgentOS 里，动作通常要经过下面的路径：

1. Agent 形成动作意图
2. `EmbodiedActionTool` 做 Critic 校验
3. 校验通过后写入目标 `ACTION.md`
4. Watchdog 消费动作并执行
5. 结果被回写到环境状态文件

因此，问题排查时要区分：

- 是**动作生成有问题**
- 还是**Critic 校验拒绝**
- 还是**Watchdog 执行失败**
- 还是**执行成功但环境未回写**

### 7.2 `EmbodiedActionTool` 的职责

核心逻辑位于 [../PhyAgentOS/agent/tools/embodied.py](../../PhyAgentOS/agent/tools/embodied.py)。开发者应关注它的几个关键职责：

- 在 fleet 模式下解析目标 `robot_id`
- 为目标机器人定位 `EMBODIED.md`、`ENVIRONMENT.md`、`ACTION.md`
- 把动作草案、环境状态、能力声明交给 Critic
- 校验通过时写入 `ACTION.md`
- 校验失败时记录到 `LESSONS.md`

这也是为什么 profile 与环境文件的表达质量会直接影响系统表现。

### 7.3 `ACTION.md` 的协议约定

Watchdog 默认从 `ACTION.md` 中提取第一个 JSON 代码块。最小格式通常类似：

```json
{
  "action_type": "move_to",
  "parameters": {
    "x": 10,
    "y": 20,
    "z": 5
  },
  "status": "pending"
}
```

开发自定义工具或外部插件时，必须保持这一约定，否则 Watchdog 无法解析。

### 7.4 导航类动作的特殊点

目标导航工具位于 [../PhyAgentOS/agent/tools/target_navigation.py](../../PhyAgentOS/agent/tools/target_navigation.py)。它会把更高层的“朝某个目标标签移动”的需求，转换为底层 `target_navigation` 动作并交给 `EmbodiedActionTool`。

对导航类能力进行扩展时，建议同时检查：

- profile 是否声明了该动作
- `ENVIRONMENT.md` 是否包含必要的目标/地图/状态字段
- driver 是否在 `get_runtime_state()` 中回写了足够的导航状态

## 8. 导航、感知与 ROS2 相关模块

### 8.1 感知模块的定位

感知相关模块主要在 [../hal/perception](../../hal/perception)。当前结构已经体现出几个层次：

- [../hal/perception/service.py](../../hal/perception/service.py)：服务化入口
- [../hal/perception/geometry_pipeline.py](../../hal/perception/geometry_pipeline.py)：几何处理
- [../hal/perception/segmentation_pipeline.py](../../hal/perception/segmentation_pipeline.py)：语义分割
- [../hal/perception/fusion_pipeline.py](../../hal/perception/fusion_pipeline.py)：多源融合
- [../hal/perception/environment_writer.py](../../hal/perception/environment_writer.py)：环境文件写回

如果你在开发新的感知链路，建议把“感知处理”和“环境落盘”明确分层，而不是把所有逻辑塞进 driver。

### 8.2 导航相关模块的定位

导航相关代码主要在 [../hal/navigation](../../hal/navigation)。从当前仓库形态看，可以将其理解为：

- 上层工具发起导航意图
- 导航 backend 负责目标解析、执行或状态查询
- 运行状态通过 `ENVIRONMENT.md` 中的 `robots.<robot_id>.nav_state` 暴露

对导航能力做扩展时，最重要的不是只让机器人“动起来”，而是让**状态可见、可回写、可解释**。

### 8.3 ROS2 适配入口

ROS2 相关目录位于 [../hal/ros2](../../hal/ros2)。其中：

- [../hal/ros2/bridge.py](../../hal/ros2/bridge.py) 负责 bridge 逻辑
- [../hal/ros2/messages.py](../../hal/ros2/messages.py) 定义消息结构
- [../hal/ros2/adapters](../../hal/ros2/adapters) 放置具体 adapter

如果你要接入新的 ROS2 topic / sensor / control 通道，建议优先按 adapter 维度扩展，而不是在单一 driver 内部堆砌临时逻辑。

## 9. 外部插件机制与 ReKep 参考路径

### 9.1 插件发现与注册机制

外部插件机制的核心在 [../hal/plugins.py](../../hal/plugins.py)。其基本流程是：

1. 插件仓库提供 `PhyAgentOS_plugin.toml`
2. 部署脚本 clone 或复制插件仓库
3. 主仓库读取 manifest 并写入本地插件 registry
4. 当内置 `DRIVER_REGISTRY` 找不到目标 driver 时，再从外部 registry 动态解析

对应内置 driver 加载逻辑可参考 [../hal/drivers/__init__.py](../../hal/drivers/__init__.py)。

### 9.2 为什么 ReKep 是一个很好的参考实现

ReKep 真实机器人能力没有硬编码在主仓库，而是通过独立插件仓库接入。这种做法的优点是：

- 主仓库保持轻量
- 复杂依赖被隔离到插件仓库
- 真实机器人部署逻辑可独立迭代
- 社区维护者可以更自然地做独立发版

### 9.3 ReKep 插件在主仓库中的接入入口

主仓库中的部署脚本是 [../scripts/deploy_rekep_real_plugin.py](../../scripts/deploy_rekep_real_plugin.py)。它负责：

- 解析 `--repo-url` / `--ref`
- clone 或同步插件仓库
- 安装 `requirements`
- 调用注册逻辑写入本地 registry

快速安装示例：

```bash
python scripts/deploy_rekep_real_plugin.py \
  --repo-url https://github.com/baiyu858/PhyAgentOS-rekep-real-plugin.git
```

### 9.4 插件作者应该继续读什么

- 想快速部署 ReKep：阅读 [phyagentos-rekep-plugin-blog-zh.md](REKEP_PLUGIN_DEPLOYMENT_zh.md)
- 想照着模板自己写插件：阅读 [PLUGIN_DEVELOPMENT_GUIDE_zh.md](PLUGIN_DEVELOPMENT_GUIDE_zh.md)
- 需要英文版模板：阅读 [PLUGIN_DEVELOPMENT_GUIDE.md](PLUGIN_DEVELOPMENT_GUIDE.md)

## 10. 测试、验证与质量门禁

### 10.1 开发前先决定验证层级

建议将验证分成四层：

1. **纯 Python 单测**：接口、配置、注册、解析逻辑
2. **driver 本地 smoke test**：直接启动 Watchdog
3. **runtime 层 dry-run**：对真实插件或远程运行时做预检
4. **Agent 全链路联调**：让 Agent -> Critic -> ACTION -> Watchdog -> ENVIRONMENT 完整走通

### 10.2 主仓库中值得参考的测试

| 路径 | 适合参考的主题 |
| --- | --- |
| [../tests/test_hal_external_plugins.py](../../tests/test_hal_external_plugins.py) | 插件注册与外部 driver 解析 |
| [../tests/test_hal_base_driver.py](../../tests/test_hal_base_driver.py) | driver 基础契约 |
| [../tests/test_hal_watchdog_driver_config.py](../../tests/test_hal_watchdog_driver_config.py) | `driver-config` 透传 |
| [../tests/test_go2_navigation_stack.py](../../tests/test_go2_navigation_stack.py) | Go2 导航栈相关能力 |
| [../tests/test_perception_service.py](../../tests/test_perception_service.py) | 感知服务 |
| [../tests/test_commands.py](../../tests/test_commands.py) | CLI 命令 |
| [../tests/test_fleet_watchdog.py](../../tests/test_fleet_watchdog.py) | Fleet Watchdog 相关流程 |

### 10.3 最小测试命令建议

开发期间至少建议执行：

```bash
pytest tests/
```

如果你只改了某一块，建议先跑对应文件，例如：

```bash
pytest tests/test_hal_external_plugins.py
pytest tests/test_go2_navigation_stack.py
```

### 10.4 对真实机器人/插件更实用的验证建议

对于外部插件，尤其是真实机器人场景，建议遵循这个顺序：

1. `preflight`
2. dry-run
3. Watchdog 直连验证
4. Agent 全链路验证

这一流程在 [phyagentos-rekep-plugin-blog-zh.md](REKEP_PLUGIN_DEPLOYMENT_zh.md) 与 [PLUGIN_DEVELOPMENT_GUIDE_zh.md](PLUGIN_DEVELOPMENT_GUIDE_zh.md) 中都有较完整体现。

## 11. 文档维护与贡献建议

### 11.1 文档应该如何分层

当前建议采用如下分层：

- **README**：项目总览与入口
- **用户手册**：面向使用者的运行说明
- **开发指南**：面向开发者的扩展说明
- **专题文档**：通信架构、插件开发、具体机器人接入、计划文档

这样做的好处是：

- 首页不至于过载
- 章节边界更清晰
- 专题文档可独立迭代
- 未来可以自然迁移到多级文档站点

### 11.2 什么时候应该单独拆文档

如果某个主题同时满足以下两个条件，建议拆出独立文档：

- 超过“快速说明”范围，需要包含背景、部署、排障、示例、FAQ
- 有自己稳定的读者群，如插件作者、ROS2 开发者、运维人员

### 11.3 贡献文档时的实用建议

- 先说明“这份文档服务谁”
- 在开头放目录
- 给出“先读什么、后读什么”的路线
- 示例命令尽量与仓库实际路径一致
- 避免文档仍停留在抽象概念层，不给落地入口
- 对尚未完成的章节显式标注“开发中”

## 12. 后续扩展主题（开发中）

以下主题建议后续继续独立展开：

### 12.1 Fleet 编排与多 Agent 协同细节（开发中）

计划补充：`TASK.md`、`ORCHESTRATOR.md`、调度边界、机器人选择与冲突处理策略。

### 12.2 感知部署流水线手册（开发中）

计划补充：相机/LiDAR 接入、分割模型依赖、场景图构建与写回协议。

### 12.3 ROS2 适配开发手册（开发中）

计划补充：adapter 模板、topic 映射、TF/地图约定、导航控制链路。

### 12.4 渠道与服务化部署手册（开发中）

计划补充：`paos gateway`、bridge、channel 配置、消息路由与生产部署建议。

### 12.5 示例驱动模板与脚手架（开发中）

计划补充：最小 driver、最小 profile、最小测试、最小插件清单的一键模板。

---

当你从“理解系统”进入“修改系统”阶段时，建议把 [COMMUNICATION.md](COMMUNICATION.md)、[PLUGIN_DEVELOPMENT_GUIDE_zh.md](PLUGIN_DEVELOPMENT_GUIDE_zh.md) 与本文配合阅读，形成架构、实现与扩展三层视图。
