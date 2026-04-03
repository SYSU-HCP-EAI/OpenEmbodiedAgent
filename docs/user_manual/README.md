# PhyAgentOS 用户手册

> 面向使用者、集成者与演示操作者的运行手册。本文档基于当前仓库结构整理，重点覆盖单机模式、Fleet 多机器人模式、驱动启动方式、工作区文件职责，以及真实机器人插件接入路径。

## 目录

- [1. 手册定位与阅读路径](#1-手册定位与阅读路径)
- [2. 先理解系统是如何工作的](#2-先理解系统是如何工作的)
- [3. 安装与环境准备](#3-安装与环境准备)
- [4. 首次初始化：配置文件与工作区](#4-首次初始化配置文件与工作区)
- [5. 单机模式快速开始](#5-单机模式快速开始)
- [6. 常用运行方式总览](#6-常用运行方式总览)
- [7. 内置驱动、示例配置与真实机器人插件](#7-内置驱动示例配置与真实机器人插件)
- [8. Fleet 模式：多机器人协同运行](#8-fleet-模式多机器人协同运行)
- [9. 运行时 Markdown 文件职责](#9-运行时-markdown-文件职责)
- [10. 常见交互与操作示例](#10-常见交互与操作示例)
- [11. 常见问题与排查建议](#11-常见问题与排查建议)
- [12. 未来章节（开发中）](#12-未来章节开发中)

## 1. 手册定位与阅读路径

### 1.1 这份手册适合谁

本文档主要面向以下读者：

- 希望快速跑通 PhyAgentOS 的首次使用者
- 需要用命令行或网关方式操作 Agent 的集成使用者
- 需要启动仿真、Go2、远程底盘或真实机器人插件的演示操作者
- 需要理解运行时工作区文件如何变化的调试人员

如果你要进行二次开发、编写驱动、开发插件或研究系统内部架构，请优先继续阅读 [USER_DEVELOPMENT_GUIDE.md](../user_development_guide/README.md)。

### 1.2 文档地图

| 文档 | 用途 | 何时阅读 |
| --- | --- | --- |
| [../README_zh.md](../../README_zh.md) | 项目总览、特性与展示入口 | 第一次了解项目时 |
| [COMMUNICATION.md](../user_development_guide/COMMUNICATION.md) | 运行时通信架构、工作区拓扑与文件职责说明 | 想理解 Track A / Track B 与共享状态时 |
| [USER_DEVELOPMENT_GUIDE.md](../user_development_guide/README.md) | 面向开发者的二次开发与扩展指南 | 需要改代码、接新硬件或做插件时 |
| [phyagentos-rekep-plugin-blog-zh.md](../user_development_guide/REKEP_PLUGIN_DEPLOYMENT_zh.md) | ReKep 真实机器人插件的一键部署、联调与执行说明 | 想尽快接入 `rekep_real` 时 |
| [PLUGIN_DEVELOPMENT_GUIDE_zh.md](../user_development_guide/PLUGIN_DEVELOPMENT_GUIDE_zh.md) | 外部插件开发模板（中文） | 想按模板开发独立插件仓库时 |
| [PLUGIN_DEVELOPMENT_GUIDE.md](../user_development_guide/PLUGIN_DEVELOPMENT_GUIDE.md) | 外部插件开发模板（英文） | 需要英文材料或对外发布时 |
| [plans/Report.md](../plans/Report.md) | 项目规划与报告类文档 | 想了解后续方向时 |
| [changelog/v0.0.5.md](../changelog/v0.0.5.md) | 历史变更记录参考 | 想了解某阶段设计演进时 |

## 2. 先理解系统是如何工作的

### 2.1 双轨结构：Track A 与 Track B

PhyAgentOS 不是“LLM 直接调用硬件接口”的黑盒控制系统，而是一个显式解耦的双轨运行架构：

- **Track A（Agent / 大脑）**
  - 负责理解用户输入、规划动作、调用工具、做 Critic 校验
  - 通常通过 [`paos agent`](../../PhyAgentOS/cli/commands.py) 或 [`paos gateway`](../../PhyAgentOS/cli/commands.py) 启动
- **Track B（HAL / 小脑）**
  - 负责读取动作指令、调用驱动、执行动作、回写环境状态
  - 通常通过 [`hal/hal_watchdog.py`](../../hal/hal_watchdog.py) 启动

两者之间的共享状态优先通过工作区中的 Markdown 文件表达，而不是跨层直接 Python 调用。更完整的通信解释见 [COMMUNICATION.md](../user_development_guide/COMMUNICATION.md)。

### 2.2 单机模式与 Fleet 模式

PhyAgentOS 有两种典型运行拓扑：

1. **单机模式（single）**
   - 默认工作区通常位于 `~/.PhyAgentOS/workspace`
   - 适合单个机器人或仿真快速验证
2. **Fleet 模式（fleet）**
   - 共享工作区通常位于 `~/.PhyAgentOS/workspaces/shared`
   - 每个机器人实例都有自己的工作区
   - 适合异构多机器人协同

### 2.3 一次典型运行会发生什么

一个标准闭环通常如下：

1. 运行 `paos onboard` 初始化配置与工作区
2. 启动一个或多个 Watchdog
3. Watchdog 将对应机器人的 profile 安装为运行时 `EMBODIED.md`
4. 启动 `paos agent` 或 `paos gateway`
5. 用户输入自然语言任务
6. Agent 读取 `ENVIRONMENT.md` 等工作区文件进行规划
7. Critic 结合 `EMBODIED.md` 校验动作是否安全可行
8. 校验通过的动作被写入 `ACTION.md`
9. Watchdog 读取 `ACTION.md`，调用 driver 执行
10. Watchdog 将最新环境、连接状态、导航状态回写到 `ENVIRONMENT.md`

## 3. 安装与环境准备

### 3.1 基础要求

建议至少准备以下环境：

- Python 3.11 或更高版本
- Git
- 可访问的 LLM 提供方 API 或兼容服务
- 如需仿真：建议额外准备 `pybullet`
- 如需桥接、前端或部分渠道能力：建议准备 Node.js 18+

### 3.2 克隆与安装

如果你使用主仓库，可以直接执行：

```bash
git clone https://github.com/SYSU-HCP-EAI/PhyAgentOS.git
cd PhyAgentOS
pip install -e .
```

如果你准备使用仿真机械臂，通常还需要：

```bash
pip install pybullet
```

### 3.3 安装完成后你会得到什么

安装后，CLI 入口 `paos` 会来自项目的 Python 包入口。常见使用命令包括：

- `paos onboard`
- `paos agent`
- `paos gateway`
- `python hal/hal_watchdog.py --driver simulation`

## 4. 首次初始化：配置文件与工作区

### 4.1 执行初始化

首次使用请先运行：

```bash
paos onboard
```

该命令会完成以下工作：

- 创建或刷新 `~/.PhyAgentOS/config.json`
- 在单机模式下准备默认工作区
- 在 Fleet 模式下准备共享工作区与机器人工作区布局
- 同步工作区模板文件

如果你升级过项目版本，重新执行一次 `paos onboard` 也很有价值，因为它会刷新配置模板并补充新字段。

### 4.2 最少需要配置什么

最小可用配置通常至少包含：

- `agents.defaults.model`
- 某个 `providers.*.api_key`

一个最小示意配置如下：

```json
{
  "agents": {
    "defaults": {
      "model": "openrouter/openai/gpt-4o-mini"
    }
  },
  "providers": {
    "openrouter": {
      "api_key": "YOUR_API_KEY"
    }
  }
}
```

如果你使用自定义 OpenAI 兼容接口，可以改为配置 `providers.custom.api_base` 与对应 `api_key`。

### 4.3 默认工作区在哪里

- **单机模式**：`~/.PhyAgentOS/workspace`
- **Fleet 模式共享工作区**：`~/.PhyAgentOS/workspaces/shared`
- **Fleet 模式机器人工作区**：`~/.PhyAgentOS/workspaces/<robot_id>`

## 5. 单机模式快速开始

以下流程适合第一次跑通系统。

### 5.1 第一步：初始化工作区

```bash
paos onboard
```

### 5.2 第二步：启动 HAL Watchdog

打开终端 A，运行仿真驱动：

```bash
python hal/hal_watchdog.py --driver simulation
```

该命令会：

- 加载 `simulation` 驱动
- 将对应 profile 复制为运行时 `EMBODIED.md`
- 定期轮询 `ACTION.md`
- 执行动作并更新 `ENVIRONMENT.md`

### 5.3 第三步：启动 Agent

打开终端 B，运行：

```bash
paos agent
```

进入交互模式后，你可以直接输入自然语言任务。

如果你只想单次调用，也可以使用：

```bash
paos agent -m "看看桌面上有什么物体"
```

### 5.4 第四步：观察文件变化

第一次调试时，建议同时关注这些文件：

- `EMBODIED.md`：Watchdog 启动后是否已正确安装机器人 profile
- `ACTION.md`：Agent 是否成功写入待执行动作
- `ENVIRONMENT.md`：动作执行后环境状态是否变化
- `LESSONS.md`：动作被 Critic 拒绝时是否记录失败经验

## 6. 常用运行方式总览

### 6.1 命令速查表

| 目标 | 命令 | 说明 |
| --- | --- | --- |
| 初始化配置与工作区 | `paos onboard` | 首次运行或升级后刷新模板 |
| 交互式命令行对话 | `paos agent` | 适合本地直接调试 |
| 单轮发送消息 | `paos agent -m "..."` | 适合脚本化或快速 smoke test |
| 启动网关 | `paos gateway` | 适合渠道接入、定时任务与心跳服务 |
| 启动单机 Watchdog | `python hal/hal_watchdog.py --driver <name>` | 适合单机器人或仿真 |
| 指定工作区运行 Watchdog | `python hal/hal_watchdog.py --workspace <path> --driver <name>` | 单机模式下手动切换工作区 |
| 为驱动传入 JSON 配置 | `python hal/hal_watchdog.py --driver <name> --driver-config <file>` | 透明透传给 driver 构造器 |

### 6.2 什么时候用 `paos agent`

`paos agent` 适合：

- 本地 CLI 调试
- 快速验证工作区、Agent 和 Watchdog 是否打通
- 开发期间观察工具调用与 Critic 行为

### 6.3 什么时候用 `paos gateway`

`paos gateway` 更适合：

- 接入聊天渠道
- 使用心跳服务、定时任务与会话管理
- 将 Agent 放到“长期在线服务”形态运行

如果配置中没有启用渠道，`paos gateway` 仍可启动，但它更适合作为服务化入口而非本地交互入口。

## 7. 内置驱动、示例配置与真实机器人插件

### 7.1 当前仓库中常见的启动目标

| 类型 | 驱动名 | 状态 | 说明 |
| --- | --- | --- | --- |
| 内置仿真 | `simulation` | 可直接使用 | 最适合首次跑通 |
| 内置移动机器人 | `go2_edu` | 需准备真实配置 | 适合导航、定位与目标导航相关实验 |
| 内置远程底盘 | `xlerobot_2wheels_remote` | 需准备远程主机配置 | 通过 ZMQ 控制远程底盘 |
| 外部插件 | `rekep_real` | 需先安装插件 | 适合真实机械臂 ReKep 运行 |

### 7.2 Go2 示例

仓库已提供一个驱动配置示例：[../examples/go2_driver_config.json](../../examples/go2_driver_config.json)。

典型启动方式：

```bash
python hal/hal_watchdog.py \
  --driver go2_edu \
  --driver-config examples/go2_driver_config.json
```

这个 JSON 配置会透传给 Go2 驱动，用于远程 ROS2、视频、状态流和运动后端初始化。

### 7.3 XLerobot 远程底盘示例

仓库也提供了远程底盘配置示例：[../examples/xlerobot_2wheels_remote.driver.json](../../examples/xlerobot_2wheels_remote.driver.json)。

典型启动方式：

```bash
python hal/hal_watchdog.py \
  --driver xlerobot_2wheels_remote \
  --driver-config examples/xlerobot_2wheels_remote.driver.json
```

### 7.4 ReKep 真实机器人插件

`rekep_real` 不是主仓库内置驱动，而是通过外部插件仓库接入。推荐流程如下：

1. 安装主仓库
2. 执行插件部署脚本
3. 启动 Watchdog 时选择 `rekep_real`

示例：

```bash
python scripts/deploy_rekep_real_plugin.py \
  --repo-url https://github.com/baiyu858/PhyAgentOS-rekep-real-plugin.git

python hal/hal_watchdog.py --driver rekep_real
```

与该插件相关的详细文档请继续阅读：

- [phyagentos-rekep-plugin-blog-zh.md](../user_development_guide/REKEP_PLUGIN_DEPLOYMENT_zh.md)
- [PLUGIN_DEVELOPMENT_GUIDE_zh.md](../user_development_guide/PLUGIN_DEVELOPMENT_GUIDE_zh.md)

## 8. Fleet 模式：多机器人协同运行

### 8.1 何时使用 Fleet 模式

当你需要：

- 让一个 Agent 面向多个机器人实例协同规划
- 将共享环境与机器人私有动作队列分开维护
- 明确每台机器人的独立 `EMBODIED.md` 与 `ACTION.md`

就应该启用 Fleet 模式。

### 8.2 最小 Fleet 配置示意

你可以在 `~/.PhyAgentOS/config.json` 中配置如下结构：

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
      },
      {
        "robot_id": "xlerobot_lab_001",
        "driver": "xlerobot_2wheels_remote",
        "workspace": "~/.PhyAgentOS/workspaces/xlerobot_lab_001"
      }
    ]
  }
}
```

然后重新执行：

```bash
paos onboard
```

### 8.3 Fleet 模式的启动顺序

推荐顺序如下：

1. 配置 `embodiments.mode = "fleet"`
2. 运行 `paos onboard`
3. 为每个机器人实例启动一个 Watchdog
4. 再启动一个 `paos agent` 或 `paos gateway`

例如：

```bash
python hal/hal_watchdog.py \
  --robot-id go2_edu_001 \
  --driver-config examples/go2_driver_config.json

python hal/hal_watchdog.py \
  --robot-id xlerobot_lab_001 \
  --driver-config examples/xlerobot_2wheels_remote.driver.json

paos agent
```

> 在 Fleet 模式下，`--robot-id` 会决定该 Watchdog 绑定哪个机器人实例；对应实例的 `driver` 会从配置中解析。

### 8.4 Fleet 模式下你会看到哪些文件

- `shared/ENVIRONMENT.md`：全局环境状态
- `shared/ROBOTS.md`：机器人目录摘要
- `shared/TASK.md`：多步任务状态
- `shared/ORCHESTRATOR.md`：全局编排状态
- `<robot_id>/EMBODIED.md`：该机器人自己的运行时能力声明
- `<robot_id>/ACTION.md`：该机器人自己的动作队列

### 8.5 Fleet 模式下的交互注意事项

在多机器人模式下，面向具身动作的请求通常需要明确目标机器人。简而言之：

- 用户指令应尽量说明“由哪台机器人执行”
- 如果上层工具需要 `robot_id`，缺失时动作可能被拒绝或无法派发
- `ROBOTS.md` 与 `ENVIRONMENT.md` 是理解当前机器人编队状态的第一入口

## 9. 运行时 Markdown 文件职责

下表是日常使用最值得理解的一组文件：

| 文件 | 典型位置 | 作用 |
| --- | --- | --- |
| `ACTION.md` | 单机工作区或机器人工作区 | 待执行动作队列 |
| `EMBODIED.md` | 单机工作区或机器人工作区 | 当前机器人能力、约束与连接声明 |
| `ENVIRONMENT.md` | 单机工作区或共享工作区 | 当前环境、对象、地图、机器人状态 |
| `LESSONS.md` | 单机工作区或共享工作区 | Critic 拒绝动作后的失败经验记录 |
| `TASK.md` | 单机工作区或共享工作区 | 多步任务拆解状态 |
| `ORCHESTRATOR.md` | 单机工作区或共享工作区 | 编排层状态 |
| `ROBOTS.md` | Fleet 共享工作区 | 机器人实例目录摘要 |

如果你想进一步理解这些文件如何在不同模式下协同，请阅读 [COMMUNICATION.md](../user_development_guide/COMMUNICATION.md)。

## 10. 常见交互与操作示例

### 10.1 环境查询

```text
看看当前环境里有什么物体。
```

适合验证：

- Agent 是否能读取 `ENVIRONMENT.md`
- 当前环境状态是否已由 Watchdog 正确写回

### 10.2 机械臂抓取/操作类任务

```text
把桌上的红色苹果拿起来，放到托盘里。
```

适合验证：

- 目标物体是否已出现在环境状态中
- 当前机器人 profile 是否声明了 `pick_up` / `put_down` 等动作
- Watchdog 是否成功执行动作并清空 `ACTION.md`

### 10.3 移动机器人导航类任务

```text
移动到冰箱附近并停下。
```

适合验证：

- 场景图或地图中是否存在目标语义位置
- 当前移动机器人是否支持导航或目标导航动作

### 10.4 Fleet 多机器人协同任务

```text
让 Go2 先去门口巡检，再让机械臂把桌上的包裹抓起来准备交接。
```

适合验证：

- Agent 是否识别多个机器人实例
- 动作是否被分发到正确的机器人工作区
- `ROBOTS.md` 与 `ENVIRONMENT.md` 是否正确更新状态

## 11. 常见问题与排查建议

### 11.1 提示没有 API Key

现象：启动 `paos agent` 或 `paos gateway` 后报没有 API key。

排查建议：

- 检查 `~/.PhyAgentOS/config.json`
- 确认 `agents.defaults.model` 与对应 provider 配套
- 确认正确填写了 `providers.<name>.api_key`

### 11.2 Watchdog 启动后没有 `EMBODIED.md`

现象：Critic 提示找不到 `EMBODIED.md`。

排查建议：

- 确认已执行 `paos onboard`
- 确认 Watchdog 已成功启动
- 确认所选 driver 的 profile 文件存在且可读取
- 若使用 Fleet 模式，确认你查看的是目标机器人的工作区

### 11.3 `ACTION.md` 有内容但动作没执行

排查建议：

- 确认对应 Watchdog 仍在运行
- 检查 `ACTION.md` 中 JSON 代码块格式是否完整
- 查看 Watchdog 终端日志是否出现驱动报错
- 检查 `driver-config` 是否缺失关键参数

### 11.4 动作被 Critic 拒绝

现象：Agent 返回动作无效或不安全。

排查建议：

- 先查看 `LESSONS.md`
- 再检查目标动作是否已在 `EMBODIED.md` 的 Supported Actions 中声明
- 检查 `ENVIRONMENT.md` 中是否存在对应目标物体、地图信息或机器人连接状态

### 11.5 Fleet 模式下任务没有派发到正确机器人

排查建议：

- 检查配置中的 `robot_id`、`driver`、`workspace` 是否匹配
- 确认 Watchdog 是通过 `--robot-id` 启动
- 检查共享工作区的 `ROBOTS.md` 是否已正确生成
- 确认任务语义里明确了目标机器人

### 11.6 找不到 `rekep_real` 驱动

排查建议：

- 确认已经执行插件部署脚本
- 确认插件仓库已注册到本地插件目录
- 优先参考 [phyagentos-rekep-plugin-blog-zh.md](../user_development_guide/REKEP_PLUGIN_DEPLOYMENT_zh.md) 的完整部署步骤

## 12. 未来章节（开发中）

以下主题建议后续继续扩展为独立专题文档；当前先在总手册中预留条目：

### 12.1 ROS2 深度接入（开发中）

计划补充：导航、TF、地图、桥接与适配器的完整说明。

### 12.2 多模态感知部署（开发中）

计划补充：几何管线、语义分割、融合与环境写回的部署清单。

### 12.3 长程任务与编排（开发中）

计划补充：`TASK.md`、`ORCHESTRATOR.md`、Heartbeat、Cron 的协同使用方式。

### 12.4 渠道接入操作手册（开发中）

计划补充：Telegram、Feishu、Slack、WhatsApp 等渠道的最小配置与排障流程。

### 12.5 多机器人实战案例（开发中）

计划补充：面向 Go2、机械臂、远程底盘的组合任务样例与推荐调试路径。

---

如果你的目标已经从“使用系统”转向“扩展系统”，下一站应当是 [USER_DEVELOPMENT_GUIDE.md](../user_development_guide/README.md)。
