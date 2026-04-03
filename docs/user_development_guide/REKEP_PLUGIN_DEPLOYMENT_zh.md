# PhyAgentOS-rekep-real-plugin 接入 PhyAgentOS：一键部署、开发与执行

PhyAgentOS 把 ReKep 真机能力从主仓库中抽离出来，作为独立插件仓库发布。这么做的目标很明确：让 PhyAgentOS 主仓库保持干净、稳定、易维护，同时让 ReKep 真机运行时可以按需安装。

## 一键部署

先准备 PhyAgentOS 主仓库：

```bash
git clone https://github.com/SYSU-HCP-EAI/PhyAgentOS.git
cd PhyAgentOS
pip install -e .
pip install watchdog
```

然后一键拉取并注册 ReKep 插件：

```bash
python scripts/deploy_rekep_real_plugin.py \
  --repo-url https://github.com/baiyu858/PhyAgentOS-rekep-real-plugin.git
```

如果你还需要 solver 的可选依赖，可以加上：

```bash
python scripts/deploy_rekep_real_plugin.py \
  --repo-url https://github.com/baiyu858/PhyAgentOS-rekep-real-plugin.git \
  --with-solver
```

完成后，PhyAgentOS 就可以识别 `rekep_real` 驱动。

## 启动 PhyAgentOS 与 ReKep 真机执行

先初始化工作区：

```bash
paos onboard
```

然后开启两个终端。

终端 1，启动硬件看门狗：

```bash
python hal/hal_watchdog.py --driver rekep_real --workspace ~/.PhyAgentOS/workspace
```

终端 2，启动大脑 Agent：

```bash
paos agent
```

这样之后，Agent 负责生成动作意图，`hal_watchdog.py` 负责监听 `ACTION.md` 并调用 `rekep_real` 驱动执行，从而形成完整的“感知-决策-执行”闭环。

## ReKep 插件开发怎么做

如果你要开发或修改 ReKep 真机能力，不再需要改 PhyAgentOS 主仓库本体，而是直接在插件仓库里工作：

```bash
git clone https://github.com/baiyu858/PhyAgentOS-rekep-real-plugin.git
cd PhyAgentOS-rekep-real-plugin
```

开发时建议把插件仓库放在 PhyAgentOS 仓库同级目录，然后用本地路径安装回主仓库：

```bash
cd ../PhyAgentOS
python scripts/deploy_rekep_real_plugin.py \
  --repo-url ../PhyAgentOS-rekep-real-plugin
```

通常你会关注这几个位置：

- `phyagentos_rekep_real_plugin/driver.py`：PhyAgentOS 侧外部 HAL 驱动入口
- `runtime/dobot_bridge.py`：ReKep 真机 bridge 主入口
- `runtime/robot_factory.py`：机器人 family 的适配工厂
- `runtime/cellbot_adapter.py`：新机器人适配模板
- `runtime/docs/robot_adaptation_zh.md`：新机器人接入说明

这套结构的好处是边界清晰：PhyAgentOS 管框架与协议，插件管真机执行与机器人适配。

## 开发后如何验证

最直接的方式是先做预检：

```bash
python runtime/dobot_bridge.py preflight --pretty
```

再做一次 dry-run：

```bash
python runtime/dobot_bridge.py execute \
  --instruction "pick up the red block and place it on the tray" \
  --pretty
```

确认流程无误后，再开启真实动作执行：

```bash
python runtime/dobot_bridge.py execute \
  --instruction "pick up the red block and place it on the tray" \
  --execute_motion \
  --pretty
```

如果你在适配新机器人，也可以通过通用参数切换不同 family：

```bash
python runtime/dobot_bridge.py preflight \
  --robot_family cellbot \
  --robot_driver your_driver
```

## 相关仓库

- PhyAgentOS 主仓库：`https://github.com/SYSU-HCP-EAI/PhyAgentOS.git`
- ReKep 真机插件：`https://github.com/baiyu858/PhyAgentOS-rekep-real-plugin`
- ReKep 插件发布页：`https://github.com/baiyu858/PhyAgentOS-rekep-real-plugin/releases`
