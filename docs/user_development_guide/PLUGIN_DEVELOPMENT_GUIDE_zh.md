# PhyAgentOS 插件开发参考模板（以 PhyAgentOS-rekep-real-plugin 为例）

[English](PLUGIN_DEVELOPMENT_GUIDE.md) | 中文

> 面向社区开发者的外部插件开发、部署、调试与发布参考。

本文档回答四个问题：

1. 什么时候应该把机器人能力做成外部插件，而不是直接改 PhyAgentOS 核心。
2. 一个最小可用插件需要哪些文件和约定。
3. 以 `PhyAgentOS-rekep-real-plugin` 为例，如何开发、部署和调用插件。
4. 社区维护一个插件仓库时，推荐的测试和发布流程是什么。

## 1. 什么是 PhyAgentOS 插件

PhyAgentOS 的执行层通过 HAL（Hardware Abstraction Layer）统一接入不同本体。核心约束只有一条：

- 看门狗 `hal/hal_watchdog.py` 只和 `BaseDriver` 交互。

也就是说，只要你能提供一个符合 `BaseDriver` 接口的驱动实现，PhyAgentOS 就可以把它作为一个新的机器人本体来加载。对于不想直接修改主仓库的场景，更推荐使用外部插件仓库。

适合做成插件的情况：

- 你要接入的是特定硬件或真机运行时。
- 你需要携带较重的第三方 SDK、相机驱动或厂商代码。
- 你希望插件独立发版、独立维护依赖、独立发布到 GitHub。

不适合做成插件的情况：

- 只是修一个已有驱动的小 bug。
- 只是给主仓库内置仿真器补一个很小的行为。

## 2. PhyAgentOS 如何发现插件

PhyAgentOS 插件加载链路如下：

1. 外部插件仓库根目录必须提供 `PhyAgentOS_plugin.toml`。
2. `scripts/deploy_rekep_real_plugin.py` 这类部署脚本把插件仓库复制或 clone 到本地插件目录。
3. `hal/plugins.py` 读取 `PhyAgentOS_plugin.toml`，把驱动信息写入本地 registry。
4. `hal/drivers/__init__.py` 在内置驱动找不到时，会从外部 registry 解析驱动并动态导入。
5. `hal/hal_watchdog.py --driver <driver_name>` 启动后，就会像使用内置驱动一样使用该插件。

你可以把插件理解为“独立仓库里的 HAL driver + profile + runtime”。

## 3. 一个最小插件的目录结构

推荐目录结构如下：

```text
my-phyagentos-plugin/
├── PhyAgentOS_plugin.toml
├── README.md
├── README_zh.md
├── pyproject.toml
├── my_plugin/
│   ├── __init__.py
│   ├── driver.py
│   └── profiles/
│       └── my_robot.md
├── runtime/
│   ├── README.md
│   └── your_runtime_entry.py
└── tests/
    └── test_my_driver.py
```

各目录职责：

- `PhyAgentOS_plugin.toml`：插件清单，告诉主仓库“这个插件提供哪个 driver、入口类是什么、profile 在哪里”。
- `my_plugin/driver.py`：真正被看门狗加载的 `BaseDriver` 实现。
- `my_plugin/profiles/*.md`：本体能力档案，启动时会被复制到工作区的 `EMBODIED.md`。
- `runtime/`：真机运行时、桥接脚本、第三方 SDK、相机逻辑等。
- `tests/`：插件自己的单元测试和冒烟测试。

## 4. 清单文件怎么写

一个最小可用的 `PhyAgentOS_plugin.toml` 示例：

```toml
[plugin]
name = "my_robot"
version = "0.1.0"
description = "My robot plugin for PhyAgentOS"

[driver]
name = "my_robot"
module = "my_plugin.driver"
class = "MyRobotDriver"
profile_path = "my_plugin/profiles/my_robot.md"

[python]
sys_paths = ["."]

[[requirements]]
path = "runtime/requirements.txt"
optional = false
```

关键字段说明：

- `plugin.name`：插件名字，用于展示和 registry 元信息。
- `driver.name`：看门狗启动时传入的名字，例如 `--driver my_robot`。
- `driver.module`：驱动类所在模块。
- `driver.class`：驱动类名。
- `driver.profile_path`：本体 profile 路径。
- `python.sys_paths`：动态导入该插件时需要加入 `sys.path` 的目录。
- `requirements`：部署脚本安装依赖时要读取的 requirements 文件列表。

## 5. 驱动必须实现什么接口

所有插件驱动都必须继承 [`hal/base_driver.py`](https://github.com/SYSU-HCP-EAI/PhyAgentOS/blob/main/hal/base_driver.py) 中定义的 `BaseDriver`。

最小驱动模板：

```python
from pathlib import Path

from hal.base_driver import BaseDriver


class MyRobotDriver(BaseDriver):
    def __init__(self, gui: bool = False, **kwargs):
        self._scene = {}

    def get_profile_path(self) -> Path:
        return Path(__file__).resolve().parent / "profiles" / "my_robot.md"

    def load_scene(self, scene: dict[str, dict]) -> None:
        self._scene = dict(scene)

    def execute_action(self, action_type: str, params: dict) -> str:
        return f"executed {action_type}"

    def get_scene(self) -> dict[str, dict]:
        return dict(self._scene)
```

这四个方法的职责很明确：

- `get_profile_path()`：返回本体档案路径。
- `load_scene(scene)`：用 `ENVIRONMENT.md` 解析出来的 scene 初始化内部状态。
- `execute_action(action_type, params)`：执行动作，返回人类可读字符串。
- `get_scene()`：返回当前环境状态，用于回写 `ENVIRONMENT.md`。

可选但常用的扩展方法：

- `connect()`：建立硬件连接。
- `disconnect()`：断开连接。
- `is_connected()`：返回当前连接状态。
- `health_check()`：轻量健康检查。
- `get_runtime_state()`：把机器人运行时状态写回环境文档。

## 6. ReKep 插件是怎么实现的

ReKep 插件仓库：`https://github.com/baiyu858/PhyAgentOS-rekep-real-plugin`

它是一个很典型的社区插件样板。

核心文件分工：

- [`PhyAgentOS_plugin.toml`](https://github.com/baiyu858/PhyAgentOS-rekep-real-plugin/blob/main/PhyAgentOS_plugin.toml)
- [`driver.py`](https://github.com/baiyu858/PhyAgentOS-rekep-real-plugin/blob/main/phyagentos_rekep_real_plugin/driver.py)
- [`rekep_real.md`](https://github.com/baiyu858/PhyAgentOS-rekep-real-plugin/blob/main/phyagentos_rekep_real_plugin/profiles/rekep_real.md)
- [`runtime/README.md`](https://github.com/baiyu858/PhyAgentOS-rekep-real-plugin/blob/main/runtime/README.md)
- [`dobot_bridge.py`](https://github.com/baiyu858/PhyAgentOS-rekep-real-plugin/blob/main/runtime/dobot_bridge.py)

ReKep 的设计不是让 `BaseDriver` 自己实现完整的操控逻辑，而是采用两层结构：

1. `ReKepRealDriver` 负责适配 PhyAgentOS 的 `BaseDriver` 接口。
2. `runtime/dobot_bridge.py` 负责真机执行、后台任务、预检、长程任务等复杂流程。

这是一种很推荐的做法，因为它让边界非常清晰：

- 驱动层负责“PhyAgentOS 协议适配”。
- runtime 层负责“真实机器人能力实现”。

## 7. ReKep 驱动如何把 ACTION 映射到 runtime

`ReKepRealDriver` 的关键模式有两个：

### 7.1 原生动作映射

插件直接支持这些动作：

- `real_preflight`
- `real_execute`
- `real_execute_background`
- `real_scene_qa`
- `real_longrun_start`
- `real_longrun_status`
- `real_longrun_command`
- `real_longrun_stop`

驱动会把它们映射到 `runtime/dobot_bridge.py` 的子命令。

### 7.2 高层动作兼容

ReKep 还兼容通用高层动作：

- `move_to`
- `pick_up`
- `put_down`
- `push`
- `point_to`
- `open_gripper`
- `close_gripper`

这些动作不会直接走底层 SDK，而是先由驱动拼成自然语言 instruction，再映射到 `execute`。

这套设计很适合作为社区插件模板，因为它同时兼容：

- PhyAgentOS 通用动作协议
- 插件专用动作协议

## 8. 如何本地开发一个插件

推荐把插件仓库放在主仓库同级目录：

```bash
git clone https://github.com/SYSU-HCP-EAI/PhyAgentOS.git
git clone https://github.com/baiyu858/PhyAgentOS-rekep-real-plugin.git
```

目录结构建议如下：

```text
~/model/
├── PhyAgentOS/
└── PhyAgentOS-rekep-real-plugin/
```

然后先安装主仓库：

```bash
cd PhyAgentOS
pip install -e .
```

开发期间，从本地路径安装插件：

```bash
python scripts/deploy_rekep_real_plugin.py \
  --repo-url ../PhyAgentOS-rekep-real-plugin
```

如果你自己写的是另一个插件，也可以复用同样的思路，只需要准备一个自己的部署脚本，或者直接调用 `hal.plugins` 中的注册逻辑。

## 9. 如何为你的插件写一个部署脚本

ReKep 的部署脚本在主仓库中：

- [`deploy_rekep_real_plugin.py`](https://github.com/SYSU-HCP-EAI/PhyAgentOS/blob/main/scripts/deploy_rekep_real_plugin.py)

这个脚本做了几件事：

1. 从 git URL 或本地路径获取插件仓库。
2. 读取 `PhyAgentOS_plugin.toml`。
3. 安装插件 requirements。
4. 调用 `register_plugin()` 写入本地 registry。

如果你的插件要面向社区发布，推荐保留一个同类脚本，因为这会显著降低使用门槛。

## 10. 如何部署 ReKep 插件

### 10.1 在线安装

```bash
cd PhyAgentOS
python scripts/deploy_rekep_real_plugin.py \
  --repo-url https://github.com/baiyu858/PhyAgentOS-rekep-real-plugin.git
```

安装 solver 额外依赖：

```bash
python scripts/deploy_rekep_real_plugin.py \
  --repo-url https://github.com/baiyu858/PhyAgentOS-rekep-real-plugin.git \
  --with-solver
```

### 10.2 本地联调安装

```bash
cd PhyAgentOS
python scripts/deploy_rekep_real_plugin.py \
  --repo-url ../PhyAgentOS-rekep-real-plugin
```

### 10.3 只注册不装依赖

```bash
python scripts/deploy_rekep_real_plugin.py \
  --repo-url ../PhyAgentOS-rekep-real-plugin \
  --no-install-deps
```

这个模式适合你已经手动准备好运行环境的情况。

## 11. 如何调用 ReKep 插件

ReKep 有三种常用调用方式。

### 11.1 通过 HAL Watchdog 调用

先初始化工作区：

```bash
paos onboard
```

启动看门狗：

```bash
python hal/hal_watchdog.py --driver rekep_real --workspace ~/.PhyAgentOS/workspace
```

另开一个终端启动 Agent：

```bash
paos agent
```

这样 Agent 生成的 `ACTION.md` 就会由 `rekep_real` 驱动处理。

### 11.2 通过 ACTION.md 直接调用

写入一个原生动作：

```json
{
  "action_type": "real_execute",
  "parameters": {
    "instruction": "pick up the chili pepper and place it in the plate",
    "execute_motion": true
  },
  "status": "pending"
}
```

也可以写入一个高层动作：

```json
{
  "action_type": "pick_up",
  "parameters": {
    "target": "red_apple"
  },
  "status": "pending"
}
```

前者是插件原生协议，后者是通用动作协议。

### 11.3 直接调 runtime 调试

预检：

```bash
python runtime/dobot_bridge.py preflight --pretty
```

dry-run：

```bash
python runtime/dobot_bridge.py execute \
  --instruction "pick up the red block and place it on the tray" \
  --pretty
```

真机执行：

```bash
python runtime/dobot_bridge.py execute \
  --instruction "pick up the red block and place it on the tray" \
  --execute_motion \
  --pretty
```

这类命令最适合插件开发阶段做定位，因为它绕开了 Agent 和 Markdown 协议，可以单独验证 runtime。

## 12. 插件 profile 应该写什么

每个插件都应该提供一个本体档案，用来声明：

- 本体身份
- 支持的动作
- 默认安全策略
- 关键环境变量
- 运行依赖

ReKep 的参考实现：

- [`rekep_real.md`](https://github.com/baiyu858/PhyAgentOS-rekep-real-plugin/blob/main/phyagentos_rekep_real_plugin/profiles/rekep_real.md)

推荐把 profile 当成“给 Agent 看的硬件说明书”，而不是只写给开发者看的技术文件。

## 13. 插件开发时的测试建议

最低建议：

- 写一个 manifest 解析 / 注册测试。
- 写一个 driver 动作路由测试。
- 写一个 runtime 预检命令测试。
- 至少做一次 `python -m compileall` 冒烟。

主仓库里可以参考：

- [`tests/test_hal_external_plugins.py`](https://github.com/SYSU-HCP-EAI/PhyAgentOS/blob/main/tests/test_hal_external_plugins.py)

ReKep 插件里可以参考：

- [`tests/test_rekep_real_driver.py`](https://github.com/baiyu858/PhyAgentOS-rekep-real-plugin/blob/main/tests/test_rekep_real_driver.py)

## 14. 社区插件推荐发布清单

如果你准备把插件对外发布，建议至少准备这些内容：

- 独立 GitHub 仓库
- 清晰的 `README.md` 和 `README_zh.md`
- `PhyAgentOS_plugin.toml`
- 插件 profile
- 最小 requirements 文件
- 最少一组测试
- tag 和 release
- secrets scanning 或 pre-commit 检查

ReKep 插件就是一个完整样板：它既展示了如何做真机 runtime，也展示了如何把重依赖和第三方代码从主仓库中解耦出去。

## 15. 一个推荐的社区开发流程

可以按下面的顺序推进：

1. 先写 `PhyAgentOS_plugin.toml`。
2. 再写最小 `BaseDriver`，先让看门狗能加载起来。
3. 再补 `profile`，让 `EMBODIED.md` 能正确安装。
4. 再把复杂能力放到 `runtime/`，不要把所有逻辑都塞进 driver。
5. 本地部署插件，先跑 `preflight`，再跑 `execute` 的 dry-run。
6. 最后接入 `hal_watchdog.py` 和 `paos agent` 做全链路联调。

## 16. 总结

如果你要为 PhyAgentOS 社区开发一个插件，最重要的是记住这三个原则：

1. 主仓库只关心统一接口，不关心你的底层硬件细节。
2. 驱动层尽量薄，把复杂逻辑下沉到 runtime。
3. 插件必须自解释：清单、profile、README、测试要齐全。

以 ReKep 为例，你可以看到一个成熟插件的完整形态：

- 有独立仓库
- 有驱动入口
- 有 profile
- 有 runtime
- 有部署脚本
- 有对外文档
- 有测试与发布流程

如果你正在开发新的真实机器人插件，最简单的路径不是从零开始，而是直接参考 ReKep 插件仓库，把名字、runtime 和动作映射替换成你自己的实现。
