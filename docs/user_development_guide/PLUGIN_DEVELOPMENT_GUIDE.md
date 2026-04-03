# PhyAgentOS Plugin Development Template (Using PhyAgentOS-rekep-real-plugin as an Example)

English | [中文](PLUGIN_DEVELOPMENT_GUIDE_zh.md)

> A community-facing reference for developing, deploying, debugging, and releasing external PhyAgentOS plugins.

This guide answers four questions:

1. When should a robot capability be built as an external plugin instead of changing the PhyAgentOS core?
2. What files and conventions are required for a minimal working plugin?
3. How do you develop, deploy, and invoke a plugin using `PhyAgentOS-rekep-real-plugin` as the example?
4. What testing and release workflow is recommended for a community-maintained plugin repository?

## 1. What Is a PhyAgentOS Plugin

PhyAgentOS uses HAL (Hardware Abstraction Layer) to connect different embodiments through one unified execution interface. The core rule is simple:

- `hal/hal_watchdog.py` only talks to a `BaseDriver`.

That means any robot can be integrated into PhyAgentOS as long as you provide a driver that implements the `BaseDriver` contract. If you do not want to modify the main repository directly, an external plugin repository is the preferred path.

A plugin is the right choice when:

- You are integrating hardware-specific or real-world runtime logic.
- You need to ship heavier third-party SDKs, camera stacks, or vendor code.
- You want to version, test, and release the robot integration independently.

A plugin is usually not necessary when:

- You are only fixing a small bug in an existing built-in driver.
- You are making a tiny simulation-only improvement inside the core repository.

## 2. How PhyAgentOS Discovers Plugins

The loading path for an external plugin is:

1. The plugin repository root provides `PhyAgentOS_plugin.toml`.
2. A deployment script such as `scripts/deploy_rekep_real_plugin.py` clones or copies the plugin into the local plugin directory.
3. `hal/plugins.py` reads `PhyAgentOS_plugin.toml` and registers the driver metadata locally.
4. `hal/drivers/__init__.py` falls back to the external registry when a built-in driver is not found.
5. `hal/hal_watchdog.py --driver <driver_name>` can then load and use the plugin as if it were built in.

In practice, a plugin is “a HAL driver + a profile + a runtime shipped in a separate repository”.

## 3. Minimal Plugin Repository Layout

A recommended structure looks like this:

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

What each part does:

- `PhyAgentOS_plugin.toml`: the plugin manifest that tells PhyAgentOS which driver is provided and where its profile lives.
- `my_plugin/driver.py`: the `BaseDriver` implementation loaded by the watchdog.
- `my_plugin/profiles/*.md`: embodiment profile files copied into the workspace as `EMBODIED.md`.
- `runtime/`: real-world runtime logic, bridge scripts, vendor SDK glue, camera logic, and similar code.
- `tests/`: plugin-local unit tests and smoke tests.

## 4. Writing the Plugin Manifest

A minimal `PhyAgentOS_plugin.toml` looks like this:

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

Key fields:

- `plugin.name`: plugin identifier for display and registry metadata.
- `driver.name`: the driver name passed to the watchdog, for example `--driver my_robot`.
- `driver.module`: import path for the driver module.
- `driver.class`: driver class name.
- `driver.profile_path`: path to the embodiment profile.
- `python.sys_paths`: paths that must be added to `sys.path` when loading the plugin.
- `requirements`: requirement files installed by the deployment script.

## 5. What the Driver Must Implement

Every plugin driver must inherit from [`hal/base_driver.py`](https://github.com/SYSU-HCP-EAI/PhyAgentOS/blob/main/hal/base_driver.py).

A minimal template:

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

Responsibilities of the four required methods:

- `get_profile_path()`: returns the embodiment profile path.
- `load_scene(scene)`: initializes internal state from the scene parsed out of `ENVIRONMENT.md`.
- `execute_action(action_type, params)`: executes one action and returns a human-readable result.
- `get_scene()`: returns current environment state so `ENVIRONMENT.md` can be updated.

Optional but commonly useful methods:

- `connect()`
- `disconnect()`
- `is_connected()`
- `health_check()`
- `get_runtime_state()`

## 6. How the ReKep Plugin Is Structured

Reference repository: `https://github.com/baiyu858/PhyAgentOS-rekep-real-plugin`

Key files:

- [`PhyAgentOS_plugin.toml`](https://github.com/baiyu858/PhyAgentOS-rekep-real-plugin/blob/main/PhyAgentOS_plugin.toml)
- [`driver.py`](https://github.com/baiyu858/PhyAgentOS-rekep-real-plugin/blob/main/phyagentos_rekep_real_plugin/driver.py)
- [`rekep_real.md`](https://github.com/baiyu858/PhyAgentOS-rekep-real-plugin/blob/main/phyagentos_rekep_real_plugin/profiles/rekep_real.md)
- [`runtime/README.md`](https://github.com/baiyu858/PhyAgentOS-rekep-real-plugin/blob/main/runtime/README.md)
- [`dobot_bridge.py`](https://github.com/baiyu858/PhyAgentOS-rekep-real-plugin/blob/main/runtime/dobot_bridge.py)

The ReKep design intentionally separates concerns into two layers:

1. `ReKepRealDriver` adapts the PhyAgentOS `BaseDriver` interface.
2. `runtime/dobot_bridge.py` implements the heavy real-world execution logic, background jobs, preflight checks, and long-running task orchestration.

This split is strongly recommended because it keeps boundaries clear:

- The driver layer handles “PhyAgentOS protocol adaptation”.
- The runtime layer handles “real robot capability implementation”.

## 7. How ReKep Maps ACTIONs to Runtime Commands

`ReKepRealDriver` follows two important patterns.

### 7.1 Native Plugin Actions

The plugin directly supports actions such as:

- `real_preflight`
- `real_execute`
- `real_execute_background`
- `real_scene_qa`
- `real_longrun_start`
- `real_longrun_status`
- `real_longrun_command`
- `real_longrun_stop`

These are mapped to subcommands in `runtime/dobot_bridge.py`.

### 7.2 High-Level Action Compatibility

ReKep also accepts generic high-level actions such as:

- `move_to`
- `pick_up`
- `put_down`
- `push`
- `point_to`
- `open_gripper`
- `close_gripper`

These are translated into natural-language instructions and then routed to `execute`.

This makes ReKep a strong template for community plugins because it supports both:

- the generic PhyAgentOS action protocol
- plugin-specific action extensions

## 8. Local Development Workflow

A convenient setup is to place the plugin repository next to the main repository:

```bash
git clone https://github.com/SYSU-HCP-EAI/PhyAgentOS.git
git clone https://github.com/baiyu858/PhyAgentOS-rekep-real-plugin.git
```

Recommended layout:

```text
~/model/
├── PhyAgentOS/
└── PhyAgentOS-rekep-real-plugin/
```

Install the main repository first:

```bash
cd PhyAgentOS
pip install -e .
```

During development, install the plugin from the local checkout:

```bash
python scripts/deploy_rekep_real_plugin.py \
  --repo-url ../PhyAgentOS-rekep-real-plugin
```

If you are building a different plugin, you can follow the same pattern with your own deployment script or direct registration logic.

## 9. Writing a Deployment Script

The ReKep deployment script lives in the main repository:

- [`deploy_rekep_real_plugin.py`](https://github.com/SYSU-HCP-EAI/PhyAgentOS/blob/main/scripts/deploy_rekep_real_plugin.py)

It performs four core tasks:

1. fetch the plugin repository from a git URL or a local path
2. read `PhyAgentOS_plugin.toml`
3. install plugin requirements
4. call `register_plugin()` to write into the local registry

For community-facing plugins, a dedicated deployment script is strongly recommended because it lowers adoption friction.

## 10. Deploying the ReKep Plugin

### 10.1 Install from GitHub

```bash
cd PhyAgentOS
python scripts/deploy_rekep_real_plugin.py \
  --repo-url https://github.com/baiyu858/PhyAgentOS-rekep-real-plugin.git
```

Install optional solver dependencies as well:

```bash
python scripts/deploy_rekep_real_plugin.py \
  --repo-url https://github.com/baiyu858/PhyAgentOS-rekep-real-plugin.git \
  --with-solver
```

### 10.2 Install from a Local Checkout

```bash
cd PhyAgentOS
python scripts/deploy_rekep_real_plugin.py \
  --repo-url ../PhyAgentOS-rekep-real-plugin
```

### 10.3 Register Without Installing Dependencies

```bash
python scripts/deploy_rekep_real_plugin.py \
  --repo-url ../PhyAgentOS-rekep-real-plugin \
  --no-install-deps
```

This mode is useful when the runtime environment has already been prepared manually.

## 11. Invoking the ReKep Plugin

There are three common invocation paths.

### 11.1 Through HAL Watchdog

Initialize the workspace first:

```bash
paos onboard
```

Start the watchdog:

```bash
python hal/hal_watchdog.py --driver rekep_real --workspace ~/.PhyAgentOS/workspace
```

In another terminal, start the Agent:

```bash
paos agent
```

At this point, `ACTION.md` generated by the Agent will be executed by the `rekep_real` driver.

### 11.2 Through ACTION.md Directly

A native plugin action example:

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

A generic high-level action example:

```json
{
  "action_type": "pick_up",
  "parameters": {
    "target": "red_apple"
  },
  "status": "pending"
}
```

The first is plugin-native; the second uses the generic PhyAgentOS action space.

### 11.3 By Running the Runtime Directly

Preflight:

```bash
python runtime/dobot_bridge.py preflight --pretty
```

Dry-run:

```bash
python runtime/dobot_bridge.py execute \
  --instruction "pick up the red block and place it on the tray" \
  --pretty
```

Real execution:

```bash
python runtime/dobot_bridge.py execute \
  --instruction "pick up the red block and place it on the tray" \
  --execute_motion \
  --pretty
```

This is often the best path during plugin development because it isolates the runtime from the Agent and Markdown protocol loop.

## 12. What a Plugin Profile Should Contain

Every plugin should provide an embodiment profile that describes:

- robot identity
- supported actions
- default safety policy
- key environment variables
- required runtime assets

Reference implementation:

- [`rekep_real.md`](https://github.com/baiyu858/PhyAgentOS-rekep-real-plugin/blob/main/phyagentos_rekep_real_plugin/profiles/rekep_real.md)

A good rule is to treat the profile as a hardware instruction sheet for the Agent, not just as a developer note.

## 13. Testing Recommendations

Minimum recommendations:

- one manifest parsing and registration test
- one driver action-routing test
- one runtime preflight test
- at least one `python -m compileall` smoke check

Useful references:

- [`tests/test_hal_external_plugins.py`](https://github.com/SYSU-HCP-EAI/PhyAgentOS/blob/main/tests/test_hal_external_plugins.py)
- [`tests/test_rekep_real_driver.py`](https://github.com/baiyu858/PhyAgentOS-rekep-real-plugin/blob/main/tests/test_rekep_real_driver.py)

## 14. Recommended Release Checklist for Community Plugins

If you plan to publish your plugin, prepare at least the following:

- a standalone GitHub repository
- clear `README.md` and `README_zh.md`
- `PhyAgentOS_plugin.toml`
- an embodiment profile
- minimal requirements files
- at least one test suite
- tags and releases
- secret scanning or pre-commit protections

The ReKep plugin is a strong example because it shows both how to package a real-world runtime and how to keep heavier dependencies out of the main repository.

## 15. Suggested Community Development Workflow

A practical order of operations is:

1. write `PhyAgentOS_plugin.toml`
2. implement the smallest possible `BaseDriver`
3. add the embodiment profile so `EMBODIED.md` installs correctly
4. move complex logic into `runtime/` instead of overloading the driver
5. deploy locally, then validate `preflight` and dry-run `execute`
6. finish by integrating with `hal_watchdog.py` and `paos agent`

## 16. Summary

If you are building a community plugin for PhyAgentOS, keep these three principles in mind:

1. the main repository should only care about the interface, not your hardware internals
2. the driver layer should stay thin; push complexity into the runtime
3. a plugin should be self-explanatory: manifest, profile, README, and tests should all exist

ReKep demonstrates a complete plugin shape:

- standalone repository
- driver entrypoint
- embodiment profile
- runtime layer
- deployment script
- user-facing documentation
- tests and release workflow

If you are starting a new robot integration, the shortest path is usually not to begin from scratch, but to clone the ReKep plugin structure and replace the name, runtime, and action mapping with your own implementation.
