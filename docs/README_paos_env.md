# PAOS Runtime Guide 

This guide only focuses on how to run the demo pipeline.

## 1) Install Isaac Sim 5.1 first

- Official download page (5.1.0):  
  [https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/download.html](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/download.html)
- Quick install doc (5.1.0):  
  [https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/quick-install.html](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/quick-install.html)

## 2) Prepare Python environment

```bash
conda activate paos
```

## 3) Install required dependencies

Install both local projects into the same `paos` environment:

```bash
cd /home/zyserver/work/PhyAgentOS
conda env create -f environment.yml
pip install -e .

```

## 4) Start HAL watchdog (GUI mode)

```bash
cd /home/zyserver/work/PhyAgentOS
conda activate paos
python hal/hal_watchdog.py --gui --interval 0.05 --driver pipergo2_manipulation --driver-config examples/pipergo2_manipulation_driver.json
```

## 4b) Start HAL watchdog (VNC mode, for containers without local X)

Use this mode when you are **SSHed into a remote server** (or running inside
a container) that has **no local display / GUI**, but you still want to
visualize the Isaac Sim window from your local machine through a browser.
You can also refer to the Isaac Sim official livestream guide for
alternative approaches:
[Manual Livestream Clients (Isaac Sim 4.5.0 docs)](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/manual_livestream_clients.html).

```bash
cd /home/zyserver/work/PhyAgentOS
conda activate paos
python hal/hal_watchdog.py --vnc --interval 0.05 --driver pipergo2_manipulation --driver-config examples/pipergo2_manipulation_driver.json
```

Then open a browser at `http://<host>:31315/vnc.html` to see the Isaac Sim
window. Here `31315` is the **VNC web (noVNC) visualization port** exposed
by the watchdog; replace `<host>` with your server's IP / hostname. If the
port is occupied or blocked, adjust the port mapping in your container /
firewall accordingly.

Notes:
- `--vnc` auto-bootstraps Isaac Sim env **inside the Python process** using the
  `isaac_env` block of the driver-config JSON: sets `DISPLAY` (defaults to
  `:99`), injects `ISAAC_PATH` / `CARB_APP_PATH` / `EXP_PATH` /
  `INTERNUTOPIA_ASSETS_PATH`, sources `setup_python_env.sh`, and prepends
  `extra_pythonpath` to both `PYTHONPATH` and `sys.path`. Users no longer need
  to wrap the command in a shell script that `source`s those vars.
- `--gui` and `--vnc` are mutually exclusive. Without either flag the
  watchdog runs headless.
- On first start in `--vnc` mode the watchdog **re-execs itself once**
  (`[isaac-bootstrap] LD_LIBRARY_PATH changed; re-exec ...` →
  `[isaac-bootstrap] post-reexec ready ...`). This is required because
  glibc's dynamic loader caches `LD_LIBRARY_PATH` at process start, so
  `libcarb.so` / `isaacsim` imports only succeed after the process is
  restarted with the environment sourced from `setup_python_env.sh`.
- Customize the Isaac Sim / InternUtopia paths in
  `examples/pipergo2_manipulation_driver.json` under the `isaac_env` key.

## 5) Send PAOS agent commands

Open another terminal:

```bash
cd /home/zyserver/work/PhyAgentOS
conda activate paos
```

Then run commands in order.

First, bring up the simulation:

```bash
paos agent -m "open simulation"
```

The next two manipulation commands use the **built-in rule-based** skills
(scripted navigation + pick-and-place):

```bash
paos agent -m "go to desk"
paos agent -m "pick up the red cube and return to the starting position"
```

Alternatively, you can deploy a **VLA (Vision-Language-Action) model** to
run the pick task end-to-end. The robot drives to the cube's approach
pose, the VLA closes the loop on the grasp, the gripper is force-closed,
and the robot **stops there** holding the cube (no auto-return to the
starting position). The example below is the reference command using `smolvla-piper` model:

```bash
paos agent -m "deploy a VLA to pick up the red cube"
```

Users are free to plug in their own VLA checkpoint by editing the `vla`
block of `examples/pipergo2_manipulation_driver.json` (update `ckpt_path`,
`policy_type`, camera keys, etc.) before running the command above.

## 6) Notes

- Keep only one watchdog process running.
- If you modify driver or skill files, restart watchdog.
- If the simulator is laggy, make sure `--interval 0.05` is used.
