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

```bash
cd /home/zyserver/work/PhyAgentOS
conda activate paos
python hal/hal_watchdog.py --vnc --interval 0.05 --driver pipergo2_manipulation --driver-config examples/pipergo2_manipulation_driver.json
```

Then open a browser at `http://<host>:31315/vnc.html` to see the Isaac Sim window.

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

Then run commands in order:

```bash
paos agent -m "open simulation"
paos agent -m "go to desk"
paos agent -m "pick up the red cube and return to the starting position"
```

## 6) Notes

- Keep only one watchdog process running.
- If you modify driver or skill files, restart watchdog.
- If the simulator is laggy, make sure `--interval 0.05` is used.
