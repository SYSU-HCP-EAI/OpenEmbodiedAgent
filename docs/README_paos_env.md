# PAOS Runtime Guide 

This guide only focuses on how to run the demo pipeline.

Throughout this document, **`<repo>`** means the root directory of your PhyAgentOS git clone.
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
cd <repo>
conda env create -f environment.yml
pip install -e .

```

## 3b) Download simulation assets (`asserts/`)

Large baked scene meshes, textures, and robot USD/weights are **not** committed to Git by default. Download the shared bundle from Google Drive, then extract it so the repository root contains an `asserts/` directory (with `merom_scene_baked.usd`, `baked_scene_assets/`, `robots/`, etc.).

- **Google Drive folder (PhyAgentOS_default_asserts):**  
  [https://drive.google.com/drive/folders/1PMee8M4AGUnlXLGizW-ACADPsUjQ9YPB?usp=sharing](https://drive.google.com/drive/folders/1PMee8M4AGUnlXLGizW-ACADPsUjQ9YPB?usp=sharing)

Steps (replace `<repo>` with your clone path, e.g. `/home/zyserver/work/PhyAgentOS`):

```bash
cd <repo>
# Download asserts.zip from the folder above, then:
unzip -o asserts.zip -d .
# Expect: ./asserts/merom_scene_baked.usd, ./asserts/baked_scene_assets/, ./asserts/robots/, ...
```

If `INTERNUTOPIA_ASSETS_PATH` is unset, robot policy paths resolve under `<repo>/asserts/robots/...`. Scene paths in the example JSON files point at `asserts/merom_scene_baked.usd`.

## 4a) Start HAL watchdog (GUI mode)

From the repository root (same `<repo>` as above):

**PiperGo2 manipulation**

```bash
cd <repo>
conda activate paos
python hal/hal_watchdog.py --gui --interval 0.05 --driver pipergo2_manipulation --driver-config examples/pipergo2_manipulation_driver.json --robot-id pipergo2_manip_001
```

**Franka simulation**

```bash
cd <repo>
conda activate paos
python hal/hal_watchdog.py --gui --interval 0.05 --driver franka_simulation --driver-config examples/franka_simulation_driver.json --robot-id franka_001
```

**G1 navigation**

```bash
cd <repo>
conda activate paos
python hal/hal_watchdog.py --gui --interval 0.05 --driver g1_navigation --driver-config examples/g1_navigation_driver.json --robot-id g1_001
```

## 4b) Start HAL watchdog (VNC mode, for containers without local X)

```bash
cd <repo>
conda activate paos
python hal/hal_watchdog.py --vnc --interval 0.05 --driver pipergo2_manipulation --driver-config examples/pipergo2_manipulation_driver.json --robot-id pipergo2_manip_001
```

Then open a browser at `http://<host>:31315/vnc.html` to see the Isaac Sim window.

Notes:
- Robot ONNX/PT/USD defaults resolve under `asserts/robots/{aliengo,franka,g1,pipergo2}/` via `internutopia.macros.gm.ASSET_PATH`, which defaults to `<repo>/asserts` (falling back to `<repo>/examples`) when `INTERNUTOPIA_ASSETS_PATH` is unset.
- `--vnc` auto-bootstraps Isaac Sim env **inside the Python process** using the
  `isaac_env` block of the driver-config JSON: sets `DISPLAY` (defaults to
  `:99`), injects `ISAAC_PATH` / `CARB_APP_PATH` / `EXP_PATH` /
  `INTERNUTOPIA_ASSETS_PATH`, sources `setup_python_env.sh`, and mirrors
  `PYTHONPATH` into `sys.path`. Optional `extra_pythonpath` entries are still
  supported. Vendored `internutopia` / `internutopia_extension` under `hal/`
  are added automatically so you do not need a separate InternUtopia checkout
  on `PYTHONPATH` for HAL drivers.
- `--gui` and `--vnc` are mutually exclusive. Without either flag the
  watchdog runs headless.
- On first start in `--vnc` mode the watchdog **re-execs itself once**
  (`[isaac-bootstrap] LD_LIBRARY_PATH changed; re-exec ...` →
  `[isaac-bootstrap] post-reexec ready ...`). This is required because
  glibc's dynamic loader caches `LD_LIBRARY_PATH` at process start, so
  `libcarb.so` / `isaacsim` imports only succeed after the process is
  restarted with the environment sourced from `setup_python_env.sh`.
- Customize Isaac Sim install paths and `INTERNUTOPIA_ASSETS_PATH` in
  `examples/pipergo2_manipulation_driver.json` under the `isaac_env` key.

## 5) Send PAOS agent commands

Open another terminal:

```bash
cd <repo>
conda activate paos
```

Then run commands in order:

```bash
paos agent -m "open simulation for pipergo2/franka/g1"
paos agent -m "XXX go to desk"
paos agent -m "what is on the table"
paos agent -m "pick up the red cube and return to the starting position"
```

The table question is answered immediately from the current `ENVIRONMENT.md` scene graph / manipulation runtime state. In fleet mode, include the target `robot_id` in the tool call context.

## 6) Notes

- Keep only one watchdog process running.
- If you modify driver or skill files, restart watchdog.
- If the simulator is laggy, make sure `--interval 0.05` is used.

## 7) Example: 


<div align="center">
  <p><b>Franka_sim</b></p>
  <img src="imgs/Franka_sim.gif" alt="Franka simulation demo" width="720">
</div>

<div align="center">
  <p><b>g1_sim</b></p>
  <img src="imgs/g1_sim.gif" alt="G1 humanoid simulation demo" width="720">
</div>

<div align="center">
  <p><b>VQA_sim</b></p>
  <img src="imgs/VQA_sim.gif" alt="Visual question answering in simulation" width="720">
</div>

<div align="center">
  <p><b>VLA_sim</b></p>
  <img src="imgs/VLA_sim.gif" alt="VLA closed-loop manipulation in simulation" width="720">
</div>

For headless or container use, swap `--gui` for `--vnc` as in **4b** (same `--driver`, `--driver-config`, and `--robot-id`).
