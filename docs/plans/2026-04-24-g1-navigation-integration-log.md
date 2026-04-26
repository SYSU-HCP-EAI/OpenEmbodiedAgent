# 2026-04-24 Night Log: G1 Navigation Integration

## What Was Done Tonight

1. Added a new HAL driver for G1 navigation-only simulation integration.
- File: hal/drivers/g1_navigation_driver.py
- Driver name: g1_navigation
- Scope: enter simulation, waypoint navigation, scene export, robot camera attach
- Explicitly disabled manipulation/VLA deploy actions for this driver

2. Registered the new driver in the HAL driver registry.
- File: hal/drivers/__init__.py
- Added registry mapping:
  - g1_navigation -> hal.drivers.g1_navigation_driver.G1NavigationDriver

3. Added G1 profile and configs.
- Profile doc: hal/profiles/g1_navigation.md
- Example config: examples/g1_navigation_driver.json
- Template config: PhyAgentOS/templates/configs/g1_navigation_driver.json

4. Copied provided G1 assets into workspace-local InternUtopia path at the same URDF level as PiperGo2-style assets.
- Source:
  - /data/datasets/GRScenes/robots/g1
- Destination:
  - /data/wanghanting/codes/InternUtopia/internutopia/demo/urdf/g1
- Copied key files:
  - g1_29dof_color.usd
  - policy/move_by_speed/g1_15000.onnx

5. Made the new G1 driver prefer local copied policy assets when robot USD is from local demo/urdf/g1.

6. Added focused tests.
- New tests: tests/test_g1_navigation_driver.py
- Extended test: tests/test_hal_watchdog_driver_config.py

7. Validation completed.
- Pytest command run:
  - /data/wanghanting/miniconda3/envs/paos/bin/python -m pytest tests/test_g1_navigation_driver.py tests/test_hal_watchdog_driver_config.py -q
- Result:
  - 9 passed
- JSON config parse check completed for both new G1 config files.

## Run Commands (G1)

1. Start G1 navigation driver in VNC mode:

cd /data/wanghanting/codes/PhyAgentOS && /data/wanghanting/miniconda3/envs/paos/bin/python -m hal.hal_watchdog --vnc --interval 0.05 --driver g1_navigation --driver-config examples/g1_navigation_driver.json

2. Start G1 navigation driver in GUI mode:

cd /data/wanghanting/codes/PhyAgentOS && /data/wanghanting/miniconda3/envs/paos/bin/python -m hal.hal_watchdog --gui --interval 0.05 --driver g1_navigation --driver-config examples/g1_navigation_driver.json

## PiperGo2 Work Recap (Same Session)

1. Added embodied question answering integration and tool registration for simulator-visible context.

2. Extended runtime handling to support multi-robot/fleet style runtime state organization.

3. Stabilized PiperGo2 simulation runtime path.
- Improved scene/object export behavior used by ENVIRONMENT state updates.
- Hardened pick/place guard logic against stale or moved targets.

4. Added robot camera support for PiperGo2 and finalized camera simplification.
- Kept only dog_view as user-facing robot camera.
- Removed arm_view (arm perception already handled by VLA wrist pipeline).

5. Debugged and fixed dog_view rendering issue (white/black frame cases).
- Added first-frame camera dump support to debug real camera outputs.
- Verified final working camera orientation and runtime camera attach status.

6. Verified critical PiperGo2 regression tests in-session.

## Run Commands (PiperGo2)

1. Start PiperGo2 manipulation driver with VNC:

cd /data/wanghanting/codes/PhyAgentOS && /data/wanghanting/miniconda3/envs/paos/bin/python -m hal.hal_watchdog --vnc --interval 0.05 --driver pipergo2_manipulation --driver-config examples/pipergo2_manipulation_driver.json

2. Start PiperGo2 manipulation driver with GUI:

cd /data/wanghanting/codes/PhyAgentOS && /data/wanghanting/miniconda3/envs/paos/bin/python -m hal.hal_watchdog --gui --interval 0.05 --driver pipergo2_manipulation --driver-config examples/pipergo2_manipulation_driver.json

## GitHub CLI Commands (Repo)

Use these in /data/wanghanting/codes/PhyAgentOS:

1. Check changes:

git status

2. Stage files:

git add hal/drivers/g1_navigation_driver.py hal/drivers/__init__.py hal/profiles/g1_navigation.md examples/g1_navigation_driver.json PhyAgentOS/templates/configs/g1_navigation_driver.json tests/test_g1_navigation_driver.py tests/test_hal_watchdog_driver_config.py docs/plans/2026-04-24-g1-navigation-integration-log.md

3. Commit:

git commit -m "Add g1_navigation driver, configs, tests, and integration log"

4. Push (replace branch name if needed):

git push origin HEAD

### Optional: include PiperGo2-related updates in same commit

If you modified PiperGo2 files in this session, include them before commit:

git add hal/drivers/pipergo2_manipulation_driver.py hal/simulation/vla_pick.py hal/profiles/pipergo2_manipulation.md examples/pipergo2_manipulation_driver.json PhyAgentOS/templates/configs/pipergo2_manipulation_driver.json tests/test_embodied_question.py

## Note

If you also want to version the copied G1 assets in InternUtopia, run the same git workflow in:
- /data/wanghanting/codes/InternUtopia
