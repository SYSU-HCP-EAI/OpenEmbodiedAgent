# EMBODIED Profile: Franka Simulation

Robot: `franka_001` (fixed-base manipulator in Isaac Sim)

Supported actions:
- `start` / `enter_simulation`
- `close`
- `get_robot_state`
- `grasp` / `pick` (supports driver-side auto target resolution for blue cube when `target` is omitted)
- `place` / `release` (supports driver-side auto "aside" placement when `target` is omitted)
- `grasp_and_place_aside` (atomic helper action: grasp blue cube then place aside)

Notes:
- Franka is manipulation-only; no mobile navigation actions.
- Action execution runs through HAL watchdog and updates `ENVIRONMENT.md`.
