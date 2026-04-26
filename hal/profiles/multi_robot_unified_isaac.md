# Multi Robot Unified Isaac Driver

- **Type**: Simulation (InternUtopia + Isaac Sim)
- **Scope**: Single watchdog, single Env, multi-robot same scene
- **Default IDs**: `pipergo2_001`, `franka_001`, `g1_001`

## Supported Actions

- `start` / `enter_simulation`
- `close`
- `step`
- `api_call`
- `navigate_to_named` (requires `robot_id`, supports `pipergo2` / `g1`)
- `navigate_to_waypoint` (requires `robot_id`, supports `pipergo2` / `g1`)
- `pick` / `grasp` / `place` / `release` (requires `robot_id=franka_001`, currently placeholder response)

## Notes

- For responsiveness in GUI, this driver defaults to:
  - `rendering_interval = 5`
  - `idle_step_enabled = false`
- Prefer one pending `enter_simulation` action only.
