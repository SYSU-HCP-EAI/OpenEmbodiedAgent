# Robot Embodiment Declaration — G1 Simulation

> Profile: g1_simulation | Driver: G1SimulationDriver

## Identity

- **Name**: G1 Humanoid Robot (Simulation)
- **Type**: 7-DOF Humanoid Robot with Bipedal Locomotion (Isaac Sim)
- **Engine**: `create_g1_robot_cfg` + `move_to_point` controller (internutopia sim stack)
- **Driver**: `g1_simulation` (PhyAgentOS HAL)

## Example Configuration

Reference config: [examples/g1_simulation_driver.json](../../examples/g1_simulation_driver.json)

This config mirrors the G1 setup from `test3_multi_robot_debug.py` for multi-robot scenarios.

## Supported Actions

| Action | Parameters | Description |
|--------|-----------|-------------|
| `start` | `scene_asset_path`, `robot_position` | Start simulation API and load USD scene |
| `enter_simulation` | same as `start` | **Preferred user-facing name** |
| `close` | — | Close simulation and release resources |
| `step` | `action?` | Single environment step |
| `api_call` | `method`, `args?`, `kwargs?` | Direct API method call |
| `navigate_to_waypoint` | `waypoint_xy`, `max_steps?`, `threshold?` | Navigate to [x, y] position |
| `navigate_to_named` | `waypoint_key` or `target`, `max_steps?`, `threshold?` | Navigate to registered waypoint |

## Waypoint Format

Waypoints are [x, y] positions stored in driver config:

```json
{
  "waypoints": {
    "home": [0.0, 0.0],
    "location_a": [2.0, 3.0]
  },
  "waypoint_aliases": {
    "start": "home"
  }
}
```

## Driver-config keys

```json
{
  "scene_asset_path": "/path/to/scene.usd",
  "robot_position": [6.0, 7.5, 0.78],
  "waypoints": {
    "home": [0.0, 0.0],
    "franka_nearby": [3.0, 8.0]
  },
  "waypoint_aliases": {
    "start": "home"
  },
  "navigation_action_name": "move_to_point",
  "navigation_max_steps": 1500,
  "navigation_threshold": 0.10,
  "pythonpath": ["/path/to/internutopia"],
  "output_dir": "/tmp/paos_g1_sim_logs",
  "idle_step_enabled": true,
  "idle_steps_per_cycle": 1,
  "idle_step_interval_s": 0.033
}
```

## Notes

- Unlike `pipergo2_manipulation` or `franka_simulation`, G1 is a **locomotion-only** robot — no manipulation actions.
- G1 uses `move_to_point` action for planar navigation (x, y only; z is fixed at ground level).
- Waypoints can be registered at runtime via config or aliased for convenience.
- Internutopia must be installed in the same environment, or `pythonpath` must point to its root.

## Safety Notes

- Dispatch `start` / `enter_simulation` before any navigation actions.
- `navigate_to_waypoint` / `navigate_to_named` execute real sim motion; validate target positions.
- Dispatch `close` after a session to release the simulator.