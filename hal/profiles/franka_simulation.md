# Robot Embodiment Declaration — Franka Simulation

> Profile: franka_simulation | Driver: FrankaSimulationDriver

## Identity

- **Name**: Franka Manipulator (Simulation)
- **Type**: 7-DOF Fixed-base Robotic Arm with 2-Finger Gripper (Isaac Sim)
- **Engine**: `FrankaManipulationAPI` (internutopia sim stack)
- **Driver**: `franka_simulation` (PhyAgentOS HAL)

## Example Configuration

Reference config: [examples/franka_simulation_driver.json](../../examples/franka_simulation_driver.json)

This config mirrors the Franka setup from `test3_multi_robot_debug.py` for multi-robot scenarios.

## Supported Actions

| Action | Parameters | Description |
|--------|-----------|-------------|
| `start` | `scene_asset_path`, `robot_position` | Start simulation API and load USD scene |
| `enter_simulation` | same as `start` | **Preferred user-facing name** |
| `close` | — | Close simulation and release resources |
| `step` | `action?` | Single environment step |
| `api_call` | `method`, `args?`, `kwargs?` | Direct API method call |
| `grasp` / `pick` | `target`, `dump_name?`, `output_dir?` | Execute grasp motion to target |
| `place` / `release` | `target`, `dump_name?`, `output_dir?` | Execute place motion to target |
| `register_grasp_target` | `name`, `target` | Register a named grasp target |
| `register_place_target` | `name`, `target` | Register a named place target |

## Target Format

Manipulation targets are dicts with these keys:

| Key | Type | Description |
|-----|------|-------------|
| `position` | `[x, y, z]` | Target position (meters) |
| `orientation` | `[x, y, z, w]` | Quaternion (optional, default: down) |
| `pre_position` | `[x, y, z]` | Pre-approach position (optional) |
| `post_position` | `[x, y, z]` | Post-retreat position (optional) |
| `name` | `str` | Target name (optional) |
| `metadata` | `dict` | Additional data (optional) |

## Driver-config keys

```json
{
  "scene_asset_path": "/path/to/scene.usd",
  "robot_position": [0.0, 0.0, 0.0],
  "grasp_targets": {
    "cube": {
      "position": [0.4, 0.0, 0.1],
      "orientation": [0.0, 0.0, 1.0, 0.0],
      "pre_position": [0.4, 0.0, 0.3],
      "post_position": [0.4, 0.0, 0.3]
    }
  },
  "place_targets": {
    "pedestal": {
      "position": [0.4, -0.3, 0.1],
      "orientation": [0.0, 0.0, 1.0, 0.0]
    }
  },
  "pause_steps": 45,
  "gripper_settle_steps": 30,
  "arm_waypoint_count": 8,
  "pythonpath": ["/path/to/internutopia"],
  "output_dir": "/tmp/paos_franka_sim_logs"
}
```

## Notes

- Unlike `pipergo2_manipulation` (mobile robot), this is a **fixed-base arm** — no navigation actions.
- Internutopia must be installed in the same environment, or `pythonpath` must point to its root.
- The `start` / `enter_simulation` action loads the USD scene and initializes the Franka arm.
- `grasp` and `place` execute full pick/place motion sequences with approach, grip, and retreat phases.

## Safety Notes

- Dispatch `start` / `enter_simulation` before any manipulation actions.
- `grasp` / `place` execute real sim motion; validate target positions.
- Dispatch `close` after a session to release the simulator.
