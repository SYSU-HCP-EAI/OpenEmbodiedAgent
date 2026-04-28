# Robot Embodiment Declaration — PiperGo2 Manipulation (Sim)

> Profile: pipergo2_manipulation | Driver: PiperGo2ManipulationDriver

## Identity

- **Name**: Piper + Go2 mobile manipulation (simulation)
- **Type**: Mobile manipulator with USD scene + physics
- **Engine**: `PiperGo2ManipulationAPI` (vendor sim stack in the active Python env)

## Supported Actions

| Action | Parameters | Description |
|--------|-----------|-------------|
| `start` | same as `enter_simulation` | Start manipulation API + optional room bootstrap |
| `enter_simulation` | same as `start` | **Preferred user-facing name**: load USD scene and enter sim (requires HAL watchdog running with this driver) |
| `navigate_to_named` | `waypoint_key` or `target` (e.g. `staging_table`, `desk` if aliased) | Navigate to a named XY from driver-config `waypoints` / `waypoint_aliases` |
| `navigate_to_waypoint` | `waypoint_xy: [x,y]`, `action_name?`, `max_steps?`, `threshold?` | Navigate to raw coordinates |
| `describe_visible_scene` | — | Returns Chinese scene summary from `visible_objects` in driver-config; updates `ENVIRONMENT` snapshot |
| `run_pick_place` | `execute_place?`, `return_home_after_place?`, `navigate_after_pick?`, `navigate_after_pick_xy?`, `target_color_cn?` / `color_hint?` | Run configured `pick` then optional `release`; if `execute_place` is false and defaults enable post-pick navigation, the driver uses `pick_place_defaults.navigate_after_pick_xy` when set, else the place target XY |
| `run_vla_pick_and_return` | `pick_target_prim_path?`, `pick_nav_offset?`, `home_xy?`, `task_text?`, `max_ticks?`, `cube_lift_threshold?` (all optional; override `vla` block in driver-config) | **Self-contained**: internally scoots to the cube approach XY (cube − offset on +X), runs the closed-loop SmolVLA pick (3 training cameras + ArticulationView state), force-closes the gripper, and drives home while holding that arm pose so the cube is not dropped. Caller does **not** need to pre-navigate; the handler's approach nav works from any reachable starting position. |
| `api_call` | `method`, `args?`, `kwargs?` | Direct API method call |
| `step` | `action?` | Single env step |
| `close` | — | Close API |

## Tool-call shape (`execute_robot_action`)

Put HAL fields **inside** `parameters`. If the model places `waypoint_key` / `target` at the **top level**, PhyAgentOS folds them into `parameters` automatically — nested form is still preferred.

## Conversation mapping (typical)

1. **Go to desk** -> `navigate_to_named` with `parameters: {"waypoint_key": "desk"}` or `{"target": "desk"}` (alias -> `staging_table`), or `{"waypoint_key": "staging_table"}`.
2. **What is on the table** -> `describe_visible_scene`; agent reads the returned `scene_description: ...` and/or `ENVIRONMENT.md` -> `manipulation_runtime.table_summary_cn` / `navigable_names`.
3. **Pick the red cube** / **place it on the pedestal** -> `run_pick_place` with `target_color_cn: "red"` when disambiguating; set `execute_place: true` to place on the configured pedestal, `false` to pick only (demo configs often then navigate to `navigate_after_pick_xy`, e.g. back to spawn).
4. **Deploy a VLA to pick the red cube and return to start** -> `run_vla_pick_and_return` with `{}`. Single action covers approach-scoot + SmolVLA closed-loop pick + return-home. The handler works from any reachable starting position (including spawn / `robot_home`), so a prior `navigate_to_named` is **not** required by the Critic. All numerical knobs (ckpt path, cameras, thresholds) come from `vla` block in `driver-config`.

## Driver-config keys (summary)

- `pythonpath`, `waypoints`, `visible_objects`, `pick_place`, `room_bootstrap`, `pick_place_defaults` (includes `navigate_to_place_pedestal_after_pick`, `navigate_after_pick_xy`, `return_home_after_place`), `objects`, `api_kwargs` — see `examples/pipergo2_manipulation_driver.json`.
- `vla` — closed-loop SmolVLA pick config. Keys: `ckpt_path` (path to the SmolVLA checkpoint), `task_text`, `cameras` (3 mounts matching the training rig: `camera1` chest, `camera2` gripper wrist w/ `flip_horizontal: true`, `camera3` world tripod), `cam_resolution`, `pick_target_prim_path`, `pick_nav_offset`, `home_xy`, `sim_hz`, `control_hz`, `n_action_steps`, `max_ticks`, `cube_lift_threshold`, `close_gripper_*`, `max_per_tick_delta_*`, `robot_prim_path`.
- Idle responsiveness tuning (optional): `idle_step_enabled`, `idle_steps_per_cycle`, `idle_step_interval_s`.
- UI + camera tuning (optional): `room_lighting` (`grey_studio`), `camera_eye_offset`, `camera_target_z_offset`, `camera_target_min_z`.

If `internutopia` is not installed in the same environment where watchdog runs, set `pythonpath` to the InternUtopia repo root (the directory that contains the `internutopia/` package). This fixes module discovery when watchdog is started from another working directory.

## Parameters vs HAL Watchdog `--driver-config`

When the watchdog starts with e.g.  
`python hal/hal_watchdog.py --gui --driver pipergo2_manipulation --driver-config /path/to/driver.json`,  
the driver **already loads** `scene_asset_path`, `objects`, `api_kwargs`, waypoints, etc. from that JSON. Use **`--gui`** so a graphics window can open (Isaac/Omniverse).

Therefore **`start` and `enter_simulation` may use empty parameters `{}`**. The Critic must accept `{}` in that setup. Optional parameters in the action only **override** defaults for that run.

A copy of the reference driver JSON is synced into the Agent workspace as  
`configs/pipergo2_manipulation_driver.json` (for humans / the LLM to read).  
It is **not** required to repeat those fields in every `execute_robot_action` call.

## Safety Notes

- Dispatch `start` / `enter_simulation` before navigation, scene description, or pick/place.
- `run_pick_place` executes real sim motion; Critic should validate bounds and wording.
- `run_vla_pick_and_return` handles approach + VLA pick + return home as a single action. The Critic must **not** require a prior `navigate_to_named` to pass — the handler's internal approach nav works from any reachable starting position (spawn, desk, or anywhere else). Do NOT pre-dispatch a separate `navigate_to_named` for the approach point or the home.
- Dispatch `close` after a session to release the simulator.
