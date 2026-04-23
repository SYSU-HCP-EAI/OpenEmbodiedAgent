# Tool Usage Notes

Tool signatures are provided automatically via function calling.
This file documents non-obvious constraints and usage patterns.

## exec — Safety Limits

- Commands have a configurable timeout (default 60s)
- Dangerous commands are blocked (rm -rf, format, dd, shutdown, etc.)
- Output is truncated at 10,000 characters
- `restrictToWorkspace` config can limit file access to the workspace

## cron — Scheduled Reminders

- Please refer to cron skill for usage.

## execute_robot_action — HAL / simulation (Track B)

Physical execution is **not** inside this process. A separate terminal must run the hardware watchdog, for example:

`python hal/hal_watchdog.py --gui --driver pipergo2_manipulation --driver-config examples/pipergo2_manipulation_driver.json`

Use **`--gui`** so Isaac/Omniverse can open a window (the driver also sets `force_gui` when `--gui` is passed).

That process installs `EMBODIED.md` from the driver profile and polls `ACTION.md`.

**Manipulation sim workflow** (after watchdog is running):

1. **`enter_simulation`** (or **`start`**) with **`parameters: {}`** when the watchdog was started with `--driver-config …/pipergo2_manipulation_driver.json`. The driver already holds `scene_asset_path` and objects; you do **not** need to pass them again. (Do **not** use the path `examples/...` from the Agent: that file lives under the git repo, not under `~/.PhyAgentOS/workspace`. For reading defaults, use **`configs/pipergo2_manipulation_driver.json`** inside the workspace after `paos onboard`.)
2. **`navigate_to_named`** — e.g. `{"waypoint_key": "desk"}` or `"staging_table"` to move near the table (aliases come from driver-config `waypoint_aliases`).
3. **`describe_visible_scene`** — user asks what is on the table; reply using the tool result / `ENVIRONMENT.md` `manipulation_runtime.table_summary_cn`.
4. **`run_pick_place`** — pick/place; use `"execute_place": false` for pick-only.
5. **`close`** when the session should tear down the sim.

If the user says “go to desk” / “走到桌子” and `EMBODIED.md` lists these actions, call **`execute_robot_action`** with the steps above instead of asking whether they meant a robot.
