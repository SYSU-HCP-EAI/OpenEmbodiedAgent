# Action Queue

Write one pending action (or a queue) in the JSON block below.
HAL Watchdog will execute the first action with `status = pending`.

```json
{
  "schema_version": "PhyAgentOS.action_queue.v1",
  "actions": [
    {
      "id": "start_demo",
      "action_type": "start",
      "parameters": {
        "scene_asset_path": "/path/to/your_scene.usd",
        "robot_start": [0.58, 7.50704, 0.55],
        "arm_mass_scale": 0.25,
        "api_kwargs": {
          "force_gui": true,
          "pause_steps": 90,
          "arm_settle_steps": 90,
          "arm_motion_steps": 150,
          "navigation_offset": 0.42,
          "enable_arm_ik": true,
          "allow_arm_heuristic_fallback": false
        }
      },
      "status": "pending"
    }
  ]
}
```

For direct manipulation API calls, use:
- `action_type`: `api_call`
- `parameters.method`: e.g. `pick`, `release`
- `parameters.args` / `parameters.kwargs`: passthrough method args
