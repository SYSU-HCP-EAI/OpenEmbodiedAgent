> Profile: g1_navigation | Driver: G1NavigationDriver

This profile connects a Unitree G1 simulation in InternUtopia to PhyAgentOS for navigation-only tasks. It supports entering the simulator, waypoint navigation, scene export, and robot-mounted cameras.

Supported actions:
- `enter_simulation` or `start`: start the G1 simulation.
- `close`: close the simulation.
- `step`: step the environment with a raw InternUtopia action.
- `navigate_to_named`: navigate to a configured waypoint alias or key.
- `navigate_to_waypoint`: navigate to an explicit XY target.
- `describe_visible_scene`: summarize configured visible objects from the current runtime scene.
- `api_call`: call a method on the underlying G1 simulation API.

Not supported:
- `run_pick_place`
- `run_vla_pick_and_return`

Driver config fields:
- `scene_asset_path`, `robot_usd_path`, `robot_start`
- `pythonpath`, `isaac_env`
- `navigation_action_name`, `navigation_max_steps`, `navigation_threshold`
- `waypoints`, `waypoint_aliases`
- `robot_cameras`, `robot_camera_resolution`, `robot_camera_warmup_steps`, `robot_camera_debug_dump_dir`
- `visible_objects`, `objects`

Runtime channels:
- `robots.<robot_id>.connection_state`
- `robots.<robot_id>.robot_pose`
- `robots.<robot_id>.nav_state`
- `robots.<robot_id>.cameras`

Notes:
- This integration is navigation-only by design.
- SmolVLA deploy actions are intentionally disabled because the current workspace has no G1 manipulation stack or G1-specific SmolVLA control path.
- The default example mounts a body-view camera under the G1 pelvis for a stable first-person-ish view that can be refined later if needed.