> Profile: xlerobot_sim | Driver: XLerobotSimDriver

This profile connects the local `xlerobot` USD asset to InternUtopia and exposes a first-pass mobile-manipulation simulator through PhyAgentOS.

Supported actions:
- `enter_simulation` or `start`: start the XLerobot simulation.
- `close`: close the simulation.
- `step`: step the environment with a raw action dictionary.
- `set_joint_targets`: directly set one or more articulation joints.
- `set_gripper`: set left, right, or both jaw joints.
- `describe_visible_scene`: summarize configured visible objects from the current runtime scene.
- `api_call`: call a method on the underlying XLerobot simulation API.

Not supported yet:
- `navigate_to_named` (disabled by default)
- `navigate_to_waypoint` (disabled by default)
- `run_pick_place`
- `run_vla_pick_and_return`

Driver config fields:
- `scene_asset_path`, `robot_usd_path`, `robot_start`
- `pythonpath`, `isaac_env`
- `waypoints`, `waypoint_aliases`
- `visible_objects`, `objects`

Runtime channels:
- `robots.<robot_id>.connection_state`
- `robots.<robot_id>.robot_pose`
- `robots.<robot_id>.nav_state`

Notes:
- This is a first-pass simulator bridge for dual-arm control with manipulation-oriented joint control.
- The asset exposes base motion through root prismatic joints (`root_x_axis_joint`, `root_y_axis_joint`, `root_z_rotation_joint`) rather than a stable locomotion controller.
- Because direct waypoint writes can destabilize the articulation and blow the robot/camera out of scene, navigation is disabled by default. Only re-enable it for debugging with `experimental_navigation_enabled: true`.
- Manipulation currently uses direct joint and jaw control; high-level autonomous pick-place is not wired yet.