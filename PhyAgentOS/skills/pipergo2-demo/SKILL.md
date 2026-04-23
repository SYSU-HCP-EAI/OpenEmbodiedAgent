---
name: pipergo2-demo
description: Deterministic demo mapping for open sim, go to desk, then pick red cube and return to spawn.
metadata: {"PhyAgentOS":{"always":true},"nanobot":{"emoji":"🧪"}}
---

# PiperGo2 Demo Skill

This skill is a strict demo router for these sequential intents:

1. `open simulation`
2. `go to desk`
3. `pick up the red cube and return to the starting position` (rule-based pick)
4. `deploy a VLA to pick up the red cube and return to the starting position` (SmolVLA closed-loop pick)

## Preconditions

- HAL watchdog must already be running with:
  - driver: `pipergo2_manipulation`
  - driver-config: `examples/pipergo2_manipulation_driver.json` (or equivalent)
- If simulation may be cold, dispatch `enter_simulation` first.

## Intent Mapping (MUST follow)

### A) Open Simulation

When user input semantically means opening simulation (examples: `open simulation`, `start simulation`):

- call `execute_robot_action` with:
  - `action_type`: `enter_simulation`
  - `parameters`: `{}`
  - `reasoning`: short reason

### B) Go To Desk

When user input semantically means "go to desk" (examples: `go to desk`, `go near table`, `move to desk`):

- call `execute_robot_action` with:
  - `action_type`: `navigate_to_named`
  - `parameters`: `{"waypoint_key":"desk"}`
  - `reasoning`: short reason

### C) Pick Up Red Cube And Return To Start

When user input semantically means picking the red cube then driving back to the robot spawn / home
(examples: `pick up the red cube and return to the starting position`, `pick up the red cube and go back to the start`,
`抓起红方块回到出发点`, `grab the red cube and return home`; legacy wording such as
`pick up the red cube and move next to the rear pedestal` MUST map here as well — it no longer goes to the pedestal):

- call `execute_robot_action` **once** with:
  - `action_type`: `run_pick_place`
  - `parameters`: `{"target_color_cn":"red","execute_place":false}`
  - `reasoning`: short reason

Post-pick navigation to **robot_home** is driven by driver-config `pick_place_defaults.navigate_after_pick_xy` (not a second tool call).

### D) VLA Pick Up Red Cube And Return To Start

When user input semantically means using a **VLA / SmolVLA / learned policy** to pick the red cube and drive
back to spawn (examples: `deploy a vla to pick up the red cube and return to the starting position`,
`use the vla to pick the red cube and go home`, `run the vla pick`, `vla pick and return`, `让 vla 抓红方块再回来`,
`用 vla 模型抓红色方块然后回到起点`):

- call `execute_robot_action` **once** with:
  - `action_type`: `run_vla_pick_and_return`
  - `parameters`: `{}`
  - `reasoning`: short reason

Do **NOT** emit a separate `navigate_to_named` for the approach point or the home. The handler is
**self-contained**: it scoots to the cube approach pose from wherever the robot currently is, runs the SmolVLA
closed-loop pick, force-closes the gripper, then drives home while holding the arm pose. A prior "go to desk"
is **not** required — the Critic must accept this action even when `robot_xy` is still at spawn / `robot_home`.

Distinguish D (VLA) from C (rule-based):
- If the phrasing mentions `vla`, `smolvla`, `policy`, `learned`, `deploy a model`, `神经网络抓` → route to D.
- Otherwise (no VLA keyword) → route to C.

## Demo Safety Rules

- Never claim success without tool result confirmation.
- Treat HAL watchdog `Result:` semantics as source of truth.
- If tool returns `Error: API not started`, do **not** auto-start; explicitly ask user to run `open simulation` first.
- Keep responses short and operational for live demo.
