---
name: pipergo2-demo
description: Deterministic PiperGo2 demo mapping for go to desk, answer table questions, pick red cube, and VLA pick-return tasks.
metadata: {"nanobot":{"emoji":"🧪"}}
---

# PiperGo2 Demo Skill

This skill is a strict PiperGo2 demo router for these intents:

1. `go to desk`
2. `what is on the table`
3. `pick up the red cube and return to the starting position` (rule-based pick)
4. `deploy a VLA to pick up the red cube and return to the starting position` (SmolVLA closed-loop pick)

## Preconditions

- HAL watchdog must already be running for the target robot driver.
- If multiple robots exist, prefer the one explicitly requested by user. If user does not specify, prefer an idle simulation robot over one already running.

## Intent Mapping (MUST follow)

### A) Go To Desk / Table

When user input semantically means moving to the desk/table waypoint
(examples: `go to desk`, `go near table`, `move to desk`, `move to table`, `move to the table`, `navigate to table`):

- call `execute_robot_action` with:
  - `action_type`: `navigate_to_named`
  - `parameters`: `{"waypoint_key":"table"}`
  - `reasoning`: short reason

This mapping applies to both manipulation drivers and navigation-only drivers (for example G1).

### B) What Is On The Table

When user input semantically asks what is on the table or what is visible from the current view
(examples: `what is on the table`, `what's on the table`, `桌子上有什么`, `你的视角里有什么`, `你看到了什么`, `what can you see`):

- call `answer_embodied_question` with:
  - `question`: the user's original question
  - `robot_id`: include only in fleet mode

This is read-only. Do **NOT** dispatch `execute_robot_action` for pure embodied QA.

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
- For navigation intents, do **not** reply from `ENVIRONMENT.md` runtime text alone (for example stale `nav_state=running`). You MUST dispatch a real navigation action tool call.
- Only skip redispatch when `ACTION.md` already contains a pending navigation action for the same target.
- Keep responses short and operational for live demo.

## Driver Guardrail

- Only use actions `run_pick_place` and `run_vla_pick_and_return` when target driver is `pipergo2_manipulation`.
- For navigation-only drivers (for example `g1_navigation`), do not mention manipulation/VLA success text. Keep output to simulation/navigation readiness.
