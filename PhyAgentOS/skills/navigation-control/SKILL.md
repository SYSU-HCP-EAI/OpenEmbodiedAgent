---
name: navigation-control
description: Route robot movement intents like move to table/desk/home to execute_robot_action navigate_to_named. Use for phrases like move to the table, go to desk, navigate home, and related Chinese equivalents.
metadata: {"nanobot":{"emoji":"🧭"}}
---

# Navigation Control Skill

Use this skill whenever the user asks the robot to move or navigate to a named place.

## Rules

- Always dispatch movement requests through `execute_robot_action`.
- For table/desk intents, call `execute_robot_action` with:
  - `action_type`: `navigate_to_named`
  - `parameters`: `{"waypoint_key": "table"}`
  - `reasoning`: short reason
- For home/start intents, call `execute_robot_action` with:
  - `action_type`: `navigate_to_named`
  - `parameters`: `{"waypoint_key": "home"}`
  - `reasoning`: short reason

## Trigger Phrases (examples)

- `move to the table`
- `move to table`
- `go to the table`
- `go to desk`
- `navigate to table`
- `go home`
- `move to home`
- `回到桌子旁`
- `移动到桌子`
- `回到起点`

## Robot Selection

- In single mode, omit `robot_id`.
- In fleet mode, include `robot_id` only when user explicitly names the robot.

## Safety

- Do not answer from stale ENVIRONMENT text alone.
- Navigation requests must trigger a real action call.
- If tool returns API-not-started errors, surface the exact error and suggest opening simulation.
