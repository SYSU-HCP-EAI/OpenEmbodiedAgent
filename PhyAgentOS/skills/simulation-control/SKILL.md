---
name: simulation-control
description: Route open simulation and start simulation requests to execute_robot_action enter_simulation. Use for phrases like open simulation, start simulation, launch the simulator, open g1 simulation, or open xlerobot simulation.
metadata: {"nanobot":{"emoji":"🕹️"}}
---

# Simulation Control Skill

Use this skill whenever the user asks to open or start a simulation.

## Rules

- Always call `execute_robot_action` with `action_type: enter_simulation`.
- Do not answer from stale `ENVIRONMENT.md` state alone.
- In single mode, omit `robot_id` and let the current watchdog / current `EMBODIED.md` profile decide the target robot.
- In fleet mode, include `robot_id` when the user explicitly names a robot.
- If the user explicitly mentions `g1` or `g1_001`, include `{"robot_id":"g1_001"}`.
- If the user explicitly mentions `piper`, `pipergo2`, or `pipergo2_manip_001`, include `{"robot_id":"pipergo2_manip_001"}`.
- If the user explicitly mentions `xlerobot` or `xlerobot_001`, include `{"robot_id":"xlerobot_001"}` when fleet mode is active.

## Response Style

After tool success, keep the response short:

- `Simulation started successfully and is ready for tasks.`
- `Simulation for <robot_id> is started and ready for tasks.`

Do not emit long Piper-specific or manipulation-specific narration unless the user explicitly asks for status.