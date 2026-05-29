"""Minecraft skill runtime: drives an episode on a MinecraftTarget."""

from __future__ import annotations

import logging
import time
from typing import Any

from PhyAgentOS.runtime.schemas import AdapterPlan, SessionResult, SessionSpec
from PhyAgentOS.runtime.skills.base import BaseSkillRuntime
from PhyAgentOS.runtime.watchdog.errors import SessionTimeoutError

logger = logging.getLogger(__name__)


class MinecraftSkillRuntime(BaseSkillRuntime):
    """Execute a Minecraft episode without VLA policy.

    Reads an action plan from ``session.runtime_hints`` (or falls back to a
    single-step observe-then-wait loop).  The runtime iterates ``observe ->
    pick_action -> step`` until ``done``, ``success``, or ``max_steps``.
    """

    runtime_kind = "builtin"

    def start(self, skill_ctx) -> None:
        pass

    def cancel(self, skill_ctx, reason: str) -> None:
        pass

    def snapshot(self, skill_ctx) -> dict:
        return {"status": "idle"}

    def run(
        self,
        session: SessionSpec,
        target,
        target_adapter,
        policy_adapter,
        action_bridges,
        policy_client,
        adapter_plan: AdapterPlan,
    ) -> SessionResult:
        target.build()
        session_ctx = session.model_dump(mode="json")
        session_ctx["adapter_plan"] = adapter_plan.model_dump(mode="json")

        target.configure_session({
            "session_id": session.session_id,
            "task_description": session.task_description,
            "target_ref": session.target_ref,
            "skill_ref": session.skill_ref,
        })

        raw_obs = target.reset(session_ctx)
        num_steps = 0
        total_reward = 0.0
        start_time = time.monotonic()
        timeout_s = session.timeouts.execute_timeout_s

        action_plan: list[dict[str, Any]] = _extract_action_plan(session)

        for step_idx in range(session.execution.max_steps):
            if time.monotonic() - start_time > timeout_s:
                raise SessionTimeoutError(
                    f"session {session.session_id} exceeded {timeout_s}s"
                )

            target_info = {
                "step_index": step_idx,
                "task_description": session.task_description,
            }
            runtime_obs = target_adapter.to_runtime_observation(raw_obs, target_info)

            action = _pick_action(action_plan, step_idx, runtime_obs)

            bridged_action = action
            for bridge in action_bridges:
                bridged_action = bridge.apply(bridged_action, target_info)

            transition = target.step(bridged_action)
            num_steps += 1
            raw_obs = transition.get("obs", target.observe())
            total_reward += float(transition.get("reward", 0.0))

            if bool(transition.get("done", False)) or bool(
                transition.get("info", {}).get("success", False)
            ):
                success = bool(transition.get("info", {}).get("success", False))
                return SessionResult(
                    status="succeeded" if success else "failed",
                    success=success,
                    num_steps=num_steps,
                    return_value=total_reward,
                    metadata={"done": True},
                )

        return SessionResult(
            status="failed",
            success=False,
            num_steps=num_steps,
            return_value=total_reward,
            error_code="MAX_STEPS_EXCEEDED",
            error_message="session reached max_steps without success",
        )


def _extract_action_plan(session: SessionSpec) -> list[dict[str, Any]]:
    hints = session.runtime_hints
    queries = hints.perception_queries if hints else []
    plan: list[dict[str, Any]] = []
    for q in queries:
        if isinstance(q, dict) and "type" in q:
            plan.append(q)
    return plan


def _pick_action(
    action_plan: list[dict[str, Any]],
    step_idx: int,
    runtime_obs: dict[str, Any],
) -> dict[str, Any]:
    if action_plan and step_idx < len(action_plan):
        return action_plan[step_idx]
    return {"type": "look", "params": {"yaw": step_idx * 15.0 % 360, "pitch": 0.0}}
