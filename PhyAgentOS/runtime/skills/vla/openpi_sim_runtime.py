"""OpenPI-style VLA runtime for simulation targets."""

from __future__ import annotations

from statistics import mean

from PhyAgentOS.runtime.schemas import SessionResult, SessionSpec
from PhyAgentOS.runtime.skills.base import BaseSkillRuntime
from PhyAgentOS.runtime.watchdog.errors import PolicyProtocolError


class OpenPISimSkillRuntime(BaseSkillRuntime):
    """Execute a sim rollout by repeatedly querying a policy client."""

    def run(self, session: SessionSpec, target, adapter, policy_client) -> SessionResult:
        target.build()
        raw_obs = target.reset(session.model_dump(mode="json"))
        action_buffer = []
        latencies: list[float] = []
        total_reward = 0.0
        num_steps = 0

        target_info = {
            **getattr(target, "config", {}),
            "task_description": session.task_description,
            "replan_every": session.execution.replan_every,
        }

        for step_idx in range(session.execution.max_steps):
            if adapter.should_replan(step_idx, len(action_buffer), target_info):
                observation = adapter.make_observation(raw_obs, step_idx=step_idx, target_info=target_info)
                action_payload = policy_client.infer(observation)
                policy_meta = action_payload.get("policy_meta", {})
                if "policy_latency_ms" in policy_meta:
                    latencies.append(float(policy_meta["policy_latency_ms"]))
                action_buffer = adapter.decode_action_chunk(action_payload, target_info)

            if not action_buffer:
                raise PolicyProtocolError("policy returned an empty action buffer")

            action = action_buffer.pop(0)
            transition = target.step(action)
            num_steps = step_idx + 1
            raw_obs = transition["obs"]
            total_reward += float(transition.get("reward", 0.0))
            info = transition.get("info", {})

            if bool(info.get("success", False)) or bool(transition.get("done", False)):
                return SessionResult(
                    status="succeeded" if bool(info.get("success", False)) else "failed",
                    success=bool(info.get("success", False)),
                    num_steps=num_steps,
                    return_value=total_reward,
                    mean_policy_latency_ms=mean(latencies) if latencies else None,
                    metadata={"done": bool(transition.get("done", False))},
                )

        return SessionResult(
            status="failed",
            success=False,
            num_steps=num_steps,
            return_value=total_reward,
            mean_policy_latency_ms=mean(latencies) if latencies else None,
            error_code="MAX_STEPS_EXCEEDED",
            error_message="session reached max_steps without success",
        )
