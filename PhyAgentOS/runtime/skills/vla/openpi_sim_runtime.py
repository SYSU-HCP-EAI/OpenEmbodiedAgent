"""OpenPI-style VLA runtime for simulation targets."""

from __future__ import annotations

from statistics import mean

from PhyAgentOS.runtime.schemas import AdapterPlan, SessionResult, SessionSpec
from PhyAgentOS.runtime.skills.base import BaseSkillRuntime
from PhyAgentOS.runtime.watchdog.errors import PolicyProtocolError


class OpenPISimSkillRuntime(BaseSkillRuntime):
    """Execute a sim rollout by repeatedly querying a policy client."""

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
        if policy_adapter is None:
            raise PolicyProtocolError("OpenPI runtime requires a policy adapter")
        target.build()
        target.configure_session(
            {
                "session_id": session.session_id,
                "task_description": session.task_description,
                "target_ref": session.target_ref,
                "skill_ref": session.skill_ref,
                "adapter_plan": adapter_plan.model_dump(mode="json"),
                "execution": session.execution.model_dump(mode="json"),
                "safety_profile": session.safety_profile.model_dump(mode="json"),
            }
        )
        raw_obs = target.reset(session.model_dump(mode="json"))
        latencies: list[float] = []
        total_reward = 0.0
        num_steps = 0

        action_cfg = getattr(target, "config", {}).get("action", {})
        target_info = {
            **getattr(target, "config", {}),
            "task_description": session.task_description,
            "replan_every": session.execution.replan_every_steps or session.execution.replan_every,
            "action_dim": getattr(target, "config", {}).get("action_dim", action_cfg.get("action_dim", 7)),
            "max_chunk_size": action_cfg.get("max_chunk_size", getattr(target, "config", {}).get("chunk_size", action_cfg.get("chunk_size", 4))),
            "action_contract_id": action_cfg.get("id", "dummy_delta_eef_gripper_v1"),
        }
        session_ctx = session.model_dump(mode="json")
        action_contract = session_ctx.get("skill_output_action_contract", {})
        if not action_contract:
            action_contract = {
                "action_space_id": "dummy_policy_delta_eef_gripper_v1",
                "shape": ["T", int(target_info["action_dim"])],
                "dtype": "float32",
                "normalized": False,
                "representation": "delta_eef_pose_gripper",
                "frame": "base",
                "chunk": {"policy_hz": target_info.get("control_hz", 20)},
            }
        session_ctx["policy_action_contract"] = action_contract
        session_ctx["action_dim"] = int(target_info["action_dim"])

        for step_idx in range(session.execution.max_steps):
            target_info["step_index"] = step_idx
            runtime_observation = target_adapter.to_runtime_observation(raw_obs, target_info)
            session_ctx["source_observation_id"] = runtime_observation.get("observation_id")
            policy_input = policy_adapter.to_policy_input(runtime_observation, session_ctx)
            action_payload = policy_client.infer(policy_input)
            policy_meta = action_payload.get("policy_meta", {})
            if "policy_latency_ms" in policy_meta:
                latencies.append(float(policy_meta["policy_latency_ms"]))
            policy_action_chunk = policy_adapter.from_policy_output(action_payload, session_ctx)
            if policy_action_chunk.get("actions") is None:
                raise PolicyProtocolError("policy returned an empty action buffer")

            bridged_chunk = policy_action_chunk
            for bridge in action_bridges:
                bridged_chunk = bridge.apply(bridged_chunk, target_info)
            executable_chunk = target_adapter.to_executable_action_chunk(bridged_chunk, target_info)
            status = target.action_chunk(executable_chunk)
            status = {**status, **target.execution_status()}
            num_steps = int(status.get("target_step_index", status.get("executed_steps", step_idx + 1)))
            raw_obs = status.get("obs", target.observe())
            total_reward += float(status.get("reward", 0.0))

            if bool(status.get("success", False)) or bool(status.get("done", False)):
                return SessionResult(
                    status="succeeded" if bool(status.get("success", False)) else "failed",
                    success=bool(status.get("success", False)),
                    num_steps=num_steps,
                    return_value=total_reward,
                    mean_policy_latency_ms=mean(latencies) if latencies else None,
                    metadata={"done": bool(status.get("done", False))},
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
