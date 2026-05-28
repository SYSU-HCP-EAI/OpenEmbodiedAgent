"""Controlled target access boundary for skill runtimes."""

from __future__ import annotations

import time
from typing import Any

from PhyAgentOS.runtime.perception.config_resolver import ResolvedPerceptionPlan
from PhyAgentOS.runtime.perception.perception_runtime import PerceptionRuntime
from PhyAgentOS.runtime.schemas import AdapterPlan, SessionSpec, SkillSpec, TargetSpec, TargetToolManifest
from PhyAgentOS.runtime.sessions.models import EnvironmentRequest, EnvironmentSnapshot, RuntimeObservation, SessionState


class TargetSessionHandle:
    def __init__(
        self,
        *,
        session: SessionSpec,
        target_spec: TargetSpec,
        skill_spec: SkillSpec,
        target,
        target_adapter,
        action_bridges,
        adapter_plan: AdapterPlan,
        session_state: SessionState,
        perception_runtime: PerceptionRuntime | None = None,
        perception_plan: ResolvedPerceptionPlan | None = None,
        target_tool_manifest: TargetToolManifest | None = None,
        initial_observation: dict[str, Any] | None = None,
    ):
        self.session = session
        self.target_spec = target_spec
        self.skill_spec = skill_spec
        self.target = target
        self.target_adapter = target_adapter
        self.action_bridges = list(action_bridges)
        self.adapter_plan = adapter_plan
        self.session_state = session_state
        self.perception_runtime = perception_runtime
        self.perception_plan = perception_plan
        self.target_tool_manifest = target_tool_manifest
        self._pending_raw_observation = initial_observation

    def observe(self) -> RuntimeObservation:
        if self._empty_observation_allowed():
            return RuntimeObservation(
                observation_type="empty",
                timestamp_ns=time.time_ns(),
                data={},
                metadata={"semantics": self.target_spec.observation.empty_observation_semantics},
            )
        raw_observation = self._pending_raw_observation
        self._pending_raw_observation = None
        if raw_observation is None:
            raw_observation = self.target.observe()
        target_info = self._target_info()
        runtime_data = self.target_adapter.to_runtime_observation(raw_observation, target_info)
        return RuntimeObservation(
            observation_type=self.target_spec.observation.observation_type,
            timestamp_ns=runtime_data.get("timestamp_ns", time.time_ns()),
            data=runtime_data,
            metadata={"target_info": target_info},
        )

    def action_chunk(self, chunk: dict[str, Any]) -> dict[str, Any]:
        target_info = self._target_info()
        bridged_chunk = chunk
        for bridge in self.action_bridges:
            bridged_chunk = bridge.apply(bridged_chunk, target_info)
        executable_chunk = self.target_adapter.to_executable_action_chunk(bridged_chunk, target_info)
        status = self.target.action_chunk(executable_chunk)
        status = {**status, **self.target.execution_status()}
        self.session_state.last_status = status
        self.session_state.step_index = int(
            status.get("target_step_index", status.get("executed_steps", self.session_state.step_index))
        )
        obs = status.get("obs")
        if isinstance(obs, dict):
            self._pending_raw_observation = obs
        return status

    def execution_status(self) -> dict[str, Any]:
        status = self.target.execution_status()
        self.session_state.last_status = status
        return status

    def request_environment_refresh(self, request: EnvironmentRequest | None = None) -> EnvironmentSnapshot:
        if self.perception_runtime is None or self.perception_plan is None:
            return EnvironmentSnapshot(metadata={"skipped": "perception_not_configured"})
        raw_observation = self.target.observe_for_environment(
            {"session_id": self.session.session_id, "environment_refresh": True}
        )
        self.perception_runtime.refresh_environment(
            self.perception_plan,
            self.target,
            observation=raw_observation,
        )
        return EnvironmentSnapshot(metadata={"refreshed": True, "requested_outputs": (request or EnvironmentRequest()).requested_outputs})

    def call_target_tool(self, tool_name: str, arguments: dict[str, Any]) -> dict[str, Any]:
        if self.target_tool_manifest is None or tool_name not in self.target_tool_manifest.expose:
            return {"status": "rejected", "error_code": "TARGET_TOOL_NOT_EXPOSED", "message": tool_name}
        return self.target.call_target_tool(tool_name, arguments)

    def stop(self, reason: str) -> None:
        self.session_state.cancelled = True
        self.target.cancel(reason)

    def _empty_observation_allowed(self) -> bool:
        return (
            self.target_spec.observation.observation_type == "empty"
            and self.target_spec.observation.empty_observation_allowed
            and self.skill_spec.observation_contract.observation_type == "empty"
            and self.skill_spec.observation_contract.empty_observation_allowed
        )

    def _target_info(self) -> dict[str, Any]:
        action_cfg = self.target_spec.config.get("action", {})
        return {
            **self.target_spec.config,
            "task_description": self.session.task_description,
            "step_index": self.session_state.step_index,
            "replan_every": self.session.execution.replan_every_steps or self.session.execution.replan_every,
            "action_dim": self.target_spec.config.get("action_dim", action_cfg.get("action_dim", 7)),
            "max_chunk_size": action_cfg.get(
                "max_chunk_size",
                self.target_spec.config.get("chunk_size", action_cfg.get("chunk_size", 4)),
            ),
            "action_contract_id": action_cfg.get("id", "dummy_delta_eef_gripper_v1"),
        }
