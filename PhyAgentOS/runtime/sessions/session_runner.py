"""Single-session lifecycle shell."""

from __future__ import annotations

import time
from uuid import uuid4

from PhyAgentOS.runtime.adapters.factory import build_adapter_stack
from PhyAgentOS.runtime.perception.config_resolver import ResolvedPerceptionPlan
from PhyAgentOS.runtime.perception.perception_runtime import PerceptionRuntime
from PhyAgentOS.runtime.policy.base_client import BasePolicyClient
from PhyAgentOS.runtime.schemas import AdapterPlan, SessionResult, SessionSpec, SkillSpec, TargetSpec, TargetToolManifest
from PhyAgentOS.runtime.sessions.models import SessionState, SkillContext, SkillRuntimeResult
from PhyAgentOS.runtime.sessions.target_session_handle import TargetSessionHandle
from PhyAgentOS.runtime.skills.builtin import BuiltinSkillRuntime
from PhyAgentOS.runtime.skills.policy import PolicySkillRuntime
from PhyAgentOS.runtime.watchdog.errors import PolicyProtocolError


class SessionRunner:
    def __init__(
        self,
        *,
        session: SessionSpec,
        target_spec: TargetSpec,
        skill_spec: SkillSpec,
        adapter_plan: AdapterPlan,
        target,
        skill_runtime,
        policy_client: BasePolicyClient | None,
        perception_runtime: PerceptionRuntime | None,
        perception_plan: ResolvedPerceptionPlan | None,
        target_tool_manifest: TargetToolManifest | None = None,
    ):
        self.session = session
        self.target_spec = target_spec
        self.skill_spec = skill_spec
        self.adapter_plan = adapter_plan
        self.target = target
        self.skill_runtime = skill_runtime
        self.policy_client = policy_client
        self.perception_runtime = perception_runtime
        self.perception_plan = perception_plan
        self.target_tool_manifest = target_tool_manifest
        self.state = SessionState(session_id=session.session_id, trace_id=f"trace_{uuid4().hex[:12]}")

    def start(self) -> SessionResult:
        self.state.started_at_ns = time.time_ns()
        self.state.heartbeat()
        target_adapter, _, action_bridges = build_adapter_stack(self.adapter_plan)
        self.target.build()
        self.state.heartbeat()
        session_ctx = self._session_context()
        self.target.configure_session(session_ctx)
        self.state.heartbeat()
        self.target.start_session(session_ctx)
        self.state.heartbeat()
        initial_observation = self.target.reset(self.session.model_dump(mode="json"))
        self.state.heartbeat()
        handle = TargetSessionHandle(
            session=self.session,
            target_spec=self.target_spec,
            skill_spec=self.skill_spec,
            target=self.target,
            target_adapter=target_adapter,
            action_bridges=action_bridges,
            adapter_plan=self.adapter_plan,
            session_state=self.state,
            perception_runtime=self.perception_runtime,
            perception_plan=self.perception_plan,
            target_tool_manifest=self.target_tool_manifest,
            initial_observation=initial_observation,
        )
        skill_ctx = SkillContext(
            session=self.session,
            target=self.target_spec,
            skill=self.skill_spec,
            task_description=self.session.task_description,
            metadata={"trace_id": self.state.trace_id},
        )
        self.skill_runtime.start(skill_ctx)
        self.state.heartbeat()
        if self.skill_spec.runtime_kind == "policy":
            if self.policy_client is None:
                raise PolicyProtocolError("policy runtime requires a policy client")
            if not isinstance(self.skill_runtime, PolicySkillRuntime):
                raise PolicyProtocolError("registered skill runtime is not a PolicySkillRuntime")
            result = self.skill_runtime.run_policy_loop(skill_ctx, handle, self.adapter_plan, self.policy_client)
        else:
            if not isinstance(self.skill_runtime, BuiltinSkillRuntime):
                raise PolicyProtocolError("registered skill runtime is not a BuiltinSkillRuntime")
            result = self.skill_runtime.run_builtin_loop(skill_ctx, handle, self.adapter_plan)
        self.state.completed_at_ns = time.time_ns()
        self.state.heartbeat()
        return self._to_session_result(result)

    def cancel(self, reason: str) -> None:
        self.state.cancelled = True
        self.state.heartbeat()
        self.target.cancel(reason)

    def snapshot(self) -> dict:
        return {
            "session_id": self.session.session_id,
            "step_index": self.state.step_index,
            "cancelled": self.state.cancelled,
            "last_status": self.state.last_status,
            "started_at_ns": self.state.started_at_ns,
            "last_heartbeat_ns": self.state.last_heartbeat_ns,
            "completed_at_ns": self.state.completed_at_ns,
        }

    def close(self) -> None:
        self.target.close()

    def _session_context(self) -> dict:
        return {
            "session_id": self.session.session_id,
            "task_description": self.session.task_description,
            "target_ref": self.session.target_ref,
            "skill_ref": self.session.skill_ref,
            "adapter_plan": self.adapter_plan.model_dump(mode="json"),
            "execution": self.session.execution.model_dump(mode="json"),
            "safety_profile": self.session.safety_profile.model_dump(mode="json"),
        }

    def _to_session_result(self, result: SkillRuntimeResult) -> SessionResult:
        session_result = result.to_session_result()
        if session_result.num_steps is None:
            session_result.num_steps = self.state.step_index
        return session_result
