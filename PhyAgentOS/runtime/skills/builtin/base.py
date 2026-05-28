"""Base class for builtin skill runtimes."""

from __future__ import annotations

from abc import abstractmethod
from typing import Literal

from PhyAgentOS.runtime.schemas import AdapterPlan, TargetToolManifest
from PhyAgentOS.runtime.sessions.models import SkillContext, SkillRuntimeResult
from PhyAgentOS.runtime.skills.base import BaseSkillRuntime


class BuiltinSkillRuntime(BaseSkillRuntime):
    runtime_kind: Literal["builtin"] = "builtin"

    @abstractmethod
    def run_builtin_loop(
        self,
        skill_ctx: SkillContext,
        target_handle,
        adapter_plan: AdapterPlan,
    ) -> SkillRuntimeResult:
        """Run the builtin skill loop."""

    def open_agent_loop(
        self,
        skill_ctx: SkillContext,
        target_handle,
        tool_manifest: TargetToolManifest,
    ):
        raise NotImplementedError("agent execution loop is not implemented")
