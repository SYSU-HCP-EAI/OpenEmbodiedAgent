"""Base class for policy-backed skill runtimes."""

from __future__ import annotations

from abc import abstractmethod
from typing import Literal

from PhyAgentOS.runtime.policy.base_client import BasePolicyClient
from PhyAgentOS.runtime.schemas import AdapterPlan
from PhyAgentOS.runtime.sessions.models import SkillContext, SkillRuntimeResult
from PhyAgentOS.runtime.skills.base import BaseSkillRuntime


class PolicySkillRuntime(BaseSkillRuntime):
    runtime_kind: Literal["policy"] = "policy"

    @abstractmethod
    def run_policy_loop(
        self,
        skill_ctx: SkillContext,
        target_handle,
        adapter_plan: AdapterPlan,
        policy_client: BasePolicyClient,
    ) -> SkillRuntimeResult:
        """Run the policy-driven observation/action loop."""
