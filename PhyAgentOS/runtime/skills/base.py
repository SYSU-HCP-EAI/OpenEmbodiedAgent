"""Base class for runtime skill backends."""

from __future__ import annotations

from abc import ABC, abstractmethod

from PhyAgentOS.runtime.schemas import AdapterPlan, SessionResult, SessionSpec


class BaseSkillRuntime(ABC):
    @abstractmethod
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
        """Execute one session."""
