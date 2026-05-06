"""Base class for runtime skill backends."""

from __future__ import annotations

from abc import ABC, abstractmethod

from PhyAgentOS.runtime.schemas import SessionResult, SessionSpec


class BaseSkillRuntime(ABC):
    @abstractmethod
    def run(self, session: SessionSpec, target, adapter, policy_client) -> SessionResult:
        """Execute one session."""
