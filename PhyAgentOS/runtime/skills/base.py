"""Common base class for HAL v3 skill runtimes."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Literal

from PhyAgentOS.runtime.sessions.models import SkillContext


class BaseSkillRuntime(ABC):
    runtime_kind: Literal["policy", "builtin"]

    @abstractmethod
    def start(self, skill_ctx: SkillContext) -> None:
        """Initialize skill-local state for one session."""

    @abstractmethod
    def cancel(self, skill_ctx: SkillContext, reason: str) -> None:
        """Cancel skill-local execution."""

    @abstractmethod
    def snapshot(self, skill_ctx: SkillContext) -> dict:
        """Return skill-local runtime state."""

    def required_environment_outputs(self, skill_ctx: SkillContext) -> list[str]:
        return []
