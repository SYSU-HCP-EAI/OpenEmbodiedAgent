"""Base rollout target definitions."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any


class BaseRolloutTarget(ABC):
    """Contract shared by all runtime rollout targets."""

    @abstractmethod
    def build(self) -> None:
        """Initialize target resources."""

    @abstractmethod
    def reset(self, session_ctx: dict[str, Any]) -> dict[str, Any]:
        """Reset the target for a session and return the initial observation."""

    @abstractmethod
    def observe(self) -> dict[str, Any]:
        """Return the latest target observation."""

    def observe_for_environment(self, session_ctx: dict[str, Any]) -> dict[str, Any]:
        """Return a read-only observation for environment/perception refresh."""
        return self.observe()

    @abstractmethod
    def step(self, action: Any) -> dict[str, Any]:
        """Apply one target-level action and return transition data."""

    def configure_session(self, session_ctx: dict[str, Any]) -> dict[str, Any]:
        """Configure the target after preflight acceptance."""
        return {"configured": True, "session_id": session_ctx.get("session_id")}

    def action_chunk(self, executable_action_chunk: dict[str, Any]) -> dict[str, Any]:
        """Apply an executable action chunk and return target execution status."""
        raise NotImplementedError("target does not implement action_chunk")

    def execution_status(self) -> dict[str, Any]:
        """Return the latest target execution status."""
        return {"accepted": True, "safety_status": "ok"}

    @abstractmethod
    def close(self) -> None:
        """Release target resources."""
