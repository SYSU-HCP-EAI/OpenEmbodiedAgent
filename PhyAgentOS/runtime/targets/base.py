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

    @abstractmethod
    def step(self, action: Any) -> dict[str, Any]:
        """Apply one target-level action and return transition data."""

    @abstractmethod
    def close(self) -> None:
        """Release target resources."""
