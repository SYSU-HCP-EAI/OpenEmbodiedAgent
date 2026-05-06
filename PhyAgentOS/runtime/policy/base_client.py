"""Policy client base class."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any


class BasePolicyClient(ABC):
    """Common interface for local dummy and remote policy clients."""

    @abstractmethod
    def infer(self, observation: dict[str, Any]) -> dict[str, Any]:
        """Return a policy payload containing at least ``actions``."""

    def close(self) -> None:
        """Release client resources."""
