"""Adapter base classes."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any


class BaseTargetAdapter(ABC):
    @abstractmethod
    def make_observation(
        self,
        raw_obs: dict[str, Any],
        step_idx: int,
        target_info: dict[str, Any],
    ) -> dict[str, Any]:
        """Convert target observation to policy observation."""
