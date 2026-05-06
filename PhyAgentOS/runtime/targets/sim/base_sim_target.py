"""Simulation target base class."""

from __future__ import annotations

from abc import abstractmethod
from typing import Any

import numpy as np

from PhyAgentOS.runtime.targets.base import BaseRolloutTarget


class BaseSimTarget(BaseRolloutTarget):
    @abstractmethod
    def build(self) -> None:
        """Initialize target resources."""

    @abstractmethod
    def reset(self, session_ctx: dict[str, Any]) -> dict[str, Any]:
        """Reset the simulation and return an observation."""

    @abstractmethod
    def observe(self) -> dict[str, Any]:
        """Return the latest observation."""

    @abstractmethod
    def step(self, action: np.ndarray) -> dict[str, Any]:
        """Step the simulation and return transition data."""

    @abstractmethod
    def close(self) -> None:
        """Release target resources."""
