"""Adapter and bridge base classes."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any


class BaseTargetAdapter(ABC):
    @abstractmethod
    def to_runtime_observation(self, raw_obs: dict[str, Any], target_info: dict[str, Any]) -> dict[str, Any]:
        """Convert target raw observation to canonical RuntimeObservation."""

    @abstractmethod
    def to_executable_action_chunk(
        self,
        action_chunk: dict[str, Any],
        target_info: dict[str, Any],
    ) -> dict[str, Any]:
        """Convert a bridged action chunk into target-executable action chunk."""


class BasePolicyAdapter(ABC):
    @abstractmethod
    def to_policy_input(
        self,
        runtime_observation: dict[str, Any],
        session_ctx: dict[str, Any],
    ) -> dict[str, Any]:
        """Convert RuntimeObservation to policy-specific input payload."""

    @abstractmethod
    def from_policy_output(
        self,
        policy_output: dict[str, Any],
        session_ctx: dict[str, Any],
    ) -> dict[str, Any]:
        """Convert policy output into typed PolicyActionChunk."""


class BaseActionBridge(ABC):
    @abstractmethod
    def apply(self, action_chunk: dict[str, Any], target_info: dict[str, Any]) -> dict[str, Any]:
        """Apply a deterministic action conversion."""
