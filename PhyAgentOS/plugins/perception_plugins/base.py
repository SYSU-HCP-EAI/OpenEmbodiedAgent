"""Base perception plugin protocol."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

from PhyAgentOS.runtime.perception.sensor_frame import SensorFrame
from PhyAgentOS.runtime.schemas.perception import EnvironmentDelta


class BasePerceptionPlugin(ABC):
    def __init__(self, plugin_id: str, config: dict[str, Any] | None = None):
        self.plugin_id = plugin_id
        self.config = dict(config or {})

    def preflight(self) -> None:
        """Perform plugin-local preflight. Heavy dependencies must be imported lazily here."""

    @abstractmethod
    def run(self, frame: SensorFrame, context: dict[str, Any]) -> EnvironmentDelta:
        """Run the plugin and return compact environment delta data."""

