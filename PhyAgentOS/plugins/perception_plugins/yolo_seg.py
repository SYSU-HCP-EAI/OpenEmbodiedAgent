"""YOLO segmentation plugin placeholder with lazy dependency loading."""

from __future__ import annotations

from PhyAgentOS.plugins.perception_plugins.base import BasePerceptionPlugin
from PhyAgentOS.runtime.perception.sensor_frame import SensorFrame
from PhyAgentOS.runtime.schemas.perception import EnvironmentDelta
from PhyAgentOS.runtime.watchdog.errors import SchemaValidationError


class YOLOSegPlugin(BasePerceptionPlugin):
    def preflight(self) -> None:
        raise SchemaValidationError("YOLO plugin requires optional dependencies in this MVP")

    def run(self, frame: SensorFrame, context: dict) -> EnvironmentDelta:
        raise SchemaValidationError("YOLO plugin is not available in dependency-free MVP")

