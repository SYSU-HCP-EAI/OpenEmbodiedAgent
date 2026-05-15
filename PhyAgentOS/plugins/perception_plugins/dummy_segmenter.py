"""Dependency-free 2D segmenter for tests and smoke runs."""

from __future__ import annotations

from PhyAgentOS.plugins.perception_plugins.base import BasePerceptionPlugin
from PhyAgentOS.runtime.perception.sensor_frame import SensorFrame
from PhyAgentOS.runtime.schemas.perception import EnvironmentDelta, EnvironmentObject, EnvironmentObjectSource


class DummySegmenter(BasePerceptionPlugin):
    def run(self, frame: SensorFrame, context: dict) -> EnvironmentDelta:
        label = str(self.config.get("label", "dummy_object"))
        object_id = str(self.config.get("object_id", f"obj_{label}"))
        camera_id = next(iter(frame.sensor_channels), None)
        source = EnvironmentObjectSource(
            target_id=frame.target_id,
            session_id=frame.session_id,
            source_plugin_id=self.plugin_id,
            camera_id=camera_id,
            sensor_id=camera_id,
        )
        obj = EnvironmentObject(
            label=label,
            confidence=float(self.config.get("confidence", 0.99)),
            source=source,
            bbox_2d=list(self.config.get("bbox_2d", [0.0, 0.0, 1.0, 1.0])),
        )
        return EnvironmentDelta(
            objects={object_id: obj},
            generated_outputs=["boxes_2d", "labels", "confidence"],
            refresh_scope=[object_id],
        )

