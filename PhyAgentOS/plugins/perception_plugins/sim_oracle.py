"""Debug-only sim oracle plugin."""

from __future__ import annotations

from PhyAgentOS.plugins.perception_plugins.base import BasePerceptionPlugin
from PhyAgentOS.runtime.perception.sensor_frame import SensorFrame
from PhyAgentOS.runtime.schemas.perception import EnvironmentDelta, EnvironmentObject, EnvironmentObjectSource


class SimOraclePlugin(BasePerceptionPlugin):
    def run(self, frame: SensorFrame, context: dict) -> EnvironmentDelta:
        objects = self.config.get("objects", {})
        delta = EnvironmentDelta(generated_outputs=["objects_3d", "scene_graph"])
        if not isinstance(objects, dict):
            return delta
        for object_id, payload in objects.items():
            if not isinstance(payload, dict):
                continue
            delta.objects[object_id] = EnvironmentObject(
                label=str(payload.get("label", object_id)),
                confidence=float(payload.get("confidence", 1.0)),
                source=EnvironmentObjectSource(
                    target_id=frame.target_id,
                    session_id=frame.session_id,
                    source_plugin_id=self.plugin_id,
                ),
                pose=payload.get("pose"),
                geometry=payload.get("geometry"),
            )
            delta.refresh_scope.append(object_id)
        return delta

