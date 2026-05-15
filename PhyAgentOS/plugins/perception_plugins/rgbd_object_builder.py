"""Dependency-free RGB-D object builder for MVP tests."""

from __future__ import annotations

from PhyAgentOS.plugins.perception_plugins.base import BasePerceptionPlugin
from PhyAgentOS.runtime.perception.sensor_frame import SensorFrame
from PhyAgentOS.runtime.schemas.perception import EnvironmentDelta


class RGBDObjectBuilder(BasePerceptionPlugin):
    def run(self, frame: SensorFrame, context: dict) -> EnvironmentDelta:
        merged = EnvironmentDelta(generated_outputs=["objects_3d", "scene_graph"])
        for delta in context.values():
            if not isinstance(delta, EnvironmentDelta):
                continue
            for object_id, obj in delta.objects.items():
                obj.source.source_plugin_id = self.plugin_id
                obj.source.pipeline.append(self.plugin_id)
                obj.pose = {
                    "frame_id": self.config.get("output_frame", "base_link"),
                    "position_m": self.config.get("position_m", [0.0, 0.0, 0.0]),
                    "position_source": "dummy_rgbd",
                    "reliability": "test_fixture",
                }
                obj.geometry = {
                    "extent_m": self.config.get("extent_m", [0.1, 0.1, 0.1]),
                    "depth_stats_m": {"valid_ratio": 1.0},
                }
                merged.objects[object_id] = obj
                merged.refresh_scope.append(object_id)
        if len(merged.objects) >= 2:
            ids = list(merged.objects)
            merged.relations.append(
                {
                    "subject": ids[0],
                    "type": "near",
                    "object": ids[1],
                    "confidence": 0.5,
                    "source": {"source_plugin_id": self.plugin_id},
                }
            )
        return merged

