"""Semantic navigation tool that resolves scene targets and dispatches actions."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import TYPE_CHECKING, Any

from PhyAgentOS.agent.tools.base import Tool
from PhyAgentOS.agent.tools.embodied import EmbodiedActionTool
from PhyAgentOS.embodiment_registry import EmbodimentRegistry
from PhyAgentOS.utils.helpers import load_environment_doc

if TYPE_CHECKING:
    from PhyAgentOS.embodiment_registry import EmbodimentRegistry


class SemanticNavigationTool(Tool):
    """Resolve semantic targets into concrete navigation actions."""

    def __init__(self, workspace: Path, action_tool: EmbodiedActionTool, registry: EmbodimentRegistry | None = None):
        self.workspace = workspace
        self.action_tool = action_tool
        self.registry = registry

    @property
    def name(self) -> str:
        return "semantic_navigate"

    @property
    def description(self) -> str:
        return "Navigate to a semantic target such as an object class, node id, or zone."

    @property
    def parameters(self) -> dict[str, Any]:
        return {
            "type": "object",
            "properties": {
                "target_class": {"type": "string"},
                "target_id": {"type": "string"},
                "zone_name": {"type": "string"},
                "robot_id": {"type": "string"},
                "approach_distance": {"type": "number", "minimum": 0.1},
                "timeout_s": {"type": "integer", "minimum": 1},
                "reasoning": {"type": "string"},
            },
            "required": ["robot_id", "reasoning"],
        }

    async def execute(
        self,
        robot_id: str,
        reasoning: str,
        target_class: str | None = None,
        target_id: str | None = None,
        zone_name: str | None = None,
        approach_distance: float = 0.5,
        timeout_s: int = 120,
    ) -> str:
        env = load_environment_doc(self.workspace / "ENVIRONMENT.md")
        target = self._resolve_target(
            env,
            robot_id=robot_id,
            target_class=target_class,
            target_id=target_id,
            zone_name=zone_name,
        )
        if "error" in target:
            return f"Error: {target['error']}"

        action_parameters = {
            "robot_id": robot_id,
            "approach_distance": approach_distance,
            "timeout_s": timeout_s,
            "target_ref": {
                "kind": target["kind"],
                "id": target["id"],
                "label": target["label"],
            },
            "goal_pose": self._calculate_approach_pose(env, robot_id, target, approach_distance),
        }
        if target_class:
            action_parameters["target_class"] = target_class
        if target_id:
            action_parameters["target_id"] = target_id
        if zone_name:
            action_parameters["zone_name"] = zone_name

        return await self.action_tool.execute(
            action_type="semantic_navigate",
            parameters=action_parameters,
            reasoning=reasoning,
        )

    def _resolve_target(
        self,
        env: dict[str, Any],
        *,
        robot_id: str,
        target_class: str | None,
        target_id: str | None,
        zone_name: str | None,
    ) -> dict[str, Any]:
        scene_graph = env.get("scene_graph", {})
        nodes = scene_graph.get("nodes", [])

        if target_id:
            for node in nodes:
                if node.get("id") == target_id:
                    return {
                        "kind": "node",
                        "id": node["id"],
                        "label": node.get("class", node["id"]),
                        "center": node.get("center", {}),
                        "size": node.get("size", {}),
                    }
            return {"error": f"target id '{target_id}' not found"}

        if target_class:
            robot_pose = (((env.get("robots") or {}).get(robot_id) or {}).get("robot_pose") or {})
            candidates = [node for node in nodes if node.get("class") == target_class]
            if not candidates:
                return {"error": f"target class '{target_class}' not found"}
            if "x" in robot_pose and "y" in robot_pose:
                candidates.sort(
                    key=lambda node: math.hypot(
                        (node.get("center") or {}).get("x", 0.0) - robot_pose["x"],
                        (node.get("center") or {}).get("y", 0.0) - robot_pose["y"],
                    )
                )
            node = candidates[0]
            return {
                "kind": "node",
                "id": node.get("id", target_class),
                "label": node.get("class", target_class),
                "center": node.get("center", {}),
                "size": node.get("size", {}),
            }

        if zone_name:
            for zone in (env.get("map", {}) or {}).get("zones", []):
                if zone.get("name") == zone_name:
                    center = zone.get("center") or {}
                    return {
                        "kind": "zone",
                        "id": zone_name,
                        "label": zone_name,
                        "center": center,
                        "size": zone.get("size", {}),
                    }
            return {"error": f"zone '{zone_name}' not found"}

        return {"error": "one of target_class, target_id, or zone_name is required"}

    @staticmethod
    def _calculate_approach_pose(
        env: dict[str, Any],
        robot_id: str,
        target: dict[str, Any],
        approach_distance: float,
    ) -> dict[str, Any]:
        center = target.get("center") or {}
        robot_pose = (((env.get("robots") or {}).get(robot_id) or {}).get("robot_pose") or {})
        rx = float(robot_pose.get("x", center.get("x", 0.0) - approach_distance))
        ry = float(robot_pose.get("y", center.get("y", 0.0)))
        tx = float(center.get("x", 0.0))
        ty = float(center.get("y", 0.0))
        heading = math.atan2(ty - ry, tx - rx) if (tx, ty) != (rx, ry) else 0.0

        return {
            "frame": center.get("frame", "map"),
            "x": tx - approach_distance * math.cos(heading),
            "y": ty - approach_distance * math.sin(heading),
            "z": float(center.get("z", 0.0)),
            "yaw": heading,
        }
