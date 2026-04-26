"""Read-only embodied question answering over ENVIRONMENT.md."""

from __future__ import annotations

import re
from pathlib import Path
from typing import Any

from PhyAgentOS.agent.tools.base import Tool
from PhyAgentOS.embodiment_registry import EmbodimentRegistry
from hal.simulation.scene_io import load_environment_doc


class EmbodiedQuestionTool(Tool):
    """Answer grounded scene questions from the current environment snapshot."""

    _VISIBLE_PATTERN = re.compile(
        r"(?:what(?:'s| is| do)\s+(?:in\s+)?(?:your\s+view|visible|there)|what\s+can\s+you\s+see|"
        r"你(?:的)?视角里有什么|你看到了什么|你能看到什么)",
        re.IGNORECASE,
    )
    _SURFACE_PATTERN = re.compile(
        r"\b(?:what(?:'s| is)|which(?: objects| items)?(?: are)?|is\s+there\s+(?:anything|any(?: objects| items)?))\s+"
        r"(?P<relation>on|in|inside|near|around)\s+(?:the\s+)?"
        r"(?P<surface>[a-z0-9_\- ]+?)"
        r"(?:\s+in\s+front\s+of\s+you)?(?:\?|$)",
        re.IGNORECASE,
    )
    _RELATION_MAP = {
        "on": {"ON"},
        "in": {"IN", "INSIDE"},
        "inside": {"IN", "INSIDE"},
        "near": {"NEAR", "CLOSE_TO"},
        "around": {"NEAR", "CLOSE_TO"},
    }
    _RELATION_PREFIX = {
        "on": "On",
        "in": "In",
        "inside": "Inside",
        "near": "Near",
        "around": "Near",
    }

    def __init__(self, workspace: Path, registry: EmbodimentRegistry | None = None):
        self.workspace = workspace
        self.registry = registry

    @property
    def name(self) -> str:
        return "answer_embodied_question"

    @property
    def description(self) -> str:
        return (
            "Answer a grounded embodied question from the current ENVIRONMENT.md scene graph. "
            "Use this for questions like 'what is on the table'. In fleet mode include robot_id."
        )

    @property
    def parameters(self) -> dict[str, Any]:
        return {
            "type": "object",
            "properties": {
                "question": {"type": "string"},
                "robot_id": {"type": "string"},
            },
            "required": ["question"],
        }

    async def execute(self, question: str, robot_id: str | None = None) -> str:
        if self.registry and self.registry.is_fleet and not robot_id:
            return "Error: robot_id is required for embodied questions in fleet mode."

        try:
            env_path = self._resolve_environment_path(robot_id)
        except KeyError as exc:
            return f"Error: {exc}"

        env = load_environment_doc(env_path)
        if not isinstance(env, dict) or not env:
            return "I don't have any current embodied scene data yet."

        scene_graph = env.get("scene_graph") or {}
        nodes = scene_graph.get("nodes") or []
        edges = scene_graph.get("edges") or []
        parsed = self._parse_surface_question(question)
        fallback_summary = self._fallback_summary(env)
        if parsed is None and self._is_visible_scene_question(question):
            if not isinstance(nodes, list):
                return fallback_summary or "I don't have a usable scene graph right now."
            visible_nodes = [
                node
                for node in nodes
                if isinstance(node, dict) and self._normalize(str(node.get("role", ""))) != "surface"
            ]
            if not visible_nodes:
                return fallback_summary or "I don't currently see any grounded scene objects."
            labels = [self._node_phrase(node) for node in visible_nodes]
            return f"I can currently see: {', '.join(labels)}."

        if parsed is None:
            return fallback_summary or "I can answer grounded scene questions like 'what is on the table'."

        relation, surface_name = parsed
        if not isinstance(nodes, list) or not isinstance(edges, list):
            return fallback_summary or "I don't have a usable scene graph right now."

        target_nodes = [node for node in nodes if self._node_matches_surface(node, surface_name)]
        if not target_nodes:
            return fallback_summary or f"I couldn't find '{surface_name}' in the current scene graph."

        node_by_id = {
            str(node.get("id")): node
            for node in nodes
            if isinstance(node, dict) and str(node.get("id", "")).strip()
        }
        target_ids = {
            str(node.get("id"))
            for node in target_nodes
            if isinstance(node, dict) and str(node.get("id", "")).strip()
        }
        wanted_relations = self._RELATION_MAP.get(relation, {relation.upper()})

        matched_nodes: list[dict[str, Any]] = []
        seen_sources: set[str] = set()
        for edge in edges:
            if not isinstance(edge, dict):
                continue
            target_id = str(edge.get("target", "")).strip()
            source_id = str(edge.get("source", "")).strip()
            if target_id not in target_ids or source_id in seen_sources:
                continue
            if str(edge.get("relation", "")).upper() not in wanted_relations:
                continue
            node = node_by_id.get(source_id)
            if node is None:
                continue
            seen_sources.add(source_id)
            matched_nodes.append(node)

        if not matched_nodes:
            return fallback_summary or f"{self._RELATION_PREFIX[relation]} the {surface_name}, I don't see anything right now."

        labels = [self._node_phrase(node) for node in matched_nodes]
        return f"{self._RELATION_PREFIX[relation]} the {surface_name}, I can see: {', '.join(labels)}."

    def _resolve_environment_path(self, robot_id: str | None) -> Path:
        if self.registry:
            return self.registry.resolve_environment_path(robot_id=robot_id, default_workspace=self.workspace)
        return self.workspace / "ENVIRONMENT.md"

    @classmethod
    def _parse_surface_question(cls, question: str) -> tuple[str, str] | None:
        match = cls._SURFACE_PATTERN.search(str(question or "").strip())
        if not match:
            return None
        relation = match.group("relation").strip().lower()
        surface = cls._normalize(match.group("surface"))
        if not surface:
            return None
        return relation, surface

    @staticmethod
    def _normalize(value: str) -> str:
        return " ".join(str(value or "").strip().lower().split())

    @classmethod
    def _is_visible_scene_question(cls, question: str) -> bool:
        return bool(cls._VISIBLE_PATTERN.search(str(question or "").strip()))

    @classmethod
    def _node_matches_surface(cls, node: Any, surface_name: str) -> bool:
        if not isinstance(node, dict):
            return False
        candidates = [
            node.get("id", ""),
            node.get("class", ""),
            node.get("object_key", ""),
            node.get("role", ""),
            node.get("label", ""),
        ]
        normalized = [cls._normalize(candidate) for candidate in candidates if str(candidate or "").strip()]
        if surface_name in normalized:
            return True
        return any(surface_name in candidate or candidate in surface_name for candidate in normalized)

    @staticmethod
    def _node_phrase(node: dict[str, Any]) -> str:
        color = str(node.get("color_label_cn", "")).strip()
        node_class = str(node.get("class", "")).strip()
        object_key = str(node.get("object_key", "") or node.get("id", "")).strip()
        if color and node_class and node_class.lower() != "object":
            return f"{color} {node_class}"
        if node_class and node_class.lower() != "object":
            return node_class
        if object_key:
            return object_key.replace("_", " ")
        return "unknown object"

    @staticmethod
    def _fallback_summary(env: dict[str, Any]) -> str:
        objects = env.get("objects") or {}
        if not isinstance(objects, dict):
            return ""
        runtime = objects.get("manipulation_runtime") or {}
        if not isinstance(runtime, dict):
            return ""
        summary = str(runtime.get("table_summary_cn", "")).strip()
        return summary

    @staticmethod
    def _manipulation_table_summary(env: dict[str, Any]) -> str:
        objects = env.get("objects") or {}
        runtime = objects.get("manipulation_runtime") if isinstance(objects, dict) else None
        if isinstance(runtime, dict):
            summary = str(runtime.get("table_summary_cn", "")).strip()
            if summary:
                return summary
        return ""

    def _target_matches(
        self,
        target: str,
        surface_aliases: set[str],
        node_by_id: dict[str, dict[str, Any]],
    ) -> bool:
        norm_target = self._normalize(target)
        if norm_target in surface_aliases:
            return True
        target_node = node_by_id.get(norm_target)
        if target_node is None:
            return False
        candidates = self._node_terms(target_node)
        return bool(candidates & surface_aliases)

    @staticmethod
    def _dedupe_nodes(nodes: list[dict[str, Any]]) -> list[dict[str, Any]]:
        seen: set[str] = set()
        out: list[dict[str, Any]] = []
        for node in nodes:
            key = str(node.get("id") or node.get("object_key") or node.get("class") or id(node))
            if key in seen:
                continue
            seen.add(key)
            out.append(node)
        return out

    def _objects_by_location(self, objects: dict[str, Any], surface_aliases: set[str] | None) -> list[str]:
        labels: list[str] = []
        for key, value in objects.items():
            if key == "manipulation_runtime" or not isinstance(value, dict):
                continue
            location = self._normalize(str(value.get("location", "")))
            if surface_aliases is not None and location not in surface_aliases:
                continue
            labels.append(self._describe_object(key, value))
        return labels

    def _describe_node(self, node: dict[str, Any]) -> str:
        color = str(node.get("color_label_cn") or node.get("color") or "").strip()
        label = str(node.get("class") or node.get("object_key") or node.get("id") or "object").strip()
        if color and color.lower() not in label.lower():
            return f"{color} {label}".strip()
        return label.replace("_", " ")

    def _describe_object(self, key: str, value: dict[str, Any]) -> str:
        color = str(value.get("color", "")).strip()
        label = str(value.get("type") or value.get("class") or key).strip().replace("_", " ")
        if color and color.lower() not in label.lower():
            return f"{color} {label}".strip()
        return label

    def _node_terms(self, node: dict[str, Any]) -> set[str]:
        fields = [
            node.get("id"),
            node.get("class"),
            node.get("object_key"),
            node.get("role"),
        ]
        terms = {self._normalize(str(field)) for field in fields if field}
        return {term for term in terms if term}

    def _surface_aliases(self, surface: str) -> set[str]:
        base = self._normalize(surface)
        aliases = {base}
        if base in {"table", "desk"}:
            aliases.update({"table", "desk", "surface_table", "furniture_table", "staging_table"})
        if base in {"pedestal", "stand"}:
            aliases.update({"pedestal", "place_pedestal", "surface_pedestal"})
        return aliases

    @staticmethod
    def _relation_alias(raw: str) -> str:
        normalized = EmbodiedQuestionTool._normalize(raw)
        if normalized in {"in", "inside"}:
            return "in"
        if normalized in {"near", "next to"}:
            return "close_to"
        return "on"

    @staticmethod
    def _relation_to_phrase(relation: str) -> str:
        return {
            "in": "in",
            "close_to": "near",
            "on": "on",
        }.get(relation, "on")

    @staticmethod
    def _strip_articles(text: str) -> str:
        return re.sub(r"^(?:a|an|the)\s+", "", text.strip())

    @staticmethod
    def _normalize(text: str) -> str:
        return re.sub(r"\s+", " ", re.sub(r"[^a-z0-9_\- ]+", " ", text.lower())).strip()