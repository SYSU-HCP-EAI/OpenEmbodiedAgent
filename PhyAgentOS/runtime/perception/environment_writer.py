"""Atomic ENVIRONMENT.md v2 writer for perception deltas."""

from __future__ import annotations

import json
import math
import re
from pathlib import Path
from typing import Any

import yaml

from PhyAgentOS.runtime.schemas.common import utc_now
from PhyAgentOS.runtime.schemas.environment import EnvironmentDocument, PerceptionRunRecord
from PhyAgentOS.runtime.schemas.perception import EnvironmentDelta, EnvironmentObject, RefreshScope
from PhyAgentOS.runtime.state_io.atomic_file import atomic_write_text

_ENV_BLOCK_RE = re.compile(
    r"(?P<fence>`{3,}|~{3,})\s*(?P<lang>json|yaml|yml)\s*\n(?P<body>.*?)(?:\n(?P=fence)\s*)",
    re.DOTALL | re.IGNORECASE,
)


class EnvironmentWriter:
    def __init__(self, workspace: Path, *, position_threshold_m: float = 0.25):
        self.workspace = workspace
        self.position_threshold_m = position_threshold_m

    def write(
        self,
        *,
        target_id: str,
        session_id: str,
        run_id: str,
        sensor_config_ref: str,
        perception_config_ref: str | None,
        pipeline_id: str | None,
        pipeline: list[str],
        requested_outputs: list[str],
        artifact_dir: str,
        delta: EnvironmentDelta,
        latency_ms: float | None = None,
    ) -> EnvironmentDocument:
        document = self._load_v2_document()
        now = utc_now().isoformat()

        objects = dict(document.objects)
        scope = self._resolve_scope(target_id, delta)
        local_to_global: dict[str, str] = {}
        incoming: dict[str, EnvironmentObject] = {}

        for local_id, obj in delta.objects.items():
            obj.source.local_object_id = obj.source.local_object_id or local_id
            global_id = self._resolve_global_object_id(objects, incoming, local_id, obj, scope)
            obj.identity.update(
                {
                    "global_id": global_id,
                    "local_object_id": local_id,
                    "anchor": self._object_anchor(obj),
                    "position_threshold_m": scope.position_threshold_m,
                }
            )
            incoming[global_id] = obj
            local_to_global[local_id] = global_id

        scoped_global_ids = {
            local_to_global.get(object_id, object_id) for object_id in (scope.object_ids or delta.refresh_scope)
        }
        if not scoped_global_ids:
            scoped_global_ids = set(incoming)
        for object_id, existing in list(objects.items()):
            source = existing.source
            if source.target_id == target_id and object_id in scoped_global_ids and object_id not in incoming:
                del objects[object_id]
        objects.update(incoming)

        relations = [
            self._rewrite_relation_ids(relation, local_to_global)
            for relation in document.scene_graph.get("relations", [])
            if relation.get("subject") in objects and relation.get("object") in objects
        ]
        if delta.relations:
            relations.extend(self._rewrite_relation_ids(relation, local_to_global) for relation in delta.relations)

        runs = dict(document.perception.get("runs", {}))
        runs[run_id] = PerceptionRunRecord(
            target_id=target_id,
            session_id=session_id,
            sensor_config_ref=sensor_config_ref,
            perception_config_ref=perception_config_ref,
            pipeline_id=pipeline_id,
            pipeline=pipeline,
            status="ok",
            requested_outputs=requested_outputs,
            generated_outputs=delta.generated_outputs,
            refresh_scope=scope.model_dump(mode="json", exclude_none=True),
            latency_ms=latency_ms,
            num_objects=len(incoming),
            artifact_dir=artifact_dir,
        )

        targets = dict(document.targets)
        targets[target_id] = {
            "latest_observation_at": now,
            "available_sensors": [],
            "sensor_config_ref": sensor_config_ref,
            "active_perception_pipeline": pipeline,
            "last_perception_run_id": run_id,
        }

        updated = EnvironmentDocument(
            updated_at=now,
            targets=targets,
            objects=objects,
            scene_graph={"relations": relations},
            perception={"runs": runs},
            map=document.map,
            tf=document.tf,
        )
        self._save(updated)
        return updated

    def _resolve_scope(self, target_id: str, delta: EnvironmentDelta) -> RefreshScope:
        if delta.scope is not None:
            return delta.scope
        return RefreshScope(
            target_id=target_id,
            object_ids=list(delta.refresh_scope or delta.objects.keys()),
            position_threshold_m=self.position_threshold_m,
        )

    def _resolve_global_object_id(
        self,
        existing: dict[str, EnvironmentObject],
        incoming: dict[str, EnvironmentObject],
        local_id: str,
        obj: EnvironmentObject,
        scope: RefreshScope,
    ) -> str:
        anchor = self._object_anchor(obj)
        if anchor is not None:
            for candidate_id, candidate in {**existing, **incoming}.items():
                candidate_anchor = self._object_anchor(candidate)
                if candidate_anchor is None:
                    continue
                if anchor["frame_id"] != candidate_anchor["frame_id"]:
                    continue
                if obj.label != candidate.label:
                    continue
                if self._distance(anchor["position_m"], candidate_anchor["position_m"]) <= scope.position_threshold_m:
                    return candidate_id

        namespaced = f"{obj.source.target_id}::{local_id}"
        if namespaced not in existing and namespaced not in incoming:
            return namespaced
        suffix = 2
        while f"{namespaced}::{suffix}" in existing or f"{namespaced}::{suffix}" in incoming:
            suffix += 1
        return f"{namespaced}::{suffix}"

    def _object_anchor(self, obj: EnvironmentObject) -> dict[str, Any] | None:
        pose = obj.pose if isinstance(obj.pose, dict) else None
        if not pose:
            return None
        frame_id = pose.get("frame_id")
        position = pose.get("position_m")
        if not frame_id or not isinstance(position, list) or len(position) < 3:
            return None
        try:
            return {"frame_id": str(frame_id), "position_m": [float(position[0]), float(position[1]), float(position[2])]}
        except (TypeError, ValueError):
            return None

    def _distance(self, left: list[float], right: list[float]) -> float:
        return math.sqrt(sum((left[idx] - right[idx]) ** 2 for idx in range(3)))

    def _rewrite_relation_ids(self, relation: dict[str, Any], local_to_global: dict[str, str]) -> dict[str, Any]:
        rewritten = dict(relation)
        if rewritten.get("subject") in local_to_global:
            rewritten["subject"] = local_to_global[rewritten["subject"]]
        if rewritten.get("object") in local_to_global:
            rewritten["object"] = local_to_global[rewritten["object"]]
        return rewritten

    def _load_v2_document(self) -> EnvironmentDocument:
        path = self.workspace / "ENVIRONMENT.md"
        if not path.exists():
            return EnvironmentDocument(updated_at=utc_now().isoformat())
        payload = self._load_block(path)
        if payload.get("schema_version") != "PhyAgentOS.environment.v2":
            return EnvironmentDocument(updated_at=utc_now().isoformat())
        try:
            return EnvironmentDocument.model_validate(payload)
        except Exception:
            return EnvironmentDocument(updated_at=utc_now().isoformat())

    def _load_block(self, path: Path) -> dict[str, Any]:
        match = _ENV_BLOCK_RE.search(path.read_text(encoding="utf-8"))
        if match is None:
            return {}
        body = match.group("body")
        try:
            if match.group("lang").lower() == "json":
                data = json.loads(body)
            else:
                data = yaml.safe_load(body) or {}
        except Exception:
            return {}
        return data if isinstance(data, dict) else {}

    def _save(self, document: EnvironmentDocument) -> None:
        data = document.model_dump(mode="json", exclude_none=True)
        text = (
            "# Environment State\n\n"
            "Auto-updated by PhyAgentOS perception runtime.\n\n"
            f"```json\n{json.dumps(data, indent=2, ensure_ascii=False)}\n```\n"
        )
        atomic_write_text(self.workspace / "ENVIRONMENT.md", text)
