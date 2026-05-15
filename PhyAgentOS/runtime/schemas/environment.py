"""PhyAgentOS.environment.v2 schema."""

from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, Field

from PhyAgentOS.runtime.schemas.perception import EnvironmentObject


class PerceptionRunRecord(BaseModel):
    target_id: str
    session_id: str | None = None
    sensor_config_ref: str
    perception_config_ref: str | None = None
    pipeline_id: str | None = None
    pipeline: list[str] = Field(default_factory=list)
    status: Literal["ok", "failed"] = "ok"
    requested_outputs: list[str] = Field(default_factory=list)
    generated_outputs: list[str] = Field(default_factory=list)
    refresh_scope: dict[str, Any] = Field(default_factory=dict)
    latency_ms: float | None = None
    num_objects: int | None = None
    artifact_dir: str | None = None


class EnvironmentDocument(BaseModel):
    schema_version: Literal["PhyAgentOS.environment.v2"] = "PhyAgentOS.environment.v2"
    updated_at: str
    targets: dict[str, dict[str, Any]] = Field(default_factory=dict)
    objects: dict[str, EnvironmentObject] = Field(default_factory=dict)
    scene_graph: dict[str, Any] = Field(default_factory=lambda: {"relations": []})
    perception: dict[str, dict[str, PerceptionRunRecord]] = Field(default_factory=lambda: {"runs": {}})
    map: dict[str, Any] = Field(default_factory=dict)
    tf: dict[str, Any] = Field(default_factory=dict)
