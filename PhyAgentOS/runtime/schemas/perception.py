"""Runtime perception configuration and delta schemas."""

from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, Field


class PerceptionModelSpec(BaseModel):
    id: str
    type: str
    provider: Literal[
        "python_module",
        "local_checkpoint",
        "local_worker",
        "remote_endpoint",
        "builtin_dummy",
    ]
    module: str | None = None
    checkpoint_path: str | None = None
    endpoint: str | None = None
    healthcheck: str | None = None
    required_packages: list[str] = Field(default_factory=list)
    install_hint: str = ""


class PerceptionPluginCandidate(BaseModel):
    id: str
    type: str
    model_ref: str | None = None
    priority: int = 0
    requires_plugins: list[str] = Field(default_factory=list)
    requires_outputs: list[str] = Field(default_factory=list)
    requires_modalities: list[str] = Field(default_factory=list)
    requires_sensors: list[str] = Field(default_factory=list)
    produces: list[str] = Field(default_factory=list)
    module: str | None = None
    class_name: str | None = None
    config: dict[str, Any] = Field(default_factory=dict)


class PerceptionPipelineSpec(BaseModel):
    id: str
    required_outputs: list[str]
    use_plugins: list[str]


class PerceptionConfigDocument(BaseModel):
    version: Literal["runtime_perception_config_v1"] = "runtime_perception_config_v1"
    target_id: str
    strict_preflight: Literal[True] = True
    default_outputs: list[str] = Field(default_factory=list)
    models: list[PerceptionModelSpec] = Field(default_factory=list)
    plugin_candidates: list[PerceptionPluginCandidate] = Field(default_factory=list)
    pipelines: list[PerceptionPipelineSpec] = Field(default_factory=list)


class EnvironmentObjectSource(BaseModel):
    target_id: str
    session_id: str | None = None
    perception_run_id: str | None = None
    pipeline: list[str] = Field(default_factory=list)
    camera_id: str | None = None
    sensor_id: str | None = None
    source_plugin_id: str | None = None
    local_object_id: str | None = None


class EnvironmentObject(BaseModel):
    label: str
    confidence: float | None = None
    state: str = "visible"
    source: EnvironmentObjectSource
    bbox_2d: list[float] | None = None
    mask_artifact_uri: str | None = None
    pose: dict[str, Any] | None = None
    geometry: dict[str, Any] | None = None
    identity: dict[str, Any] = Field(default_factory=dict)
    valid_until: str | None = None


class RefreshScope(BaseModel):
    target_id: str
    sensor_ids: list[str] = Field(default_factory=list)
    frame_id: str | None = None
    mode: str = "authoritative_object_ids"
    object_ids: list[str] = Field(default_factory=list)
    position_threshold_m: float = 0.25


class EnvironmentDelta(BaseModel):
    objects: dict[str, EnvironmentObject] = Field(default_factory=dict)
    relations: list[dict[str, Any]] = Field(default_factory=list)
    generated_outputs: list[str] = Field(default_factory=list)
    refresh_scope: list[str] = Field(default_factory=list)
    scope: RefreshScope | None = None
