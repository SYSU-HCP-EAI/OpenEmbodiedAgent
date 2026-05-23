"""Runtime target registry schemas."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field


class TargetPerceptionRefs(BaseModel):
    enabled: bool = False
    strict_preflight: Literal[True] = True
    sensor_config_ref: Path | None = None
    perception_config_ref: Path | None = None
    artifact_dir: Path | None = None
    config_version: str | None = None


class TargetRuntimeSpec(BaseModel):
    target_runtime: str
    target_endpoint: str
    target_adapter: str
    runtime_contract_ref: Path


class TargetSpec(BaseModel):
    model_config = ConfigDict(extra="forbid")

    id: str
    type: Literal["sim", "real_robot"]
    embodiment: str | None = None
    backend: str | None = None
    benchmark: str | None = None
    enabled: bool = True
    workspace: str
    supported_skills: list[str] = Field(default_factory=list)
    runtime: TargetRuntimeSpec
    perception: TargetPerceptionRefs = Field(default_factory=TargetPerceptionRefs)
    config: dict[str, Any] = Field(default_factory=dict)


class TargetsDocument(BaseModel):
    version: Literal["runtime_target_registry_v1"] = "runtime_target_registry_v1"
    targets: list[TargetSpec] = Field(default_factory=list)
