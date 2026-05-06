"""Runtime target registry schemas."""

from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, Field


class TargetSpec(BaseModel):
    id: str
    type: Literal["sim", "real_robot"]
    backend: str | None = None
    benchmark: str | None = None
    enabled: bool = True
    workspace: str
    supported_skills: list[str] = Field(default_factory=list)
    adapter: str
    config: dict[str, Any] = Field(default_factory=dict)


class TargetsDocument(BaseModel):
    version: Literal["runtime_target_registry_v1"] = "runtime_target_registry_v1"
    targets: list[TargetSpec] = Field(default_factory=list)
