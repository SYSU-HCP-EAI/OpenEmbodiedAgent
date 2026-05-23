"""Runtime skill registry schemas."""

from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, Field


class SkillRequirements(BaseModel):
    sensors: list[str] = Field(default_factory=list)
    environment_outputs: list[str] = Field(default_factory=list)
    object_queries_from_task: bool = False
    min_confidence: float | None = None
    require_metric_geometry: bool = False
    strict_environment_contract: bool = True


class SkillSpec(BaseModel):
    id: str
    category: Literal["vla", "builtin", "hybrid", "debug"]
    runtime: str
    supported_target_types: list[Literal["sim", "real_robot"]]
    policy_client: str | None = None
    policy_adapter: str | None = None
    supports_chunk: bool = False
    default_replan_every: int = 1
    input_contract: dict[str, Any] = Field(default_factory=dict)
    output_contract: dict[str, Any] = Field(default_factory=dict)
    adapter_requirements: dict[str, Any] = Field(default_factory=dict)
    requires: SkillRequirements = Field(default_factory=SkillRequirements)


class SkillsDocument(BaseModel):
    version: Literal["runtime_skill_registry_v1"] = "runtime_skill_registry_v1"
    skills: list[SkillSpec] = Field(default_factory=list)
