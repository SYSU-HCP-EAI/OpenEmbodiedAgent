"""Runtime compatibility preflight result schemas."""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, Field

from PhyAgentOS.runtime.schemas.adapter_plan import AdapterPlan


class TargetToolManifest(BaseModel):
    expose: list[str] = Field(default_factory=list)
    validation_mode: Literal["strict"] = "strict"
    agent_exposure: Literal["target_tools", "constrained_target_tools"] = "target_tools"


class MissingItem(BaseModel):
    code: str
    field: str
    expected: str
    found: str | None = None
    triggered_by: str
    fix: str


class RuntimeCompatibilityPreflightResult(BaseModel):
    verdict: Literal["accepted", "rejected"]
    session_id: str
    target_id: str
    skill_id: str
    runner_type: str = "SessionRunner"
    skill_runtime_kind: Literal["policy", "builtin"] | None = None
    execution_mode: str | None = None
    adapter_plan: AdapterPlan | None = None
    target_tool_manifest: TargetToolManifest | None = None
    missing_items: list[MissingItem] = Field(default_factory=list)
    warnings: list[str] = Field(default_factory=list)
