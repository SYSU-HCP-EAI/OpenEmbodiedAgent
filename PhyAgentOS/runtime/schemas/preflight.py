"""Runtime compatibility preflight result schemas."""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, Field

from PhyAgentOS.runtime.schemas.adapter_plan import AdapterPlan


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
    adapter_plan: AdapterPlan | None = None
    missing_items: list[MissingItem] = Field(default_factory=list)
    warnings: list[str] = Field(default_factory=list)
