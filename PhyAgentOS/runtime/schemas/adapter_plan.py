"""Adapter plan schema produced by runtime compatibility preflight."""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, Field


class AdapterPlan(BaseModel):
    target_adapter: str
    policy_adapter: str | None = None
    observation_path: list[str] = Field(default_factory=list)
    action_path: list[str] = Field(default_factory=list)
    action_bridges: list[str] = Field(default_factory=list)
    validation_mode: Literal["strict"] = "strict"

    @property
    def adapter_plan_id(self) -> str:
        return f"adapter_plan:{self.target_adapter}:{self.policy_adapter or 'no_policy'}"
