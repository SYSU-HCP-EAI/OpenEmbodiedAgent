"""Runtime target contract schemas."""

from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, Field


class ActionComponentSpec(BaseModel):
    name: str
    unit: str | None = None


class ActionChunkSpec(BaseModel):
    max_chunk_size: int | None = None
    preferred_chunk_size: int | None = None
    preferred_replan_after_steps: int | None = None
    switch_policy: Literal["soft_blend", "hard_switch"] = "hard_switch"


class TargetActionContract(BaseModel):
    id: str
    accepted_representations: list[str]
    shape: list[int | str]
    dtype: str
    normalized: bool
    frame: str
    control_mode: str
    control_hz: float
    components: list[ActionComponentSpec] = Field(default_factory=list)
    chunk: ActionChunkSpec = Field(default_factory=ActionChunkSpec)


class RuntimeSafetySpec(BaseModel):
    require_target_side_validation: Literal[True] = True
    workspace_bounds_ref: str | None = None
    max_translation_per_step_m: float | None = None
    max_rotation_per_step_rad: float | None = None
    max_linear_velocity_mps: float | None = None
    max_angular_velocity_radps: float | None = None
    gripper_range: tuple[float, float] | None = None
    stop_on_nan: Literal[True] = True
    stop_on_timeout: Literal[True] = True
    operator_override_required: bool = False


class TargetRuntimeContractDocument(BaseModel):
    version: Literal["runtime_target_contract_v1"] = "runtime_target_contract_v1"
    target_id: str
    target_adapter: str
    observation: dict[str, Any] = Field(default_factory=dict)
    action_contract: TargetActionContract
    safety: RuntimeSafetySpec
    capabilities: dict[str, Any] = Field(default_factory=dict)
