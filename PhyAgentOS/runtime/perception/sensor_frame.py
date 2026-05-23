"""Sensor frame container passed to perception plugins."""

from __future__ import annotations

from typing import Any

from pydantic import BaseModel, ConfigDict, Field


class SensorFrame(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)

    target_id: str
    session_id: str
    channels: dict[str, Any] = Field(default_factory=dict)
    sensor_channels: dict[str, str] = Field(default_factory=dict)
    metadata: dict[str, Any] = Field(default_factory=dict)

