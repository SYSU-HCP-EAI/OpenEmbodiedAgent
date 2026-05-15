"""Sensor frame container passed to perception plugins."""

from __future__ import annotations

from typing import Any

from pydantic import BaseModel, Field


class SensorFrame(BaseModel):
    target_id: str
    session_id: str
    channels: dict[str, Any] = Field(default_factory=dict)
    sensor_channels: dict[str, str] = Field(default_factory=dict)
    metadata: dict[str, Any] = Field(default_factory=dict)

    class Config:
        arbitrary_types_allowed = True

