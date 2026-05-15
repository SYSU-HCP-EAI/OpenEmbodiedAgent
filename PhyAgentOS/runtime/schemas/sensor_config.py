"""External runtime sensor configuration schemas."""

from __future__ import annotations

from enum import StrEnum
from typing import Any, Literal

from pydantic import BaseModel, Field


class SensorModality(StrEnum):
    RGB = "rgb"
    DEPTH = "depth"
    POINTCLOUD = "pointcloud"
    LIDAR = "lidar"
    PROPRIO = "proprio"
    POSE = "pose"
    SIM_GT = "sim_gt"


class SensorCalibration(BaseModel):
    intrinsics: str | None = None
    extrinsics: str | None = None
    aligned_to: str | None = None
    depth_scale: float | None = None
    depth_unit: Literal["meter", "millimeter"] | None = None


class SensorSpec(BaseModel):
    id: str
    modality: SensorModality
    role: str
    source: str
    observation_key: str
    frame_id: str | None = None
    enabled: bool = True
    resolution: tuple[int, int] | None = None
    hz: float | None = None
    calibration: SensorCalibration = Field(default_factory=SensorCalibration)
    required_fields: list[str] = Field(default_factory=list)


class ObservationChannelSpec(BaseModel):
    dtype: str
    shape: list[int | None]


class SensorConfigDocument(BaseModel):
    version: Literal["runtime_sensor_config_v1"] = "runtime_sensor_config_v1"
    target_id: str
    updated_at: str | None = None
    sensors: list[SensorSpec]
    sync: dict[str, Any] = Field(default_factory=dict)
    observation_schema: dict[str, ObservationChannelSpec] = Field(default_factory=dict)

