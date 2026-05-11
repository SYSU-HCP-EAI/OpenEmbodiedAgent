from typing import Optional, Tuple

from pydantic import BaseModel


class BaseCfg(BaseModel):
    def update(self, **kwargs):
        return self.model_copy(update=kwargs, deep=True)


class SensorCfg(BaseCfg):
    """
    Represents a model for sensors, encapsulating their attributes and providing a structured approach to handling sensor data within a base model framework.

    This SensorModel class extends the BaseModel, inheriting its functionality while adding specific fields tailored for describing sensors. It includes attributes for the sensor's name, primary path (optional), and type, offering a standardized way to organize and access sensor information across various parts of an application.

    Attributes:
        name (str): The unique identifier for the sensor.
        prim_path (Optional[str], optional): The primary path associated with the sensor, if any. Defaults to None.
        type (str): The type of the sensor, specifying its functionality or category.
    """

    name: str
    prim_path: Optional[str] = None
    type: str
