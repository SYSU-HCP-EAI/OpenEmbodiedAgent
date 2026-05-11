from typing import Optional

from internutopia.core.config.metric import MetricCfg


class RecordingMetricCfg(MetricCfg):
    name: Optional[str] = 'recording_metric'
    type: Optional[str] = 'RecordingMetric'
    robot_name: str
    fields: list = None  # fields that need to be recorded.
