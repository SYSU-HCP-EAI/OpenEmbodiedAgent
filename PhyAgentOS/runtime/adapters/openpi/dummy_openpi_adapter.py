"""Dummy simulation adapter for OpenPI-style policy observations."""

from __future__ import annotations

from typing import Any

import numpy as np

from PhyAgentOS.runtime.adapters.openpi.base_openpi_adapter import BaseOpenPIAdapter
from PhyAgentOS.runtime.watchdog.errors import AdapterError


class DummyOpenPIAdapter(BaseOpenPIAdapter):
    def make_observation(
        self,
        raw_obs: dict[str, Any],
        step_idx: int,
        target_info: dict[str, Any],
    ) -> dict[str, Any]:
        try:
            image = raw_obs["image"]
            wrist_image = raw_obs["wrist_image"]
            state = raw_obs["state"]
        except KeyError as exc:
            raise AdapterError(f"dummy observation missing key: {exc.args[0]}") from exc
        return {
            "observation/image": np.asarray(image, dtype=np.uint8),
            "observation/wrist_image": np.asarray(wrist_image, dtype=np.uint8),
            "observation/state": np.asarray(state, dtype=np.float32),
            "prompt": str(target_info["task_description"]),
        }
