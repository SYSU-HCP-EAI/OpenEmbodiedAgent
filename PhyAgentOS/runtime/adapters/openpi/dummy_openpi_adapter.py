"""Dummy simulation adapter for OpenPI-style policy observations."""

from __future__ import annotations

from typing import Any

import numpy as np

from PhyAgentOS.runtime.adapters.openpi.base_openpi_adapter import BaseOpenPIAdapter
from PhyAgentOS.runtime.watchdog.errors import AdapterError


class DummyOpenPIAdapter(BaseOpenPIAdapter):
    def to_policy_input(
        self,
        runtime_observation: dict[str, Any],
        session_ctx: dict[str, Any],
    ) -> dict[str, Any]:
        try:
            sensors = runtime_observation["sensors"]
            image = sensors["front_rgb"]["data"]
            wrist_image = sensors["wrist_rgb"]["data"]
            state = sensors["proprio"]["data"]
        except KeyError as exc:
            raise AdapterError(f"dummy observation missing key: {exc.args[0]}") from exc
        if image is None or wrist_image is None or state is None:
            raise AdapterError("dummy observation missing image, wrist_image, or state")
        return {
            "observation/image": np.asarray(image, dtype=np.uint8),
            "observation/wrist_image": np.asarray(wrist_image, dtype=np.uint8),
            "observation/state": np.asarray(state, dtype=np.float32),
            "prompt": str(session_ctx["task_description"]),
        }
