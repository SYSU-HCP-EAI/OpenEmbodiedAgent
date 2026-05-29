"""Minecraft target adapter: converts between protocol observations and runtime format."""

from __future__ import annotations

from typing import Any

import numpy as np

from PhyAgentOS.runtime.adapters.base import BaseTargetAdapter
from PhyAgentOS.runtime.watchdog.errors import AdapterError


class MinecraftTargetAdapter(BaseTargetAdapter):
    """Normalize mineflayer bridge observations into RuntimeObservation format."""

    def to_runtime_observation(self, raw_obs: dict[str, Any], target_info: dict[str, Any]) -> dict[str, Any]:
        state = raw_obs.get("state")
        if state is None:
            raise AdapterError("minecraft observation missing 'state' field")
        state_array = np.asarray(state, dtype=np.float32)
        image = raw_obs.get("image")
        if image is not None:
            image_array = np.asarray(image, dtype=np.uint8)
        else:
            image_array = np.zeros((224, 224, 3), dtype=np.uint8)

        sensors: dict[str, Any] = {
            "front_rgb": {
                "kind": "image",
                "observation_key": "image.front_rgb",
                "data": image_array,
                "dtype": "uint8",
                "layout": "HWC",
            },
            "proprio": {
                "kind": "vector",
                "observation_key": "state.proprio",
                "data": state_array,
                "dtype": "float32",
            },
        }

        runtime_obs: dict[str, Any] = {
            "observation_id": f"mc_obs_{target_info.get('step_index', 0)}",
            "sensors": sensors,
            "target_info": target_info,
        }
        for key in ("nearby_blocks", "nearby_entities", "inventory", "info"):
            value = raw_obs.get(key)
            if value is not None:
                runtime_obs[key] = value

        return runtime_obs

    def to_executable_action_chunk(
        self,
        action_chunk: dict[str, Any],
        target_info: dict[str, Any],
    ) -> dict[str, Any]:
        actions = action_chunk.get("actions")
        if actions is None:
            single_action = action_chunk.get("action")
            if single_action is not None:
                actions = [single_action]
            else:
                raise AdapterError("action chunk must contain 'actions' list or single 'action' dict")

        action_list = []
        for act in actions:
            if isinstance(act, dict):
                action_list.append(act)
            else:
                raise AdapterError(f"each action must be a dict, got {type(act).__name__}")

        return {
            "chunk_id": action_chunk.get("chunk_id", "mc_chunk"),
            "source_observation_id": action_chunk.get("source_observation_id"),
            "actions": action_list,
            "safety": {
                "require_target_side_validation": False,
                "stop_on_timeout": True,
            },
        }
