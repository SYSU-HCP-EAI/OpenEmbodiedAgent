"""Minecraft Java Edition rollout target.  Connects to a mineflayer bridge
running on the same machine as Minecraft (typically Windows 11).

Architecture:
  [Linux cloud] MinecraftTarget --HTTP→ ngrok → [Windows 11] mineflayer bridge
                                                              └→ Minecraft

The bridge runs on the Windows machine alongside Minecraft, connecting
to the local game instance.  This target is a remote HTTP client — it
does NOT require Minecraft or pyCraft on the Linux side.
"""

from __future__ import annotations

import logging
import time
from typing import Any

import numpy as np

from PhyAgentOS.runtime.targets.local.base import BaseLocalTarget
from PhyAgentOS.runtime.watchdog.errors import (
    TargetConnectionError,
    TargetResetError,
    TargetStepError,
)

logger = logging.getLogger(__name__)

VALID_ACTION_TYPES = frozenset({
    "move", "look", "jump", "sneak", "sprint", "attack",
    "interact", "place", "dig", "use", "select_slot", "drop",
    "chat", "collect", "equip", "craft",
})


class MinecraftTarget(BaseLocalTarget):
    """Remote target connected to a mineflayer bridge running on the game machine.

    The bridge (``game/mc-bridge/bridge_server.js``) runs on the Windows
    machine alongside Minecraft.  It spawns a mineflayer bot that joins
    the local Minecraft world and exposes an HTTP API.  This target talks
    to the bridge over HTTP (typically via an ngrok tunnel).
    """

    def __init__(self, config: dict[str, Any] | None = None):
        self.config = dict(config or {})
        self._built = False
        self._http = None
        self._bridge_url: str = str(self.config.get("bridge_url", "")).strip()
        self._position = np.zeros(3, dtype=np.float64)
        self._yaw: float = 0.0
        self._pitch: float = 0.0
        self._health: float = 20.0
        self._hunger: int = 20
        self._held_slot: int = 0
        self._on_ground: bool = True
        self._dimension: str = "overworld"
        self._step_idx: int = 0
        self._step_delay: float = float(self.config.get("step_delay", 0.1))
        self._last_status: dict[str, Any] = {"status": "idle"}

    def _get_http(self):
        if self._http is None:
            import httpx
            verify = self.config.get("verify_ssl", True)
            headers = {"ngrok-skip-browser-warning": "true"}
            self._http = httpx.Client(timeout=10.0, verify=verify, headers=headers)
        return self._http

    def build(self) -> None:
        bridge_url = self._bridge_url
        if not bridge_url:
            raise TargetConnectionError(
                "bridge_url is not configured. Set it in TARGETS.md config, "
                "e.g. https://xxxx.ngrok-free.app (the ngrok URL for the Windows bridge)."
            )
        client = self._get_http()
        try:
            resp = client.get(f"{bridge_url}/health")
            data = resp.json()
        except Exception as exc:
            raise TargetConnectionError(
                f"Cannot reach mineflayer bridge at {bridge_url}: {exc}. "
                "Ensure the bridge is running on Windows and ngrok is active."
            ) from exc

        if not data.get("bot_spawned"):
            raise TargetConnectionError(
                f"mineflayer bot not spawned at {bridge_url}. "
                "Wait for bridge to finish connecting."
            )

        self._built = True
        logger.info("MinecraftTarget connected to bridge at %s", bridge_url)

    def reset(self, session_ctx: dict[str, Any]) -> dict[str, Any]:
        if not self._built:
            raise TargetResetError("target not built")
        self._step_idx = 0
        time.sleep(self._step_delay)
        return self.observe()

    def observe(self) -> dict[str, Any]:
        client = self._get_http()
        try:
            resp = client.get(f"{self._bridge_url}/state")
            data = resp.json()
        except Exception as exc:
            logger.warning("state fetch failed: %s", exc)
            return self._cached_obs()

        bot_data = data.get("bot") or {}
        pos = bot_data.get("position", {})
        rot = bot_data.get("rotation", {})
        self._position = np.array([
            float(pos.get("x", 0)),
            float(pos.get("y", 0)),
            float(pos.get("z", 0)),
        ], dtype=np.float64)
        self._yaw = float(rot.get("yaw", self._yaw))
        self._pitch = float(rot.get("pitch", self._pitch))
        self._health = float(data.get("health", self._health))
        self._hunger = int(data.get("hunger", self._hunger))
        self._on_ground = bool(bot_data.get("on_ground", self._on_ground))
        self._dimension = str(data.get("dimension", self._dimension))

        state = np.array([
            self._position[0], self._position[1], self._position[2],
            self._yaw, self._pitch,
            self._health, float(self._hunger), float(self._held_slot),
        ], dtype=np.float32)

        return {
            "image": np.zeros((224, 224, 3), dtype=np.uint8),
            "state": state,
            "nearby_blocks": data.get("nearby_blocks", {}).get("blocks", []),
            "nearby_entities": data.get("nearby_entities", []),
            "inventory": data.get("inventory", {}),
            "info": {
                "position": {"x": float(self._position[0]), "y": float(self._position[1]), "z": float(self._position[2])},
                "rotation": {"yaw": self._yaw, "pitch": self._pitch},
                "on_ground": self._on_ground,
                "dimension": self._dimension,
                "health": self._health,
                "hunger": self._hunger,
                "world": data.get("world", {}),
                "player_list": data.get("player_list", []),
            },
        }

    def _cached_obs(self) -> dict[str, Any]:
        state = np.array([
            self._position[0], self._position[1], self._position[2],
            self._yaw, self._pitch,
            self._health, float(self._hunger), float(self._held_slot),
        ], dtype=np.float32)
        return {
            "image": np.zeros((224, 224, 3), dtype=np.uint8),
            "state": state,
            "nearby_blocks": [],
            "nearby_entities": [],
            "inventory": {},
            "info": {
                "position": {"x": float(self._position[0]), "y": float(self._position[1]), "z": float(self._position[2])},
                "rotation": {"yaw": self._yaw, "pitch": self._pitch},
                "on_ground": self._on_ground,
                "dimension": self._dimension,
            },
        }

    def step(self, action: Any) -> dict[str, Any]:
        if not self._built:
            raise TargetStepError("target not built")
        if isinstance(action, dict):
            action_type = action.get("type", "")
            params = action.get("params", {})
        else:
            raise TargetStepError(f"action must be a dict, got {type(action).__name__}")

        if action_type not in VALID_ACTION_TYPES:
            raise TargetStepError(f"unknown action type: {action_type}")

        result = self._post_action(action_type, params)
        time.sleep(self._step_delay)
        self._step_idx += 1

        if action_type == "select_slot":
            slot = int(params.get("slot", 0))
            self._held_slot = max(0, min(8, slot))

        obs = self.observe()
        done = self._check_done()
        action_ok = result.get("ok", False)
        return {
            "obs": obs,
            "reward": 0.0,
            "done": done,
            "info": {
                "ok": action_ok,
                "step_idx": self._step_idx,
                "result": result.get("result", ""),
                "action_type": action_type,
            },
        }

    def _post_action(self, action_type: str, params: dict[str, Any]) -> dict[str, Any]:
        client = self._get_http()
        try:
            resp = client.post(
                f"{self._bridge_url}/action",
                json={"type": action_type, "params": params},
            )
            return resp.json()
        except Exception as exc:
            logger.warning("action post failed: %s", exc)
            return {"ok": False, "result": str(exc)}

    def _check_done(self) -> bool:
        return False

    def describe(self) -> dict[str, Any]:
        return {"type": "minecraft_java_bot", "actions": sorted(VALID_ACTION_TYPES)}

    def configure_session(self, session_ctx: dict[str, Any]) -> dict[str, Any]:
        return {"configured": True, "session_id": session_ctx.get("session_id")}

    def start_session(self, session_ctx: dict[str, Any]) -> dict[str, Any]:
        self._step_idx = 0
        return self.observe()

    def action_chunk(self, executable_action_chunk: dict[str, Any]) -> dict[str, Any]:
        actions = executable_action_chunk.get("actions", [executable_action_chunk])
        if isinstance(actions, dict):
            actions = [actions]
        last = {"obs": self.observe(), "done": False, "info": {}}
        for act in actions:
            last = self.step(act)
            if last.get("done"):
                break
        self._last_status = {
            "executed_steps": self._step_idx,
            "success": bool(last.get("info", {}).get("success")),
            "done": bool(last.get("done")),
        }
        return self._last_status

    def execution_status(self) -> dict[str, Any]:
        return getattr(self, "_last_status", {"status": "idle"})

    def cancel(self, reason: str) -> None:
        logger.info("MinecraftTarget cancelled: %s", reason)
        self._last_status = {"status": "cancelled", "reason": reason}


    def close(self) -> None:
        if self._http is not None:
            try:
                self._http.close()
            except Exception:
                pass
            self._http = None
        self._built = False
        logger.info("MinecraftTarget disconnected")
