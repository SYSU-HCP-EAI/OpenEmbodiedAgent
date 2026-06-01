"""OpenPI-compatible websocket server for LeRobot PI0/PI0.5 checkpoints."""

from __future__ import annotations

import argparse
import asyncio
import http
import time
import traceback
from pathlib import Path
from typing import Any

import numpy as np

from PhyAgentOS.runtime.policy.msgpack_numpy import packb, unpackb
from PhyAgentOS.runtime.watchdog.errors import PolicyProtocolError


def import_lerobot() -> tuple[Any, Any]:
    try:
        from lerobot.policies.factory import make_pre_post_processors
        from lerobot.policies.pi0 import PI0Policy
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "Missing LeRobot PI0 policy dependencies. Run this server in the lerobot-pi "
            "environment that can execute openpi/scripts/minimal_lerobot_pi0_infer.py."
        ) from exc
    return PI0Policy, make_pre_post_processors


def libero_observation_to_lerobot_frame(
    observation: dict[str, Any],
    *,
    image_size: int = 256,
    empty_camera_size: int = 224,
) -> dict[str, Any]:
    task = observation.get("task", observation.get("prompt"))
    if task is None:
        raise PolicyProtocolError("policy observation missing `prompt` or `task`")
    try:
        image = observation["observation/image"]
        wrist_image = observation["observation/wrist_image"]
        state = observation["observation/state"]
    except KeyError as exc:
        raise PolicyProtocolError(f"policy observation missing `{exc.args[0]}`") from exc

    state_array = np.asarray(state, dtype=np.float32)
    if state_array.shape != (8,):
        raise PolicyProtocolError(f"`observation/state` must have shape [8], got {state_array.shape}")

    return {
        "observation.images.image": _image_to_chw_tensor(image, image_size),
        "observation.images.image2": _image_to_chw_tensor(wrist_image, image_size),
        "observation.images.empty_camera_0": _empty_image_tensor(empty_camera_size),
        "observation.state": _torch().as_tensor(state_array, dtype=_torch().float32),
        "task": str(task),
    }


def action_to_numpy(action: Any) -> np.ndarray:
    torch = _torch(optional=True)
    if torch is not None and isinstance(action, torch.Tensor):
        action = action.detach().cpu().numpy()
    elif isinstance(action, dict):
        for key in ("actions", "action"):
            if key in action:
                return action_to_numpy(action[key])
    actions = np.asarray(action, dtype=np.float32)
    if actions.ndim == 1:
        actions = actions[None, :]
    if actions.ndim != 2:
        raise PolicyProtocolError(f"policy action must have shape [A] or [T,A], got {actions.shape}")
    if actions.shape[1] < 7:
        raise PolicyProtocolError(f"policy action must have at least 7 dims, got {actions.shape}")
    return np.ascontiguousarray(actions[:, :7], dtype=np.float32)


class LeRobotPI0Policy:
    def __init__(self, model_dir: str | Path, *, tokenizer_name: str | None, device: str):
        model_dir = Path(model_dir).expanduser().resolve()
        if not model_dir.exists():
            raise PolicyProtocolError(f"model directory does not exist: {model_dir}")
        if not (model_dir / "model.safetensors").exists():
            raise PolicyProtocolError(f"expected model.safetensors under: {model_dir}")

        torch = _torch()
        PI0Policy, make_pre_post_processors = import_lerobot()
        self.model_dir = model_dir
        self.device = torch.device(device)
        self.policy = PI0Policy.from_pretrained(str(model_dir)).to(self.device).eval()
        overrides: dict[str, Any] = {"device_processor": {"device": str(self.device)}}
        if tokenizer_name is not None:
            overrides["tokenizer_processor"] = {"tokenizer_name": tokenizer_name}
        self.preprocess, self.postprocess = make_pre_post_processors(
            self.policy.config,
            str(model_dir),
            preprocessor_overrides=overrides,
        )
        self.metadata = {
            "backend": "lerobot_pi0",
            "model_dir": str(model_dir),
            "device": str(self.device),
            "action_dim": 7,
            "chunk_size": int(getattr(self.policy.config, "chunk_size", 0) or 0) or None,
        }

    def infer(self, observation: dict[str, Any]) -> dict[str, Any]:
        torch = _torch()
        started = time.perf_counter()
        frame = libero_observation_to_lerobot_frame(observation)
        batch = self.preprocess(frame)
        with torch.inference_mode():
            try:
                action = self.policy.select_action(batch)
            except Exception as first_error:
                try:
                    action = self.policy.select_action(frame)
                except Exception:
                    raise first_error
            action = self.postprocess(action)
        actions = action_to_numpy(action)
        return {
            "actions": actions,
            "policy_meta": {
                "backend": "lerobot_pi0",
                "model_dir": str(self.model_dir),
                "policy_latency_ms": (time.perf_counter() - started) * 1000,
                "chunk_size": int(actions.shape[0]),
                "action_dim": int(actions.shape[1]),
            },
        }


async def serve_policy(policy: LeRobotPI0Policy, *, host: str, port: int) -> None:
    try:
        import websockets
        import websockets.asyncio.server as websocket_server
    except ModuleNotFoundError as exc:
        raise SystemExit("Missing `websockets`; install PhyAgentOS runtime dependencies in this env.") from exc

    async def handler(websocket):
        await websocket.send(packb(policy.metadata))
        while True:
            try:
                message = await websocket.recv()
                if isinstance(message, str):
                    raise PolicyProtocolError("expected binary msgpack policy request")
                response = policy.infer(unpackb(message))
                await websocket.send(packb(response))
            except websockets.ConnectionClosed:
                break
            except Exception:
                await websocket.send(traceback.format_exc())
                await websocket.close(code=1011, reason="Internal policy server error")
                raise

    async with websocket_server.serve(
        handler,
        host,
        port,
        compression=None,
        max_size=None,
        process_request=_health_check,
    ) as server:
        await server.serve_forever()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model-dir", required=True)
    parser.add_argument("--tokenizer-name", default=None)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()
    policy = LeRobotPI0Policy(args.model_dir, tokenizer_name=args.tokenizer_name, device=args.device)
    asyncio.run(serve_policy(policy, host=args.host, port=args.port))


def _health_check(connection, request):
    if getattr(request, "path", None) == "/healthz":
        return connection.respond(http.HTTPStatus.OK, "OK\n")
    return None


def _image_to_chw_tensor(image: Any, size: int):
    torch = _torch()
    array = np.asarray(image)
    if array.size == 0:
        raise PolicyProtocolError("image observation is empty")
    if array.ndim != 3:
        raise PolicyProtocolError(f"image observation must have rank 3, got {array.shape}")
    if array.shape[0] == 3:
        chw = array
    elif array.shape[-1] == 3:
        chw = np.transpose(array, (2, 0, 1))
    else:
        raise PolicyProtocolError(f"image observation must be CHW or HWC RGB, got {array.shape}")
    tensor = torch.as_tensor(np.ascontiguousarray(chw), dtype=torch.float32)
    if np.issubdtype(array.dtype, np.integer):
        tensor = tensor / 255.0
    if tensor.shape[-2:] != (size, size):
        tensor = torch.nn.functional.interpolate(
            tensor[None],
            size=(size, size),
            mode="bilinear",
            align_corners=False,
        )[0]
    return tensor


def _empty_image_tensor(size: int):
    return _torch().zeros((3, size, size), dtype=_torch().float32)


def _torch(optional: bool = False):
    try:
        import torch
    except ModuleNotFoundError:
        if optional:
            return None
        raise
    return torch


if __name__ == "__main__":
    main()
