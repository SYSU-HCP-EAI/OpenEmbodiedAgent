"""Synchronous WebSocket client for target runtime RPC."""

from __future__ import annotations

import time
from typing import Any
from urllib.parse import urlparse, urlunparse

from PhyAgentOS.runtime.communication.envelope import RuntimeEnvelope
from PhyAgentOS.runtime.communication.msgpack_codec import decode_msgpack, encode_msgpack
from PhyAgentOS.runtime.watchdog.errors import TargetConnectionError, TargetProtocolError


class TargetWSClient:
    """Minimal msgpack-over-WebSocket client for target runtime calls."""

    def __init__(
        self,
        endpoint: str,
        *,
        target_id: str | None = None,
        timeout_s: float = 30.0,
        connect_on_init: bool = False,
    ):
        self.endpoint = endpoint
        self.target_id = target_id
        self.timeout_s = float(timeout_s)
        self._seq = 0
        self._ws = None
        if connect_on_init:
            self.connect()

    def connect(self) -> None:
        if self._ws is not None:
            return
        try:
            import websocket
        except ImportError as exc:
            raise TargetConnectionError(
                "websocket-client is required for targetws:// endpoints"
            ) from exc

        try:
            self._ws = websocket.create_connection(
                self._websocket_url(),
                timeout=self.timeout_s,
            )
        except Exception as exc:  # pragma: no cover - depends on external transport
            raise TargetConnectionError(f"failed to connect target endpoint {self.endpoint}: {exc}") from exc

    def call(
        self,
        message_type: str,
        payload: dict[str, Any] | None = None,
        *,
        session_id: str | None = None,
        skill_id: str | None = None,
        episode_id: str | None = None,
        trace_id: str | None = None,
    ) -> dict[str, Any]:
        self.connect()
        self._seq += 1
        request = RuntimeEnvelope(
            type=message_type,
            session_id=session_id,
            target_id=self.target_id,
            skill_id=skill_id,
            episode_id=episode_id,
            seq=self._seq,
            timestamp_ns=time.time_ns(),
            trace_id=trace_id,
            payload=payload or {},
        )
        try:
            self._ws.send_binary(encode_msgpack(request))
            response = decode_msgpack(self._ws.recv())
        except Exception as exc:
            raise TargetProtocolError(f"target RPC {message_type} failed: {exc}") from exc

        if response.type == "runtime.error":
            error_code = response.payload.get("error_code", "TARGET_RPC_ERROR")
            message = response.payload.get("message", "target runtime returned an error")
            raise TargetProtocolError(f"{error_code}: {message}")
        if response.seq < request.seq:
            raise TargetProtocolError(
                f"target RPC {message_type} returned stale seq {response.seq}, expected >= {request.seq}"
            )
        return response.payload

    def close(self) -> None:
        if self._ws is None:
            return
        try:
            self._ws.close()
        finally:
            self._ws = None

    @property
    def connected(self) -> bool:
        return self._ws is not None

    def _websocket_url(self) -> str:
        parsed = urlparse(self.endpoint)
        if parsed.scheme != "targetws":
            raise TargetConnectionError(f"expected targetws:// endpoint, got {self.endpoint}")
        return urlunparse(("ws", parsed.netloc, parsed.path, parsed.params, parsed.query, parsed.fragment))
