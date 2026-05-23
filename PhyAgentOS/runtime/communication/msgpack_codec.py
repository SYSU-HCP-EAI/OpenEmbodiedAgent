"""Msgpack serialization helpers for runtime RPC messages."""

from __future__ import annotations

from typing import Any

import msgpack

from PhyAgentOS.runtime.communication.envelope import RuntimeEnvelope


def encode_msgpack(envelope: RuntimeEnvelope | dict[str, Any]) -> bytes:
    payload = envelope.model_dump(mode="json") if isinstance(envelope, RuntimeEnvelope) else envelope
    return msgpack.packb(payload, use_bin_type=True)


def decode_msgpack(data: bytes) -> RuntimeEnvelope:
    payload = msgpack.unpackb(data, raw=False)
    return RuntimeEnvelope.model_validate(payload)
