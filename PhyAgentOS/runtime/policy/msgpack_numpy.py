"""Msgpack helpers compatible with OpenPI's websocket policy protocol."""

from __future__ import annotations

from typing import Any

import msgpack
import numpy as np


def _pack_array(obj: Any) -> Any:
    if isinstance(obj, (np.ndarray, np.generic)) and obj.dtype.kind in ("V", "O", "c"):
        raise ValueError(f"Unsupported dtype: {obj.dtype}")
    if isinstance(obj, np.ndarray):
        return {
            b"__ndarray__": True,
            b"data": obj.tobytes(),
            b"dtype": obj.dtype.str,
            b"shape": obj.shape,
        }
    if isinstance(obj, np.generic):
        return {
            b"__npgeneric__": True,
            b"data": obj.item(),
            b"dtype": obj.dtype.str,
        }
    return obj


def _unpack_array(obj: dict[Any, Any]) -> Any:
    if b"__ndarray__" in obj:
        return np.ndarray(buffer=obj[b"data"], dtype=np.dtype(obj[b"dtype"]), shape=obj[b"shape"])
    if b"__npgeneric__" in obj:
        return np.dtype(obj[b"dtype"]).type(obj[b"data"])
    return obj


def packb(payload: Any) -> bytes:
    return msgpack.packb(payload, default=_pack_array)


def unpackb(payload: bytes) -> Any:
    return _decode_keys(msgpack.unpackb(payload, object_hook=_unpack_array))


def _decode_keys(value: Any) -> Any:
    if isinstance(value, dict):
        decoded: dict[Any, Any] = {}
        for key, item in value.items():
            if isinstance(key, bytes):
                key = key.decode("utf-8")
            decoded[key] = _decode_keys(item)
        return decoded
    if isinstance(value, list):
        return [_decode_keys(item) for item in value]
    if isinstance(value, tuple):
        return tuple(_decode_keys(item) for item in value)
    return value
