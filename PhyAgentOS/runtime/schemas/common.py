"""Shared schema helpers for runtime protocol documents."""

from __future__ import annotations

from datetime import datetime, timezone


def utc_now() -> datetime:
    """Return a timezone-aware UTC timestamp."""
    return datetime.now(timezone.utc)


def strip_ref(value: str, prefix: str) -> str:
    """Strip a runtime URI prefix such as ``target://`` when present."""
    return value[len(prefix):] if value.startswith(prefix) else value
