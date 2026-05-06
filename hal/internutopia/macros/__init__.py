"""Minimal ``gm`` surface used by ``internutopia_extension`` robot cfgs.

``INTERNUTOPIA_ASSETS_PATH`` (parent directory of the ``robots/`` tree) may be
set explicitly.  When unset, we default to ``<repo>/asserts`` (falling back to
``<repo>/examples``) so bundled layouts such as ``asserts/robots/{...}/`` match
paths like ``gm.ASSET_PATH + '/robots/g1/policy/...'``.
"""

from __future__ import annotations

import os
from pathlib import Path
from types import SimpleNamespace


def _default_asset_path() -> str:
    # hal/internutopia/macros/__init__.py -> repo root is parents[3]
    repo = Path(__file__).resolve().parents[3]
    asserts = repo / "asserts"
    if asserts.is_dir():
        return str(asserts)
    ex = repo / "examples"
    if ex.is_dir():
        return str(ex)
    return ""


_env = os.environ.get("INTERNUTOPIA_ASSETS_PATH", "").strip().rstrip("/")
_asset = _env if _env else _default_asset_path()
gm = SimpleNamespace(ASSET_PATH=_asset)
