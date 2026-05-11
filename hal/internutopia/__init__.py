"""Shim top-level ``internutopia`` package for vendored HAL layout.

``internutopia.core`` / ``internutopia.bridge`` live as siblings ``core/`` and
``bridge/`` under ``hal/``.  ``internutopia.macros`` lives under this package
(see ``macros/``).
"""

from __future__ import annotations

from pathlib import Path

_pkg_dir = Path(__file__).resolve().parent
_hal_root = _pkg_dir.parent
__path__ = [str(_hal_root), str(_pkg_dir)]
