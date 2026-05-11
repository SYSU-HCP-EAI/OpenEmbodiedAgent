"""Install bundled InternUtopia layout (under ``hal/``) onto ``sys.path``.

Vendored layout::

    hal/
      internutopia/     # shim package (see ``internutopia/__init__.py``)
      core/             # ``internutopia.core``
      bridge/           # ``internutopia.bridge``
      internutopia_extension/  # top-level ``internutopia_extension``

External ``pythonpath`` entries in driver JSON remain supported and are
applied after this hook (they can override a system-wide install).

Robot weights / USD paths in configs use ``internutopia.macros.gm.ASSET_PATH``.
If ``INTERNUTOPIA_ASSETS_PATH`` is unset, it defaults to ``<repo>/examples`` so
``... + '/robots/g1/policy/...'`` resolves to ``asserts/robots/g1/...`` (or
``examples/robots/...`` when ``asserts/`` is absent).
"""

from __future__ import annotations

import sys
from pathlib import Path

_done = False


def hal_package_root() -> Path:
    """Directory that contains ``core/``, ``bridge/``, ``internutopia/``."""
    return Path(__file__).resolve().parent


def ensure_bundled_internutopia_sys_path() -> None:
    """Prepend ``hal/`` to ``sys.path`` once so ``internutopia*`` imports resolve."""
    global _done
    if _done:
        return
    root = str(hal_package_root())
    if root not in sys.path:
        sys.path.insert(0, root)
    _done = True
