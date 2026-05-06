"""Work around OmniGraph + Replicator orchestrator bug with empty ``deque``.

``omni.replicator.core`` passes ``collections.deque`` to
``AttributeValueHelper.set`` for ``inputs:simTimesToWrite``.  In
``omni.graph`` 1.141.x, empty-array handling only runs for ``list`` /
``numpy.ndarray``, not ``deque``, so an empty deque reaches
``__attribute.set`` and raises::

    TypeError: Unable to write from unknown dtype, kind=i, size=0

Converting ``deque`` → ``list`` restores the intended early-return on empty
sequences and stops per-frame stderr spam when robot cameras / SDG run.
"""

from __future__ import annotations

from collections import deque
from typing import Any

_applied = False


def try_apply_omnigraph_deque_attribute_shim() -> bool:
    """Patch ``AttributeValueHelper.set`` once; return whether patch is active."""
    global _applied
    if _applied:
        return True
    try:
        from omni.graph.core._impl import attribute_values as _av
    except Exception:
        return False

    cls = _av.AttributeValueHelper
    orig = cls.set
    if getattr(orig, "_paos_deque_shim", False):
        _applied = True
        return True

    def set_patched(self: Any, new_value: Any, on_gpu: bool = False) -> Any:
        if isinstance(new_value, deque):
            new_value = list(new_value)
        return orig(self, new_value, on_gpu=on_gpu)

    set_patched._paos_deque_shim = True  # type: ignore[attr-defined]
    cls.set = set_patched  # type: ignore[method-assign]
    _applied = True
    return True
