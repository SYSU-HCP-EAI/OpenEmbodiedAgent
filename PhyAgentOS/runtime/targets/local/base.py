"""Base class for local in-process targets."""

from __future__ import annotations

from PhyAgentOS.runtime.targets.base import BaseRolloutTarget


class BaseLocalTarget(BaseRolloutTarget):
    """Base class for targets hosted inside the current runtime process."""
