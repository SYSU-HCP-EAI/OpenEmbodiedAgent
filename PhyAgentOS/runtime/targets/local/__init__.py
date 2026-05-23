"""Local in-process runtime targets."""

from PhyAgentOS.runtime.targets.local.base import BaseLocalTarget
from PhyAgentOS.runtime.targets.local.dummy_sim_target import DummySimTarget

__all__ = ["BaseLocalTarget", "DummySimTarget"]
