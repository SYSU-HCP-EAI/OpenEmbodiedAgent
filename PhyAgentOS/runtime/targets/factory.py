"""Build runtime targets from target registry specs."""

from __future__ import annotations

from PhyAgentOS.runtime.schemas import TargetSpec
from PhyAgentOS.runtime.targets.sim.dummy_sim_target import DummySimTarget
from PhyAgentOS.runtime.watchdog.errors import TargetBuildError


def build_target(target: TargetSpec):
    if target.type == "sim" and target.backend == "dummy":
        return DummySimTarget(target.config)
    raise TargetBuildError(f"unsupported target: {target.id} ({target.type}/{target.backend})")
