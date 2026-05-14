"""Runtime registries for target and skill factories."""

from __future__ import annotations

from collections.abc import Callable

from PhyAgentOS.runtime.schemas import TargetSpec
from PhyAgentOS.runtime.skills.base import BaseSkillRuntime
from PhyAgentOS.runtime.skills.vla.openpi_sim_runtime import OpenPISimSkillRuntime
from PhyAgentOS.runtime.targets.factory import build_target
from PhyAgentOS.runtime.watchdog.errors import SchemaValidationError, TargetBuildError


TargetFactory = Callable[[TargetSpec], object]
SkillFactory = Callable[[], BaseSkillRuntime]


class TargetRuntimeRegistry:
    """Map target runtime keys to target factories."""

    def __init__(self):
        self._factories: dict[str, TargetFactory] = {}
        self.register("sim:dummy", build_target)

    def register(self, name: str, factory: TargetFactory) -> None:
        self._factories[name] = factory

    def build(self, target_spec: TargetSpec):
        key = self._key_for(target_spec)
        factory = self._factories.get(key)
        if factory is None:
            raise TargetBuildError(
                f"unsupported target: {target_spec.id} ({target_spec.type}/{target_spec.backend})"
            )
        return factory(target_spec)

    def _key_for(self, target_spec: TargetSpec) -> str:
        return f"{target_spec.type}:{target_spec.backend or ''}"


class SkillRuntimeRegistry:
    """Map skill runtime names to skill runtime factories."""

    def __init__(self):
        self._factories: dict[str, SkillFactory] = {}
        self.register("OpenPISimSkillRuntime", OpenPISimSkillRuntime)

    def register(self, name: str, factory: SkillFactory) -> None:
        self._factories[name] = factory

    def build(self, runtime_name: str) -> BaseSkillRuntime:
        factory = self._factories.get(runtime_name)
        if factory is None:
            raise SchemaValidationError(f"unsupported skill runtime: {runtime_name}")
        return factory()
