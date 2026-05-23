"""Build runtime targets from target registry specs."""

from __future__ import annotations

from collections.abc import Callable
from urllib.parse import urlparse

from PhyAgentOS.runtime.communication.target_ws_client import TargetWSClient
from PhyAgentOS.runtime.schemas import TargetSpec
from PhyAgentOS.runtime.targets.base import BaseRolloutTarget
from PhyAgentOS.runtime.targets.local.dummy_sim_target import DummySimTarget
from PhyAgentOS.runtime.targets.remote.proxy import RemoteTargetProxy
from PhyAgentOS.runtime.watchdog.errors import TargetBuildError


LocalTargetFactory = Callable[[TargetSpec], BaseRolloutTarget]
RemoteTargetFactory = Callable[[TargetSpec, TargetWSClient], BaseRolloutTarget]

_LOCAL_TARGET_FACTORIES: dict[str, LocalTargetFactory] = {}
_REMOTE_TARGET_FACTORIES: dict[str, RemoteTargetFactory] = {}


def register_local_target_runtime(runtime_name: str, factory: LocalTargetFactory) -> None:
    _LOCAL_TARGET_FACTORIES[runtime_name] = factory


def register_remote_target_runtime(runtime_name: str, factory: RemoteTargetFactory) -> None:
    _REMOTE_TARGET_FACTORIES[runtime_name] = factory


def build_target(target: TargetSpec, *, target_endpoint: str | None = None) -> BaseRolloutTarget:
    endpoint = target_endpoint or target.runtime.target_endpoint
    if not endpoint:
        raise TargetBuildError(f"target {target.id} does not define target_endpoint")
    if _is_local_target_endpoint(endpoint):
        return build_local_target(target)
    if _is_targetws_endpoint(endpoint):
        return build_remote_target(target, endpoint)
    raise TargetBuildError(f"unsupported target endpoint for {target.id}: {endpoint}")


def build_local_target(target: TargetSpec) -> BaseRolloutTarget:
    factory = _LOCAL_TARGET_FACTORIES.get(target.runtime.target_runtime)
    if factory is None:
        raise TargetBuildError(f"unsupported local target runtime: {target.runtime.target_runtime}")
    return factory(target)


def build_remote_target(target: TargetSpec, endpoint: str) -> BaseRolloutTarget:
    factory = _REMOTE_TARGET_FACTORIES.get(target.runtime.target_runtime)
    if factory is None:
        raise TargetBuildError(f"unsupported remote target runtime: {target.runtime.target_runtime}")
    client = TargetWSClient(endpoint, target_id=target.id)
    return factory(target, client)


def build_dummy_sim_target(target: TargetSpec) -> DummySimTarget:
    return DummySimTarget(target.config)


def build_remote_target_proxy(target: TargetSpec, client: TargetWSClient) -> RemoteTargetProxy:
    return RemoteTargetProxy(client, config=target.config)


def _is_local_target_endpoint(endpoint: str) -> bool:
    parsed = urlparse(endpoint)
    return parsed.scheme == "targetws" and parsed.netloc == "local"


def _is_targetws_endpoint(endpoint: str) -> bool:
    parsed = urlparse(endpoint)
    return parsed.scheme == "targetws"


register_local_target_runtime("DummySimTargetRuntime", build_dummy_sim_target)
register_remote_target_runtime("RemoteTargetProxy", build_remote_target_proxy)
