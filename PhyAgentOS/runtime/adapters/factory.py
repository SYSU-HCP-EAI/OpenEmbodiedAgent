"""Build target adapters from registry names."""

from __future__ import annotations

from PhyAgentOS.runtime.adapters.openpi.dummy_openpi_adapter import DummyOpenPIAdapter
from PhyAgentOS.runtime.watchdog.errors import AdapterError


def build_adapter(adapter_id: str):
    if adapter_id == "dummy_openpi_adapter":
        return DummyOpenPIAdapter()
    raise AdapterError(f"unsupported adapter: {adapter_id}")
