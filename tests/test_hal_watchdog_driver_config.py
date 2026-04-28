from __future__ import annotations

import json
from pathlib import Path
from unittest.mock import patch

import pytest

from hal.hal_watchdog import load_driver_config, watch_loop


def test_load_driver_config_returns_empty_when_omitted() -> None:
    assert load_driver_config(None) == {}


def test_load_driver_config_reads_json_object(tmp_path: Path) -> None:
    config_file = tmp_path / "driver.json"
    config_file.write_text(json.dumps({"target_navigation_backend": "real", "auto_start_remote": True}), encoding="utf-8")

    payload = load_driver_config(config_file)

    assert payload == {
        "target_navigation_backend": "real",
        "auto_start_remote": True,
    }

def test_load_driver_config_rejects_non_object_json(tmp_path: Path) -> None:
    config_file = tmp_path / "driver.json"
    config_file.write_text(json.dumps(["not", "an", "object"]), encoding="utf-8")

    with pytest.raises(ValueError, match="driver-config must be a JSON object"):
        load_driver_config(config_file)


def test_watch_loop_passes_driver_config_to_selected_driver(tmp_path: Path) -> None:
    env_file = tmp_path / "ENVIRONMENT.md"
    env_file.write_text(
        "# Environment State\n\n```json\n{\"schema_version\":\"oea.environment.v1\",\"scene_graph\":{\"nodes\":[],\"edges\":[]},\"robots\":{},\"objects\":{}}\n```\n",
        encoding="utf-8",
    )

    captured: dict[str, object] = {}

    class _DummyDriver:
        def __enter__(self):
            return self

        def __exit__(self, *_exc):
            return None

        def get_profile_path(self):
            return tmp_path / "missing.md"

        def is_connected(self):
            return True

        def connect(self):
            return True

        def load_scene(self, scene):
            captured["scene"] = scene

        def health_check(self):
            return True

        def get_scene(self):
            return {}

        def get_runtime_state(self):
            return {}

    def _fake_load_driver(name: str, **kwargs):
        captured["driver_name"] = name
        captured["driver_kwargs"] = kwargs
        return _DummyDriver()

    with patch("hal.drivers.load_driver", side_effect=_fake_load_driver):
        with patch("hal.hal_watchdog._poll_once", side_effect=KeyboardInterrupt):
            watch_loop(
                tmp_path,
                driver_name="go2_edu",
                gui=False,
                poll_interval=0.01,
                driver_kwargs={"target_navigation_backend": "real", "ssh_host": "192.168.86.17"},
                env_file=env_file,
            )

    assert captured["driver_name"] == "go2_edu"
    assert captured["driver_kwargs"] == {
        "gui": False,
        "target_navigation_backend": "real",
        "ssh_host": "192.168.86.17",
    }
