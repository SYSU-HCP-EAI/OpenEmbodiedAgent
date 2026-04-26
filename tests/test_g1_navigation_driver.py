from __future__ import annotations

from hal.drivers import load_driver
from hal.drivers.g1_navigation_driver import G1NavigationDriver


def test_g1_navigation_driver_registered() -> None:
    with load_driver("g1_navigation", gui=False) as driver:
        assert isinstance(driver, G1NavigationDriver)


def test_g1_navigation_driver_reports_robot_camera_mounts_in_runtime_state() -> None:
    driver = G1NavigationDriver(
        gui=False,
        robot_id="g1_a",
        robot_cameras={
            "body_view": {
                "parent": "/World/env_0/robots/g1/pelvis",
                "translation": [0.18, 0.0, 0.72],
                "orientation": [0.9813, 0.0, 0.192, 0.0],
                "purpose": "robot_body_view",
            },
        },
        robot_camera_resolution=[640, 400],
    )

    runtime = driver.get_runtime_state()
    cameras = runtime["robots"]["g1_a"]["cameras"]
    scene = driver.get_scene()

    assert runtime["robots"]["g1_a"]["nav_state"]["mode"] == "navigation"
    assert cameras["body_view"]["status"] == "configured"
    assert cameras["body_view"]["parent"] == "/World/env_0/robots/g1/pelvis"
    assert cameras["body_view"]["prim_path"] == "/World/env_0/robots/g1/pelvis/paos_body_view"
    assert scene["navigation_runtime"]["robot_cameras"] == cameras


def test_g1_navigation_driver_extracts_g1_robot_obs() -> None:
    driver = G1NavigationDriver(gui=False, robot_id="g1_a")
    driver._api = object()
    driver._last_obs = {"g1": {"position": [1.5, 2.25, 0.8]}}

    runtime = driver.get_runtime_state()

    assert runtime["robots"]["g1_a"]["connection_state"]["status"] == "connected"
    assert runtime["robots"]["g1_a"]["robot_pose"]["x"] == 1.5
    assert runtime["robots"]["g1_a"]["robot_pose"]["y"] == 2.25