from __future__ import annotations

from hal.drivers import load_driver, list_drivers
from hal.drivers.xlerobot_sim_driver import XLerobotSimDriver


def test_xlerobot_sim_driver_is_registered() -> None:
    assert 'xlerobot_sim' in list_drivers()
    driver = load_driver('xlerobot_sim', gui=False, robot_id='xlerobot_a')
    assert isinstance(driver, XLerobotSimDriver)
    assert driver.robot_id == 'xlerobot_a'


def test_xlerobot_sim_driver_runtime_defaults() -> None:
    driver = XLerobotSimDriver(gui=False, robot_id='xlerobot_a')

    runtime = driver.get_runtime_state()

    robot = runtime['robots']['xlerobot_a']
    assert robot['connection_state']['status'] == 'idle'
    assert robot['nav_state']['mode'] == 'mobile_manipulation'
    assert robot['nav_state']['status'] == 'idle'


def test_xlerobot_sim_driver_scene_runtime_key() -> None:
    driver = XLerobotSimDriver(gui=False, robot_id='xlerobot_a')

    scene = driver.get_scene()

    assert 'xlerobot_runtime' in scene
    assert scene['xlerobot_runtime']['state'] == 'idle'


def test_xlerobot_sim_driver_navigation_is_disabled_by_default() -> None:
    driver = XLerobotSimDriver(gui=False, robot_id='xlerobot_a')

    result = driver.execute_action('navigate_to_named', {'waypoint_key': 'table'})

    assert 'navigation is disabled' in result
    assert 'root prismatic joints' in result