from __future__ import annotations

import sys
from pathlib import Path
from typing import Tuple

import numpy as np
import torch


def ensure_manipulation_lab_on_path(repo_root: str | Path) -> Path:
    """Ensure manipulation-lab python package is importable."""
    root = Path(repo_root).resolve()
    pkg_root = root / "manipulation-lab" / "source" / "leisaac"
    pkg_str = str(pkg_root)
    if pkg_str not in sys.path:
        sys.path.insert(0, pkg_str)
    return pkg_root


def get_robot_root_pose(env, env_id: int = 0) -> Tuple[np.ndarray, np.ndarray]:
    """Return root pose (position, quaternion wxyz) from leisaac env."""
    robot = env.scene["robot"]
    pos = robot.data.root_pos_w[env_id].detach().cpu().numpy().astype(np.float64)
    quat = robot.data.root_quat_w[env_id].detach().cpu().numpy().astype(np.float64)
    return pos, quat


def move_to_point_velocity(
    pos_xy: np.ndarray,
    goal_xy: np.ndarray,
    *,
    max_linear: float = 0.12,
    linear_gain: float = 0.5,
    stop_dist: float = 0.05,
) -> np.ndarray:
    """Simple planar move-to-point velocity in world frame."""
    err = np.asarray(goal_xy, dtype=np.float64) - np.asarray(pos_xy, dtype=np.float64)
    dist = float(np.linalg.norm(err))
    if dist < stop_dist:
        return np.zeros(3, dtype=np.float32)
    direction = err / max(dist, 1e-8)
    speed = min(float(max_linear), float(linear_gain) * dist)
    return np.array([direction[0] * speed, direction[1] * speed, 0.0], dtype=np.float32)


def build_xlerobot_base_action(
    base_cmd_vxvyw: np.ndarray | torch.Tensor,
    *,
    num_envs: int,
    device: torch.device | str,
) -> torch.Tensor:
    """Build leisaac full action tensor (17-d), filling only base channels [12:15]."""
    from leisaac.devices.xlerobot_base_conversion import compute_omniwheel_velocities

    if isinstance(base_cmd_vxvyw, torch.Tensor):
        cmd = base_cmd_vxvyw.to(device=device, dtype=torch.float32).view(1, 3)
    else:
        cmd = torch.tensor(base_cmd_vxvyw, dtype=torch.float32, device=device).view(1, 3)
    action = torch.zeros((int(num_envs), 17), dtype=torch.float32, device=device)
    wheels = compute_omniwheel_velocities(cmd).view(1, 3)
    action[:, 12:15] = wheels.repeat(int(num_envs), 1)
    return action
