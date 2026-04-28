"""Closed-loop SmolVLA pick helpers for the PiperGo2 manipulation driver.

Ported 1:1 from ``test_isaacsim_with_vla_pick.py`` (Step B VLA pick +
post-pick gripper squeeze). All Isaac Sim / torch / lerobot imports are
deferred so that importing this module is cheap even when the watchdog
is running without VLA configured.

Responsibilities:
- ``VLAController`` wraps ``SmolVLAPolicy`` and applies per-tick safety
  clipping (joint limits + per-tick delta). It does NOT touch Isaac Sim
  directly; the caller supplies state/image callbacks.
- ``attach_cameras`` attaches the three training-time cameras under the
  configured parent prims.
- ``build_articulation_view`` initializes an ``ArticulationView`` for the
  PiperGo2 so we can read joint positions per tick.
- ``execute_pick`` runs the closed-loop pick + post-pick close grip.
"""

from __future__ import annotations

from typing import Any, Callable, Iterable

import numpy as np

__all__ = [
    "VLAController",
    "attach_cameras",
    "build_articulation_view",
    "execute_pick",
    "grab_rgb",
    "read_piper_state7",
    "read_piper_arm8",
    "read_cube_world_z",
    "PIPER_ARM_JOINTS_8",
    "PIPER_ARM_JOINTS_6",
    "DEFAULT_PIPER_JOINT_LIMITS",
]


PIPER_ARM_JOINTS_8 = [
    "piper_j1", "piper_j2", "piper_j3",
    "piper_j4", "piper_j5", "piper_j6",
    "piper_j7", "piper_j8",
]
PIPER_ARM_JOINTS_6 = PIPER_ARM_JOINTS_8[:6]

# Conservative Piper joint limits used when the JSON does not override.
DEFAULT_PIPER_JOINT_LIMITS: list[tuple[float, float]] = [
    (-2.618, 2.618),    # j1
    (0.00, 3.14),       # j2
    (-2.967, 0.00),     # j3
    (-1.74, 1.74),      # j4
    (-1.22, 1.22),      # j5
    (-2.0943, 2.0943),  # j6
    (0.00, 0.035),      # j7 (finger +)
    (-0.035, 0.00),     # j8 (finger -)
]


def _log(msg: str) -> None:
    print(f"[vla-pick] {msg}", flush=True)


class VLAController:
    """Thin wrapper around SmolVLAPolicy.

    State / image acquisition is injected via callbacks so this class has
    no direct dependency on Isaac Sim. The caller is responsible for:
    - providing ``read_state7`` returning a 7-D vector
      ``[j1..j6, (j7 - j8)_meters]`` (Isaac-native gripper unit); the
      controller will rescale it into the training unit internally via
      ``state_gripper_scale``.
    - providing ``read_images`` returning ``{camera_name: HxWx3 uint8}``
      with all three training camera names (typically
      ``camera1 / camera2 / camera3``) or ``None`` if any frame is empty.
    """

    def __init__(
        self,
        *,
        ckpt_path: str,
        task_text: str,
        device: str = "auto",
        gripper_scale: float = 0.25,
        gripper_bias: float = 0.0,
        state_gripper_scale: float = 4.0,
        joint_limits: Iterable[tuple[float, float]] = DEFAULT_PIPER_JOINT_LIMITS,
        max_delta_arm: float = 0.45,
        max_delta_gripper: float = 0.06,
        read_state7: Callable[[], np.ndarray | None] | None = None,
        read_images: Callable[[], dict[str, np.ndarray] | None] | None = None,
    ) -> None:
        import torch
        from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
        try:
            from lerobot.policies import make_pre_post_processors
        except ImportError:
            from lerobot.policies.factory import make_pre_post_processors  # type: ignore
        from lerobot.policies.utils import prepare_observation_for_inference

        if device == "auto":
            device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = torch.device(device)
        self.task_text = task_text
        self.gripper_scale = float(gripper_scale)
        self.gripper_bias = float(gripper_bias)
        self.state_gripper_scale = float(state_gripper_scale)
        self.joint_limits = list(joint_limits)
        self.max_delta_arm = float(max_delta_arm)
        self.max_delta_gripper = float(max_delta_gripper)
        self.read_state7 = read_state7
        self.read_images = read_images
        self._prepare_obs = prepare_observation_for_inference
        self._torch = torch

        _log(f"loading policy from {ckpt_path} on {self.device} ...")
        self.policy = SmolVLAPolicy.from_pretrained(ckpt_path).to(self.device).eval()
        self.preprocess, self.postprocess = make_pre_post_processors(
            self.policy.config,
            ckpt_path,
            preprocessor_overrides={"device_processor": {"device": str(self.device)}},
        )
        _log(
            "policy ready: "
            f"chunk_size={self.policy.config.chunk_size}, "
            f"n_obs_steps={self.policy.config.n_obs_steps}"
        )
        self.last_images: dict[str, np.ndarray] = {}
        self.last_state: np.ndarray | None = None

    def set_n_action_steps(self, n: int) -> None:
        try:
            self.policy.config.n_action_steps = int(n)
            _log(f"overrode n_action_steps -> {int(n)}")
        except Exception as exc:
            _log(f"WARNING: could not override n_action_steps: {exc}")

    def reset(self) -> None:
        if hasattr(self.policy, "reset"):
            try:
                self.policy.reset()
            except Exception as exc:
                _log(f"WARNING: policy.reset() failed: {exc}")

    def _build_obs(self) -> dict | None:
        if self.read_state7 is None or self.read_images is None:
            _log("ERR: read_state7 / read_images callbacks not wired")
            return None
        state7 = self.read_state7()
        if state7 is None:
            _log("WARNING: state read returned None")
            return None
        # Rescale gripper dim from raw prismatic-meters (Isaac unit) into the
        # Piper servo-angle unit the model was trained on.
        state7 = np.asarray(state7, dtype=np.float32).copy()
        state7[6] = state7[6] * self.state_gripper_scale
        imgs = self.read_images()
        if not imgs:
            _log("WARNING: image read returned empty")
            return None
        obs: dict[str, Any] = {"observation.state": state7}
        for name, rgb in imgs.items():
            obs[f"observation.images.{name}"] = rgb
        self.last_state = state7
        self.last_images = dict(imgs)
        return obs

    def infer_step(
        self, current_8d: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray] | None:
        """Build obs, run policy, return ``(raw_act_7d, safe_target_8d)``."""
        obs = self._build_obs()
        if obs is None:
            return None
        with self._torch.inference_mode():
            frame = self._prepare_obs(
                dict(obs), self.device, task=self.task_text, robot_type=""
            )
            batch = self.preprocess(frame)
            act = self.policy.select_action(batch)
            act = self.postprocess(act)
            act7 = act.squeeze(0).detach().cpu().float().numpy()
        target_8d = self._to_piper_8d(act7, np.asarray(current_8d, dtype=np.float32))
        return act7, target_8d

    def _to_piper_8d(self, act7: np.ndarray, current_8d: np.ndarray) -> np.ndarray:
        target = current_8d.copy()
        target[:6] = act7[:6]
        g = float(act7[6]) * self.gripper_scale + self.gripper_bias
        j7_lo, j7_hi = self.joint_limits[6]
        g = max(j7_lo * 2.0, min(j7_hi * 2.0, g))
        target[6] = g / 2.0
        target[7] = -g / 2.0

        max_delta = np.array(
            [self.max_delta_arm] * 6 + [self.max_delta_gripper] * 2,
            dtype=np.float32,
        )
        delta = np.clip(target - current_8d, -max_delta, max_delta)
        safe = current_8d + delta
        for i, (lo, hi) in enumerate(self.joint_limits):
            safe[i] = float(np.clip(safe[i], lo, hi))
        return safe.astype(np.float32)


def build_articulation_view(robot_prim_path: str):
    """Return ``(view, joint_indices)`` or ``(None, None)``."""
    try:
        try:
            from isaacsim.core.prims import ArticulationView  # type: ignore
        except ImportError:
            from omni.isaac.core.articulations import ArticulationView  # type: ignore
        view = ArticulationView(
            prim_paths_expr=robot_prim_path,
            name="pipergo2_view_vla",
        )
        view.initialize()
        joint_indices = {name: idx for idx, name in enumerate(view.dof_names)}
        _log(f"ArticulationView ready, {len(joint_indices)} DOFs")
        return view, joint_indices
    except Exception as exc:
        _log(f"WARNING: ArticulationView init failed: {exc}")
        return None, None


def attach_cameras(
    *,
    stage,
    env,
    env_lock,
    hold_action: dict,
    mounts: dict,
    resolution: tuple[int, int],
    warmup_steps: int = 60,
) -> tuple[dict, set[str]]:
    """Attach Isaac Sim ``Camera`` prims per the ``mounts`` config.

    Returns ``(cameras, flip_horizontal_names)``. Cameras whose parent
    prim is missing are skipped with a warning.
    """
    try:
        from isaacsim.sensors.camera import Camera  # type: ignore
    except ImportError:
        from omni.isaac.sensor import Camera  # type: ignore

    cameras: dict = {}
    flip_set: set[str] = set()
    for cam_name, mount in mounts.items():
        parent = mount.get("parent")
        if not parent:
            continue
        parent_prim = stage.GetPrimAtPath(parent)
        if not parent_prim.IsValid():
            _log(f"WARNING: camera parent not found: {parent} (skip {cam_name})")
            continue
        prim_path = f"{parent}/vla_{cam_name}"
        if bool(mount.get("flip_horizontal", False)):
            flip_set.add(cam_name)
        try:
            cam = Camera(
                prim_path=prim_path,
                name=f"vla_{cam_name}",
                resolution=(int(resolution[0]), int(resolution[1])),
                translation=np.array(mount["translation"], dtype=np.float32),
                orientation=np.array(mount["orientation"], dtype=np.float32),
            )
            cam.set_focal_length(float(mount.get("focal_length", 2.4)))
            cam.initialize()
            clip = mount.get("clipping_range")
            if clip:
                try:
                    cam.set_clipping_range(float(clip[0]), float(clip[1]))
                except Exception as clip_exc:
                    _log(f"WARNING: {cam_name} set_clipping_range failed: {clip_exc}")
            cameras[cam_name] = cam
            _log(f"attached {cam_name} @ {prim_path} clip={clip}")
        except Exception as exc:
            _log(f"WARNING: failed to attach {cam_name}: {exc}")

    if cameras and warmup_steps > 0:
        _log(f"warming up cameras ({warmup_steps} env.step with hold_action)")
        for _ in range(warmup_steps):
            with env_lock:
                env.step(action=hold_action)
    return cameras, flip_set


def grab_rgb(
    cam, env, env_lock, hold_action: dict, max_retries: int = 8
) -> np.ndarray | None:
    """Pull a single HxWx3 uint8 frame from ``cam``; tick env until valid."""
    last_rgb = None
    for _ in range(max_retries):
        rgba = cam.get_rgba()
        if rgba is not None and rgba.size > 0:
            rgb = rgba[..., :3].astype(np.uint8)
            last_rgb = rgb
            if rgb.any():
                return rgb
        with env_lock:
            env.step(action=hold_action)
    return last_rgb


def read_piper_state7(robot_view, joint_indices) -> np.ndarray | None:
    """Return 7-D ``[j1..j6, (j7 - j8)_meters]`` state vector or ``None``."""
    if robot_view is None or joint_indices is None:
        return None
    q = robot_view.get_joint_positions()
    if q is None:
        return None
    q = np.asarray(q)
    if q.ndim == 2:
        q = q[0]
    arm6 = np.array(
        [q[joint_indices[n]] for n in PIPER_ARM_JOINTS_6], dtype=np.float32
    )
    j7 = float(q[joint_indices["piper_j7"]]) if "piper_j7" in joint_indices else 0.0
    j8 = float(q[joint_indices["piper_j8"]]) if "piper_j8" in joint_indices else 0.0
    return np.concatenate([arm6, [j7 - j8]]).astype(np.float32)


def read_piper_arm8(robot_view, joint_indices) -> np.ndarray | None:
    """Return full 8-D ``[j1..j6, j7, j8]`` pose or ``None``."""
    if robot_view is None or joint_indices is None:
        return None
    q = robot_view.get_joint_positions()
    if q is None:
        return None
    q = np.asarray(q)
    if q.ndim == 2:
        q = q[0]
    return np.array(
        [float(q[joint_indices[n]]) for n in PIPER_ARM_JOINTS_8],
        dtype=np.float32,
    )


def read_cube_world_z(env, stage, cube_prim_path: str) -> float | None:
    """Return world-frame Z of the cube. Prefers InternUtopia's object registry."""
    try:
        obj_name = cube_prim_path.rstrip("/").split("/")[-1]
        rb = env.runner.get_obj(obj_name)
        if rb is not None:
            pos, _ = rb.get_world_pose()
            return float(pos[2])
    except Exception:
        pass
    try:
        from pxr import UsdGeom  # type: ignore
    except Exception:
        return None
    for path in (
        cube_prim_path,
        f"/World/env_0{cube_prim_path}" if cube_prim_path.startswith("/World/") else cube_prim_path,
        cube_prim_path.replace("/World/", "/World/env_0/objects/", 1),
        cube_prim_path.replace("/World/", "/World/env_0/", 1),
    ):
        prim = stage.GetPrimAtPath(path)
        if prim and prim.IsValid():
            mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
            return float(mat.ExtractTranslation()[2])
    return None


def execute_pick(
    *,
    controller: VLAController,
    env,
    env_lock,
    nav_action_name: str,
    arm_action_name: str,
    hold_action: dict,
    read_arm8: Callable[[], np.ndarray | None],
    read_cube_z: Callable[[], float | None],
    hold_xy: tuple[float, float],
    max_ticks: int,
    lift_threshold: float,
    sim_steps_per_action: int,
    close_gripper_ramp_ticks: int,
    close_gripper_hold_sim_steps: int,
    max_per_tick_delta_gripper: float,
    dump_root: str | None = None,
    dump_every: int = 1,
) -> dict:
    """Run the closed VLA pick loop + post-pick gripper squeeze.

    Returns ``{success, ticks_used, terminate, initial_cube_z,
    final_cube_z, final_arm_8d}``.

    If ``dump_root`` is given, every ``dump_every``-th tick saves the three
    camera frames (exactly what the policy saw) + state/action/cube_z to
    ``dump_root/tick_XXX/`` — mirrors ``execute_vla_pick`` in
    ``test_isaacsim_with_vla_pick.py`` for offline visual comparison.
    """
    import json as _json
    import os as _os

    controller.reset()
    initial_cube_z = read_cube_z()
    _log(f"initial cube z = {initial_cube_z}")
    hold_xy_action = (float(hold_xy[0]), float(hold_xy[1]), 0.0)

    dump_image_mod = None
    if dump_root:
        _os.makedirs(dump_root, exist_ok=True)
        try:
            from PIL import Image as dump_image_mod  # type: ignore
        except ImportError:
            dump_image_mod = None
        _log(
            f"per-tick dump enabled -> {dump_root} (every {dump_every} tick, "
            f"format={'png' if dump_image_mod is not None else 'npy'})"
        )

    success = False
    terminate_flag = False
    final_cube_z = initial_cube_z
    ticks_used = 0
    final_arm_8d: np.ndarray | None = None

    for tick in range(max_ticks):
        current_8d = read_arm8()
        if current_8d is None:
            _log("ERR: cannot read current arm pose, aborting")
            break
        out = controller.infer_step(current_8d)
        if out is None:
            _log(f"WARNING: obs build failed at tick {tick}, skipping")
            with env_lock:
                env.step(action=hold_action)
            continue
        act7, target_8d = out

        if tick == 0 or tick % 10 == 0:
            pre_cube_z = read_cube_z()
            if pre_cube_z is None or initial_cube_z is None:
                dz_str = "n/a"
            else:
                dz_str = f"{pre_cube_z - initial_cube_z:+.4f}"
            _log(
                f"tick={tick:03d} tgt_j1..6={np.round(target_8d[:6], 3).tolist()} "
                f"j7={target_8d[6]:.4f} j8={target_8d[7]:.4f} cube_z={pre_cube_z} Δz={dz_str}"
            )

        if dump_root and (tick % max(1, int(dump_every)) == 0):
            try:
                tick_dir = _os.path.join(dump_root, f"tick_{tick:03d}")
                _os.makedirs(tick_dir, exist_ok=True)
                for cam_name, rgb in controller.last_images.items():
                    out_png = _os.path.join(tick_dir, f"{cam_name}.png")
                    if dump_image_mod is not None:
                        dump_image_mod.fromarray(rgb).save(out_png)
                    else:
                        np.save(out_png.replace(".png", ".npy"), rgb)
                cur_cube_z = read_cube_z()
                meta = {
                    "tick": tick,
                    "observation.state": (
                        controller.last_state.tolist()
                        if controller.last_state is not None
                        else None
                    ),
                    "raw_action_7d": [float(x) for x in act7],
                    "target_8d": [float(x) for x in target_8d],
                    "current_8d_before": [float(x) for x in current_8d],
                    "cube_z": cur_cube_z,
                    "initial_cube_z": initial_cube_z,
                }
                with open(_os.path.join(tick_dir, "meta.json"), "w") as f:
                    _json.dump(meta, f, indent=2)
            except Exception as exc:
                _log(f"WARNING: dump failed at tick {tick}: {exc}")

        action_dict = {
            nav_action_name: [hold_xy_action],
            arm_action_name: [target_8d.tolist()],
        }
        for _ in range(sim_steps_per_action):
            with env_lock:
                _, _, terminated, _, _ = env.step(action=action_dict)
            ep = terminated[0] if isinstance(terminated, (list, tuple)) else bool(terminated)
            if ep:
                terminate_flag = True
                break
        if terminate_flag:
            _log(f"episode terminated at tick {tick}")
            break

        cube_z = read_cube_z()
        final_cube_z = cube_z
        if (
            initial_cube_z is not None
            and cube_z is not None
            and cube_z - initial_cube_z > lift_threshold
        ):
            _log(
                f"SUCCESS: cube lifted at tick {tick}, z={cube_z:.3f} "
                f"(Δ={cube_z - initial_cube_z:+.3f})"
            )
            success = True
            ticks_used = tick + 1
            break
        ticks_used = tick + 1
    else:
        _log(f"reached max_ticks={max_ticks} without lift")

    # Force-close the gripper so Step C (nav home) doesn't drop the cube.
    final_arm_8d = read_arm8()
    if final_arm_8d is not None and not terminate_flag:
        closed_8d = final_arm_8d.copy()
        for _ in range(max(0, close_gripper_ramp_ticks)):
            cur = read_arm8()
            if cur is None:
                break
            nxt = cur.copy()
            nxt[6] = max(0.0, float(cur[6]) - float(max_per_tick_delta_gripper))
            nxt[7] = min(0.0, float(cur[7]) + float(max_per_tick_delta_gripper))
            action_dict = {
                nav_action_name: [hold_xy_action],
                arm_action_name: [nxt.tolist()],
            }
            for _ in range(sim_steps_per_action):
                with env_lock:
                    _, _, terminated, _, _ = env.step(action=action_dict)
                ep = terminated[0] if isinstance(terminated, (list, tuple)) else bool(terminated)
                if ep:
                    terminate_flag = True
                    break
            if terminate_flag:
                break
            closed_8d = nxt
            if abs(float(nxt[6])) < 1e-4 and abs(float(nxt[7])) < 1e-4:
                break
        closed_8d[6] = 0.0
        closed_8d[7] = 0.0
        action_dict = {
            nav_action_name: [hold_xy_action],
            arm_action_name: [closed_8d.tolist()],
        }
        for _ in range(max(0, close_gripper_hold_sim_steps)):
            with env_lock:
                _, _, terminated, _, _ = env.step(action=action_dict)
            ep = terminated[0] if isinstance(terminated, (list, tuple)) else bool(terminated)
            if ep:
                terminate_flag = True
                break
        final_arm_8d = closed_8d
        final_cube_z = read_cube_z()
        _log(
            f"forced gripper closed j7={final_arm_8d[6]:.4f} "
            f"j8={final_arm_8d[7]:.4f} cube_z={final_cube_z}"
        )

    return {
        "success": success,
        "ticks_used": ticks_used,
        "terminate": terminate_flag,
        "initial_cube_z": initial_cube_z,
        "final_cube_z": final_cube_z,
        "final_arm_8d": None if final_arm_8d is None else final_arm_8d.tolist(),
    }
