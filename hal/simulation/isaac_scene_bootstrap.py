"""Isaac / Omniverse helpers for HAL sim drivers (lighting + viewport).

Design goals
------------
- **Opt-in**: Callers choose a *lighting mode*; ``none`` means no scene mutation.
- **Conservative dome fill**: ``dome_fill`` only creates ``/World/PAOS_DefaultDomeLight``
  when the stage has **no** ``UsdLux.DomeLight`` yet (custom lit scenes are left alone).
- **Explicit RTX preset**: ``grey_studio`` applies InternUtopia / merom-style RTX rig
  settings; use only when you intend that look (e.g. demo ``merom_scene_baked.usd``).
- **Best-effort**: Functions must not raise into driver startup; failures become log tags.

This module is the single implementation; drivers (PiperGo2, Franka, …) should delegate
here instead of copying Omniverse snippets.
"""

from __future__ import annotations

from typing import Any, Sequence


def get_stage_from_api(api: Any) -> Any | None:
    """Return USD stage from an InternUtopia handle.

    Supports:
    - ``FrankaManipulationAPI`` / ``PiperGo2ManipulationAPI`` style objects with ``_env``.
    - ``internutopia.core.gym_env.Env`` (uses ``runner._world.stage``).
    """
    try:
        inner = getattr(api, "_env", None)
        if inner is not None:
            return inner.runner._world.stage
    except Exception:
        pass
    try:
        return api.runner._world.stage
    except Exception:
        return None


def stage_has_any_dome_light(stage: Any) -> bool:
    """True if any prim on the stage is a UsdLux.DomeLight (artist- or asset-lit)."""
    if stage is None:
        return False
    try:
        from pxr import Usd, UsdLux

        for prim in Usd.PrimRange(stage.GetPseudoRoot()):
            if prim.IsA(UsdLux.DomeLight):
                return True
    except Exception:
        pass
    return False


def _apply_rtx_grey_studio_settings() -> bool:
    try:
        from omni.kit.app import get_app  # type: ignore

        settings = get_app().get_settings()
        for key, value in (
            ("/rtx/environment/visible", True),
            ("/rtx/environment/mode", "Studio"),
            ("/rtx/environment/lightRig", "Grey Studio"),
            ("/rtx/environment/lightRigName", "Grey Studio"),
            ("/rtx/sceneDb/ambientLightIntensity", 0.35),
        ):
            try:
                settings.set(key, value)
            except Exception:
                pass
        return True
    except Exception:
        return False


def _brighten_named_dome_paths(stage: Any) -> None:
    """Tune known merom-style dome prim paths if they exist (does not create prims)."""
    try:
        for path in ("/Environment/DomeLight", "/World/DomeLight"):
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsValid():
                attr = prim.GetAttribute("inputs:color")
                if attr:
                    attr.Set((0.72, 0.74, 0.78))
                iattr = prim.GetAttribute("inputs:intensity")
                if iattr:
                    iattr.Set(1800.0)
                break
    except Exception:
        pass


def ensure_paos_fallback_dome(stage: Any) -> bool:
    """If the stage has no dome light, add ``/World/PAOS_DefaultDomeLight``.

    If any ``UsdLux.DomeLight`` already exists (any path), do nothing and return True.
    """
    try:
        from pxr import Sdf, UsdLux

        if stage is None:
            return False
        if stage_has_any_dome_light(stage):
            return True

        target_path = "/World/PAOS_DefaultDomeLight"
        UsdLux.DomeLight.Define(stage, Sdf.Path(target_path))
        dome = UsdLux.DomeLight(stage.GetPrimAtPath(target_path))
        dome.CreateIntensityAttr(1800.0)
        dome.CreateExposureAttr(0.0)
        dome.CreateColorAttr((0.72, 0.74, 0.78))
        if not dome.GetTextureFileAttr().HasAuthoredValue():
            dome.CreateTextureFileAttr("")
        return True
    except Exception:
        return False


def apply_grey_studio_lighting(api: Any) -> bool:
    """RTX Grey Studio + optional merom dome tune + fallback dome only if missing."""
    stage = get_stage_from_api(api)
    ok = _apply_rtx_grey_studio_settings()
    if stage is not None:
        try:
            _brighten_named_dome_paths(stage)
        except Exception:
            pass
    dome_ok = ensure_paos_fallback_dome(stage)
    return bool(ok or dome_ok)


def normalize_lighting_mode(mode: str) -> str:
    return mode.replace("-", "_").replace(" ", "_").strip().lower()


def apply_lighting_for_mode(api: Any, mode: str) -> list[str]:
    """Return human-readable step tags for what was attempted (may be empty)."""
    key = normalize_lighting_mode(mode)
    if key in ("none", "off", ""):
        return []
    if key in ("grey_studio", "gray_studio"):
        ok = apply_grey_studio_lighting(api)
        return [("lighting:grey_studio" if ok else "lighting:grey_studio_partial")]
    if key in ("dome_fill", "dome_only", "dome"):
        stage = get_stage_from_api(api)
        if stage is not None and stage_has_any_dome_light(stage):
            return ["lighting:dome_fill_skip_scene_has_dome"]
        ok = ensure_paos_fallback_dome(stage)
        return [("lighting:dome_fill" if ok else "lighting:dome_fill_skipped")]
    return [f"lighting:unknown_mode:{mode}"]


def focus_viewport_on_robot(
    robot_xy: tuple[float, float],
    robot_z: float,
    *,
    camera_eye_offset: Sequence[float] = (-2.8, -2.2, 1.8),
    camera_target_z_offset: float = -0.4,
    camera_target_min_z: float = 0.2,
) -> None:
    try:
        from isaacsim.core.utils.viewports import set_camera_view
    except ImportError:
        try:
            from omni.isaac.core.utils.viewports import set_camera_view
        except ImportError:
            return
    eye = [
        float(robot_xy[0]) + float(camera_eye_offset[0]),
        float(robot_xy[1]) + float(camera_eye_offset[1]),
        float(robot_z) + float(camera_eye_offset[2]),
    ]
    target = [
        float(robot_xy[0]),
        float(robot_xy[1]),
        max(float(robot_z) + float(camera_target_z_offset), float(camera_target_min_z)),
    ]
    set_camera_view(
        eye=eye,
        target=target,
        camera_prim_path="/OmniverseKit_Persp",
    )


def bootstrap_isaac_scene(
    api: Any,
    *,
    robot_xy: tuple[float, float],
    robot_z: float,
    lighting_mode: str = "none",
    camera_eye_offset: Sequence[float] = (-2.8, -2.2, 1.8),
    camera_target_z_offset: float = -0.4,
    camera_target_min_z: float = 0.2,
    apply_lighting: bool = True,
    focus_camera: bool = False,
) -> list[str]:
    """Post-start viewport / lighting. **Lighting** runs only if ``apply_lighting`` is True."""
    steps: list[str] = []
    if apply_lighting:
        try:
            steps.extend(apply_lighting_for_mode(api, lighting_mode))
        except Exception as exc:
            steps.append(f"lighting_skipped:{exc}")

    if focus_camera:
        try:
            focus_viewport_on_robot(
                robot_xy,
                robot_z,
                camera_eye_offset=camera_eye_offset,
                camera_target_z_offset=camera_target_z_offset,
                camera_target_min_z=camera_target_min_z,
            )
            steps.append("viewport_focus")
        except Exception as exc:
            steps.append(f"viewport_focus_skipped:{exc}")

    return steps
