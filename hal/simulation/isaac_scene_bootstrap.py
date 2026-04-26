"""Best-effort Isaac scene bootstrap helpers used by simulation drivers."""

from __future__ import annotations

from typing import Any


def bootstrap_isaac_scene(
    api: Any,
    *,
    robot_xy: tuple[float, float],
    robot_z: float,
    lighting_mode: str = "none",
    camera_eye_offset: tuple[float, float, float] = (-2.8, -2.2, 1.8),
    camera_target_z_offset: float = -0.4,
    camera_target_min_z: float = 0.2,
    apply_lighting: bool = True,
    focus_camera: bool = False,
) -> list[str]:
    """Apply optional post-start lighting/camera tweaks and return step markers."""
    steps: list[str] = []

    if apply_lighting:
        try:
            if _apply_room_lighting(api, str(lighting_mode)):
                steps.append(f"lighting:{str(lighting_mode).strip().lower()}")
        except Exception as exc:
            steps.append(f"lighting_skipped:{exc}")

    if focus_camera:
        try:
            _focus_view_xy(
                robot_xy=robot_xy,
                robot_z=float(robot_z),
                camera_eye_offset=camera_eye_offset,
                camera_target_z_offset=float(camera_target_z_offset),
                camera_target_min_z=float(camera_target_min_z),
            )
            steps.append("viewport_focus")
        except Exception as exc:
            steps.append(f"viewport_focus_skipped:{exc}")

    return steps


def _focus_view_xy(
    *,
    robot_xy: tuple[float, float],
    robot_z: float,
    camera_eye_offset: tuple[float, float, float],
    camera_target_z_offset: float,
    camera_target_min_z: float,
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
    set_camera_view(eye=eye, target=target, camera_prim_path="/OmniverseKit_Persp")


def _apply_room_lighting(api: Any, lighting_mode: str) -> bool:
    lighting = str(lighting_mode or "").strip().lower().replace("-", "_").replace(" ", "_")
    if lighting not in {"grey_studio", "gray_studio"}:
        return False

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
    except Exception:
        # Some Isaac builds expose no get_settings() on IApp.
        # Continue and try dome-light fallback below.
        pass

    stage = None
    try:
        stage = api._env.runner._world.stage if api is not None else None  # type: ignore[attr-defined]
    except Exception:
        stage = None
    if stage is not None:
        try:
            for path in ("/Environment/DomeLight", "/World/DomeLight"):
                prim = stage.GetPrimAtPath(path)
                if prim and prim.IsValid():
                    color_attr = prim.GetAttribute("inputs:color")
                    if color_attr:
                        color_attr.Set((0.72, 0.74, 0.78))
                    intensity_attr = prim.GetAttribute("inputs:intensity")
                    if intensity_attr:
                        intensity_attr.Set(1800.0)
                    return True
        except Exception:
            pass

    return _ensure_default_dome_light(api)


def _ensure_default_dome_light(api: Any) -> bool:
    try:
        from pxr import Sdf, UsdLux
    except Exception:
        return False

    stage = None
    try:
        stage = api._env.runner._world.stage if api is not None else None  # type: ignore[attr-defined]
    except Exception:
        stage = None
    if stage is None:
        return False

    try:
        target_path = None
        for path in ("/Environment/DomeLight", "/World/DomeLight", "/World/PAOS_DefaultDomeLight"):
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsValid():
                target_path = path
                break
        if target_path is None:
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

