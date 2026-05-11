import os
from collections import OrderedDict
from typing import Optional

import numpy as np

from internutopia.core.robot.articulation import IArticulation
from internutopia.core.robot.articulation_action import ArticulationAction
from internutopia.core.robot.articulation_subset import ArticulationSubset
from internutopia.core.robot.rigid_body import IRigidBody
from internutopia.core.robot.robot import BaseRobot
from internutopia.core.scene.scene import IScene
from internutopia.core.util import log
from internutopia.core.util.physics import remove_collider, set_collider
from internutopia_extension.configs.robots.xlerobot import (
    XLEROBOT_ARM_JOINT_NAMES,
    XLEROBOT_BASE_JOINT_NAMES,
    XLEROBOT_EXPECTED_DOF_NAMES,
    XLEROBOT_GRIPPER_JOINT_NAMES,
    XLEROBOT_HEAD_JOINT_NAMES,
    XlerobotRobotCfg,
)

# 与 xlerobot_isaaclab `JointVelocityActionCfg` 一致：平面全向速度控制，不可施加大位置刚度
_BASE_VELOCITY_JOINTS = XLEROBOT_BASE_JOINT_NAMES


def _xle_np44_from_pos_quat(pos: np.ndarray, quat_wxyz: np.ndarray) -> np.ndarray:
    """列向量约定 p' = R @ p + t，四元数 wxyz。"""
    w, x, y, z = [float(v) for v in np.asarray(quat_wxyz, dtype=float).reshape(-1)[:4]]
    norm = (w * w + x * x + y * y + z * z) ** 0.5
    if norm < 1e-12:
        R = np.eye(3, dtype=float)
    else:
        w, x, y, z = w / norm, x / norm, y / norm, z / norm
        R = np.array(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
            ],
            dtype=float,
        )
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(pos, dtype=float).reshape(-1)[:3]
    return T


def _xle_np44_to_gf44(T: np.ndarray):
    """numpy 4x4 -> Gf.Matrix4d（与 PhysX/UsdGeom 列向量约定一致）。"""
    from pxr import Gf

    T = np.asarray(T, dtype=float).reshape(4, 4)
    return Gf.Matrix4d(
        float(T[0, 0]),
        float(T[0, 1]),
        float(T[0, 2]),
        float(T[0, 3]),
        float(T[1, 0]),
        float(T[1, 1]),
        float(T[1, 2]),
        float(T[1, 3]),
        float(T[2, 0]),
        float(T[2, 1]),
        float(T[2, 2]),
        float(T[2, 3]),
        float(T[3, 0]),
        float(T[3, 1]),
        float(T[3, 2]),
        float(T[3, 3]),
    )


@BaseRobot.register("XlerobotRobot")
class XlerobotRobot(BaseRobot):
    """XLeRobot dual-arm mobile manipulator loaded from the project USD (Isaac Lab asset)."""

    EEF_CENTER_OFFSET = np.array([0.0, 0.0, 0.06], dtype=float)

    @staticmethod
    def _expand_gain_values(values, count: int, fill_value: float) -> np.ndarray:
        if count <= 0:
            return np.zeros(0, dtype=float)
        if values is None:
            return np.full(count, fill_value, dtype=float)
        arr = np.asarray(values, dtype=float).reshape(-1)
        if arr.size == 0:
            return np.full(count, fill_value, dtype=float)
        if arr.size == 1:
            return np.full(count, float(arr[0]), dtype=float)
        if arr.size < count:
            arr = np.pad(arr, (0, count - arr.size), constant_values=float(arr[-1]))
        return arr[:count].astype(float, copy=False)

    def __init__(self, config: XlerobotRobotCfg, scene: IScene):
        super().__init__(config, scene)
        self._start_position = np.array(config.position) if config.position is not None else None
        self._start_orientation = np.array(config.orientation) if config.orientation is not None else None

        log.debug(f"xlerobot {config.name}: position    : {self._start_position}")
        log.debug(f"xlerobot {config.name}: orientation : {self._start_orientation}")
        log.debug(f"xlerobot {config.name}: usd_path         : {config.usd_path}")
        log.debug(f"xlerobot {config.name}: config.prim_path : {config.prim_path}")

        self._robot_scale = np.array(config.scale) if config.scale is not None else np.array([1.0, 1.0, 1.0])
        self.articulation = IArticulation.create(
            prim_path=config.prim_path,
            name=config.name,
            position=self._start_position,
            orientation=self._start_orientation,
            usd_path=config.usd_path,
            scale=self._robot_scale,
        )
        # 优先使用资产源头碰撞配置；运行时碰撞重写在该资产上会放大不稳定风险。
        # self._patch_collision_meshes()
        self._patch_base_joint_drives()
        self._manip_hold_q = None
        self._manip_joint_indices = None
        self._base_joint_indices = None
        self._holonomic_controller_name = None
        self._enable_manipulator_pose_hold = bool(getattr(config, "enable_manipulator_pose_hold", True))
        self._odom_joint_indices: dict[str, int] = {}
        self._odom_origin_xy = None
        self._enable_visual_shell_follow_root = os.getenv("XLE_VISUAL_FOLLOW_ROOT", "1") == "1"
        self._base_joint_body_targets_patched = False
        self._base_joint_body_patch_attempts = 0

    def _compose_robot_link_path(self, link_tail: str) -> str:
        """``articulation`` 根 prim 下的直接子路径（无嵌套引用层时与 map 键一致）。"""
        prim = getattr(self.articulation, "prim", None)
        if prim is not None and prim.IsValid():
            base = str(prim.GetPath()).rstrip("/")
        else:
            base = str(getattr(self.config, "prim_path", "") or "").rstrip("/") or "/xlerobot"
        tail = str(link_tail).lstrip("/")
        return f"{base}/{tail}"

    def _resolve_xlerobot_asset_root_prefix(self) -> Optional[str]:
        """USD 里 ``joints/`` 与 ``root``、``base_link`` 同属一层 payload 根。

        Isaac Sim ``SingleArticulation.prim`` 常指向 PhysX articulation root link（例如 ``.../root``），
        若仍用该路径拼接 ``/joints/...`` 会 miss，表现为 ``tried=0``、底盘 drive 补丁不生效、
        视觉补偿写到 ``root`` 子树也无法带动 ``base_link``/手臂等同层刚体。

        返回：其下存在 ``joints/root_x_axis_joint`` 的 stage 路径前缀；找不到则 ``None``。
        """
        ap = getattr(self.articulation, "prim", None)
        if ap is None or not ap.IsValid():
            return None
        stage = ap.GetStage()
        probe = "/joints/root_x_axis_joint"
        cur = ap
        for _ in range(12):
            if not cur.IsValid():
                break
            base = str(cur.GetPath()).rstrip("/")
            if stage.GetPrimAtPath(base + probe).IsValid():
                return base
            parent = cur.GetParent()
            if not parent.IsValid() or str(parent.GetPath()) in ("/", ""):
                break
            cur = parent
        cfg = str(getattr(self.config, "prim_path", "") or "").rstrip("/")
        if cfg and stage.GetPrimAtPath(cfg + probe).IsValid():
            return cfg
        return None

    @staticmethod
    def _rigid_body_stage_path(rb: IRigidBody) -> str:
        u = rb.unwrap()
        for attr in ("prim_path", "name"):
            v = getattr(u, attr, None)
            if isinstance(v, str) and v:
                return v
        n = getattr(rb, "name", None)
        return str(n) if n else ""

    def _get_link_rigid_body(self, link_tail: str) -> Optional[IRigidBody]:
        """在 ``_rigid_body_map`` 中解析 link 刚体。

        ``BaseRobot.create_rigid_bodies`` 在「articulation 根同时挂 ArticulationRoot + RigidBody」时
        会从父 prim 子树枚举刚体，实际路径常为 ``.../xlerobot/xlerobot/root``，而
        ``articulation.prim`` 仍在外层 ``.../xlerobot``；仅靠字符串拼接会 Key miss。
        """
        t = str(link_tail).lstrip("/").rstrip("/")
        if not t or not self._rigid_body_map:
            return None
        direct = self._compose_robot_link_path(t)
        hit = self._rigid_body_map.get(direct)
        if hit is not None:
            return hit
        cfg_base = str(getattr(self.config, "prim_path", "") or "").rstrip("/")
        if cfg_base:
            hit = self._rigid_body_map.get(f"{cfg_base}/{t}")
            if hit is not None:
                return hit
        suffix = "/" + t
        matches = [k for k in self._rigid_body_map if k == t or k.endswith(suffix)]
        if not matches:
            return None
        ap = ""
        prim = getattr(self.articulation, "prim", None)
        if prim is not None and prim.IsValid():
            ap = str(prim.GetPath()).rstrip("/")
        if ap:
            pref = ap + "/"
            under = [k for k in matches if k.startswith(pref)]
            if under:

                def _depth_key(path: str) -> tuple[int, int]:
                    rel = path[len(pref) :] if path.startswith(pref) else path
                    return (len(rel.split("/")) if rel else 0, len(path))

                under.sort(key=_depth_key)
                return self._rigid_body_map[under[0]]
        if len(matches) == 1:
            return self._rigid_body_map[matches[0]]
        matches.sort(key=len)
        return self._rigid_body_map[matches[0]]

    def _usd_link_path(self, link_tail: str) -> str:
        """与 ``_rigid_body_map`` / stage 一致的绝对 prim 路径。"""
        rb = self._get_link_rigid_body(link_tail)
        if rb is not None:
            p = self._rigid_body_stage_path(rb)
            if p:
                return p
        return self._compose_robot_link_path(link_tail)

    def _patch_collision_meshes(self) -> None:
        """Patch XLeRobot collision meshes conservatively.

        NOTE:
        A global ``/visuals`` collider strip is too aggressive for this asset. Some links
        rely on inherited collision/mass authoring and removing those APIs can lead to
        negative-mass fallback warnings (invalid inertia), then solver blow-ups once base
        motion starts. We only patch ``base_link`` subtrees here.
        """
        try:
            from pxr import PhysxSchema, Usd, UsdPhysics
        except Exception as exc:
            log.warning(f"xlerobot: unable to import pxr for collision patch: {exc}")
            return

        root_prim = getattr(self.articulation, "prim", None)
        if root_prim is None or not root_prim.IsValid():
            log.warning("xlerobot: articulation prim invalid, skip collision patch")
            return

        visuals_roots_patched = 0
        collisions_roots_patched = 0
        visuals_meshes_patched = 0
        collisions_meshes_patched = 0

        def _remove_collision_apis(prim) -> None:
            for api in (
                UsdPhysics.CollisionAPI,
                UsdPhysics.MeshCollisionAPI,
                PhysxSchema.PhysxCollisionAPI,
            ):
                try:
                    prim.RemoveAPI(api)
                except Exception:
                    pass

        def _mark_collision_subtree(prim, approx) -> None:
            try:
                if UsdPhysics.CollisionAPI.CanApply(prim):
                    collider = UsdPhysics.CollisionAPI.Apply(prim)
                    collider.GetCollisionEnabledAttr().Set(True)
            except Exception:
                pass
            try:
                if UsdPhysics.MeshCollisionAPI.CanApply(prim):
                    mesh_collider = UsdPhysics.MeshCollisionAPI.Apply(prim)
                    mesh_collider.GetApproximationAttr().Set(approx)
            except Exception:
                pass
            try:
                physx = PhysxSchema.PhysxCollisionAPI.Apply(prim)
                if hasattr(physx, "CreateApproximationAttr"):
                    physx.CreateApproximationAttr().Set(
                        "convexDecomposition"
                        if approx == UsdPhysics.Tokens.convexDecomposition
                        else "convexHull"
                    )
            except Exception:
                pass

        base_prefix = f"{root_prim.GetPath().pathString}/base_link"
        for prim in Usd.PrimRange(root_prim):
            prim_path = prim.GetPath().pathString
            if not prim_path.startswith(base_prefix):
                continue
            try:
                if prim_path.endswith("/visuals") or "/visuals/" in prim_path:
                    if prim_path.endswith("/visuals"):
                        _remove_collision_apis(prim)
                        remove_collider(prim)
                        visuals_roots_patched += 1
                    elif prim.GetTypeName() == "Mesh":
                        _remove_collision_apis(prim)
                        visuals_meshes_patched += 1
                    continue
                if not (prim_path.endswith("/collisions") or "/collisions/" in prim_path):
                    continue

                approx = (
                    UsdPhysics.Tokens.convexDecomposition
                    if "/base_link/collisions" in prim_path
                    else UsdPhysics.Tokens.convexHull
                )
                if prim_path.endswith("/collisions"):
                    _mark_collision_subtree(prim, approx)
                    set_collider(prim, approx=approx, init_state=True)
                    collisions_roots_patched += 1
                elif prim.GetTypeName() == "Mesh":
                    _mark_collision_subtree(prim, approx)
                    collisions_meshes_patched += 1
            except Exception as exc:
                log.warning(f"xlerobot: collision patch failed for {prim_path}: {exc}")

        if visuals_roots_patched or collisions_roots_patched or visuals_meshes_patched or collisions_meshes_patched:
            log.info(
                "xlerobot: collision patch applied "
                f"(visual_roots={visuals_roots_patched}, collision_roots={collisions_roots_patched}, "
                f"visual_meshes={visuals_meshes_patched}, collision_meshes={collisions_meshes_patched})"
            )

    def _patch_base_joint_drives(self) -> None:
        """Force base drive to velocity-friendly settings.

        XLeRobot base joints are commanded by velocity (root_x/root_y/root_z). If USD drive
        stiffness is left very high (position spring to targetPosition=0), it fights velocity
        command and can inject huge impulses once the base starts translating/rotating.
        """
        try:
            from pxr import Usd, UsdPhysics
        except Exception as exc:
            log.warning(f"xlerobot: unable to import pxr for base drive patch: {exc}")
            return

        root_prim = getattr(self.articulation, "prim", None)
        if root_prim is None or not root_prim.IsValid():
            return

        stage = root_prim.GetStage()
        asset_root = self._resolve_xlerobot_asset_root_prefix()
        if not asset_root:
            log.warning(
                "xlerobot: cannot resolve asset root (no joints/root_x_axis_joint under articulation.prim "
                f"ancestors); drive patch skipped. articulation.prim={root_prim.GetPath().pathString}"
            )
            return
        if asset_root != str(root_prim.GetPath()).rstrip("/"):
            log.info(
                "xlerobot: base drive patch uses asset_root=%s (articulation.prim=%s)",
                asset_root,
                root_prim.GetPath().pathString,
            )
        patched = 0
        for joint_name, drive_type, damping in (
            ("root_x_axis_joint", "linear", 100.0),
            ("root_y_axis_joint", "linear", 100.0),
            ("root_z_rotation_joint", "angular", 48.0),
        ):
            prim = stage.GetPrimAtPath(f"{asset_root}/joints/{joint_name}")
            if not prim.IsValid():
                continue
            try:
                drive = UsdPhysics.DriveAPI.Get(prim, drive_type)
                if not drive or not drive.GetPrim().IsValid():
                    continue
                drive.GetStiffnessAttr().Set(0.0)
                # Keep damping finite for velocity tracking, but avoid huge values.
                drive.GetDampingAttr().Set(float(damping))
                patched += 1
            except Exception as exc:
                log.warning(f"xlerobot: base drive patch failed for {joint_name}: {exc}")

        if patched > 0:
            log.info(f"xlerobot: base drive patch applied on {patched} joints")

    def _patch_base_joint_body_targets(self) -> bool:
        """可选实验：交换底盘三关节 body0/body1，验证资产移动链方向。"""
        if os.getenv("XLE_SWAP_BASE_JOINT_BODIES", "0") != "1":
            return True
        try:
            from pxr import Usd
        except Exception as exc:
            log.warning(f"xlerobot: unable to import pxr.Usd for base joint body swap: {exc}")
            return False
        root_prim = getattr(self.articulation, "prim", None)
        if root_prim is None or not root_prim.IsValid():
            return False
        stage = root_prim.GetStage()
        swapped = 0
        tried = 0
        root_prefix = self._resolve_xlerobot_asset_root_prefix()
        if not root_prefix:
            log.warning(
                "xlerobot: base joint body swap skipped — cannot resolve asset root "
                f"(articulation.prim={root_prim.GetPath().pathString})"
            )
            return True

        def _resolve_joint_prim(joint_name: str):
            p = stage.GetPrimAtPath(f"{root_prefix}/joints/{joint_name}")
            if p.IsValid():
                return p
            suffix = f"/joints/{joint_name}"
            for prim in Usd.PrimRange.AllPrims(stage.GetPseudoRoot()):
                if not prim.IsValid():
                    continue
                pp = prim.GetPath().pathString
                if not pp.startswith(root_prefix):
                    continue
                if pp.endswith(suffix):
                    return prim
            return None

        for joint_name in ("root_x_axis_joint", "root_y_axis_joint", "root_z_rotation_joint"):
            prim = _resolve_joint_prim(joint_name)
            if prim is None or not prim.IsValid():
                continue
            tried += 1
            try:
                body0_rel = prim.GetRelationship("physics:body0")
                body1_rel = prim.GetRelationship("physics:body1")
                if not body0_rel.IsValid() or not body1_rel.IsValid():
                    continue
                body0 = list(body0_rel.GetTargets() or [])
                body1 = list(body1_rel.GetTargets() or [])
                if len(body0) != 1 or len(body1) != 1 or body0[0] == body1[0]:
                    continue
                body0_rel.SetTargets(body1)
                body1_rel.SetTargets(body0)
                swapped += 1
                log.warning(
                    "xlerobot: swapped %s body0=%s body1=%s on prim=%s",
                    joint_name,
                    body0[0],
                    body1[0],
                    prim.GetPath().pathString,
                )
            except Exception as exc:
                log.warning(f"xlerobot: base joint body swap failed for {joint_name}: {exc}")
        if swapped:
            log.warning(
                "xlerobot: swapped body targets on %s base joints "
                "(XLE_SWAP_BASE_JOINT_BODIES=1 experimental)",
                swapped,
            )
            return True
        log.warning(
            "xlerobot: base-joint body swap enabled but no joints swapped "
            "(tried=%s, asset_root=%s). Topology likely already correct; not retrying.",
            tried,
            root_prefix,
        )
        return True

    def post_reset(self):
        super().post_reset()
        if not self._base_joint_body_targets_patched:
            self._base_joint_body_targets_patched = self._patch_base_joint_body_targets()
        self._robot_base = self._resolve_robot_base()
        self._arm_base = self._resolve_arm_base()
        self._end_effector = self._resolve_end_effector()
        self.set_gains()
        self._cache_manipulator_hold_and_base_indices()

    def _resolve_robot_base(self) -> IRigidBody:
        preferred_names = []
        if getattr(self.config, "base_link_name", None):
            preferred_names.append(self.config.base_link_name)
        # 默认 root：与全向关节驱动的 PhysX 根体一致；base_link 仅当 USD 已正确随底盘平移时使用。
        preferred_names.extend(["root", "base_link", "base", "trunk"])

        seen = set()
        for link_name in preferred_names:
            if link_name in seen:
                continue
            seen.add(link_name)
            rigid_body = self._get_link_rigid_body(link_name)
            if rigid_body is not None:
                log.debug(f"xlerobot {self.config.name}: using base link {link_name}")
                return rigid_body

        available_links = sorted(path.split("/")[-1] for path in self._rigid_body_map)
        raise KeyError(
            f"Cannot find base link for {self.config.name}. "
            f"Tried {preferred_names}. Available link names (last segment): {available_links}"
        )

    def get_robot_scale(self):
        return self._robot_scale

    def get_robot_base(self) -> IRigidBody:
        return self._robot_base

    def get_arm_base(self) -> IRigidBody:
        return self._arm_base

    def get_end_effector(self) -> IRigidBody:
        return self._end_effector

    def get_pose(self):
        pos, quat = self._robot_base.get_pose()
        pos = np.asarray(pos, dtype=float).reshape(-1)
        quat = np.asarray(quat, dtype=float).reshape(-1)

        # 某些 USD 结构下可见壳体 pose 不随 root_x/root_y 变化，导致“关节在动但位姿不动”。
        # 这里使用底盘关节位置构造 XY 里程计，保证控制闭环与位移判定可用。
        if (
            self._odom_origin_xy is not None
            and self._odom_joint_indices
            and self.articulation is not None
            and self.articulation.handles_initialized
        ):
            try:
                q = np.asarray(self.articulation.get_joint_positions(), dtype=float).reshape(-1)
                ix = self._odom_joint_indices.get("root_x_axis_joint")
                iy = self._odom_joint_indices.get("root_y_axis_joint")
                # XLe 资产中 root_x/root_y 关节与世界位移方向相反，故用 -q。
                if ix is not None and iy is not None and ix < q.size and iy < q.size:
                    pos = pos.copy()
                    pos[0] = float(self._odom_origin_xy[0] - q[ix])
                    pos[1] = float(self._odom_origin_xy[1] - q[iy])
            except Exception:
                pass

        return pos, quat

    def _resolve_end_effector(self) -> IRigidBody:
        for suffix in ("Fixed_Jaw", "Fixed_Jaw_2"):
            rigid_body = self._get_link_rigid_body(suffix)
            if rigid_body is not None:
                return rigid_body
        log.warning("xlerobot: no gripper rigid body found, falling back to base for eef pose")
        return self._robot_base

    def _resolve_arm_base(self) -> IRigidBody:
        for suffix in ("base_link", "torso_link", "base"):
            rigid_body = self._get_link_rigid_body(suffix)
            if rigid_body is not None:
                return rigid_body
        return self._robot_base

    def _cache_manipulator_hold_and_base_indices(self) -> None:
        """在 reset 后缓存上身/夹爪/头的关节角作为每步位置保持目标；并记录全向底盘控制器名。"""
        self._holonomic_controller_name = None
        ctrls = getattr(self.config, "controllers", None) or []
        for c in ctrls:
            if getattr(c, "type", None) == "HolonomicPlanarMoveToPointController":
                self._holonomic_controller_name = c.name
                break

        self._manip_joint_indices = None
        self._manip_hold_q = None
        self._base_joint_indices = None
        if not self.articulation.handles_initialized:
            return
        dof_names = list(self.articulation.dof_names or [])
        if not dof_names:
            return

        manip_names = [n for n in XLEROBOT_EXPECTED_DOF_NAMES if n in dof_names and n not in _BASE_VELOCITY_JOINTS]
        if manip_names:
            subset_m = ArticulationSubset(self.articulation, manip_names)
            ji = subset_m.joint_indices
            if ji is not None:
                self._manip_joint_indices = np.asarray(ji, dtype=np.int64)
                q_sub = subset_m.get_joint_positions()
                if q_sub is not None:
                    self._manip_hold_q = np.asarray(q_sub, dtype=np.float64).reshape(-1)

        base_names = [n for n in _BASE_VELOCITY_JOINTS if n in dof_names]
        if base_names:
            subset_b = ArticulationSubset(self.articulation, base_names)
            if subset_b.joint_indices is not None:
                self._base_joint_indices = np.asarray(subset_b.joint_indices, dtype=np.int64)

        self._odom_joint_indices = {}
        for jn in ("root_x_axis_joint", "root_y_axis_joint", "root_z_rotation_joint"):
            if jn in dof_names:
                self._odom_joint_indices[jn] = int(dof_names.index(jn))
        try:
            base_pos, _ = self._robot_base.get_pose()
            base_pos = np.asarray(base_pos, dtype=float).reshape(-1)
            if base_pos.size >= 2 and np.isfinite(base_pos[:2]).all():
                self._odom_origin_xy = np.array(base_pos[:2], dtype=float)
            elif self._start_position is not None:
                self._odom_origin_xy = np.array(self._start_position[:2], dtype=float)
            else:
                self._odom_origin_xy = np.zeros(2, dtype=float)
        except Exception:
            if self._start_position is not None:
                self._odom_origin_xy = np.array(self._start_position[:2], dtype=float)
            else:
                self._odom_origin_xy = np.zeros(2, dtype=float)

    def _apply_manipulator_pose_hold(self) -> None:
        """用 PD 目标角保持上身，避免每步 ``set_joint_positions`` 硬拧与 PhysX/底盘速度叠加导致关节速度爆炸（日志里 arm_|qd|_max 数百）。"""
        if self._manip_hold_q is None or self._manip_joint_indices is None:
            return
        if self._manip_hold_q.size != self._manip_joint_indices.size:
            return
        self.articulation.apply_action(
            ArticulationAction(
                joint_positions=np.asarray(self._manip_hold_q, dtype=np.float64).reshape(-1),
                joint_indices=np.asarray(self._manip_joint_indices, dtype=np.int64).reshape(-1),
            )
        )

    def _zero_base_velocities(self) -> None:
        if self._base_joint_indices is None or self._base_joint_indices.size == 0:
            return
        zeros = np.zeros(self._base_joint_indices.size, dtype=np.float64)
        self.articulation.set_joint_velocities(zeros, joint_indices=self._base_joint_indices)

    def _zero_manipulator_velocities(self) -> None:
        if self._manip_joint_indices is None or self._manip_joint_indices.size == 0:
            return
        zeros = np.zeros(self._manip_joint_indices.size, dtype=np.float64)
        self.articulation.set_joint_velocities(zeros, joint_indices=self._manip_joint_indices)

    @staticmethod
    def _xle_xform_prim_path_for_rb(stage, rb_path: str) -> Optional[str]:
        """刚体常在 ``.../link/visuals/mesh``；把 USD 写回不含 visuals/collisions 段的 link 根 prim。"""
        cur = stage.GetPrimAtPath(str(rb_path))
        while cur.IsValid():
            pl = str(cur.GetPath()).lower()
            if "/visuals" in pl or "/collisions" in pl:
                cur = cur.GetParent()
                continue
            return str(cur.GetPath())
        return None

    def _xle_set_prim_local_tr_orient_from_gf(self, stage, prim_path: str, T_local: "Gf.Matrix4d") -> None:
        """在 prim 上写 translate + orient（由局部 4x4 分解），与 UsdGeom 矩阵约定一致。"""
        from pxr import Gf, UsdGeom

        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            return
        xformable = UsdGeom.Xformable(prim)
        if not xformable:
            return
        m = T_local
        tr = m.ExtractTranslation()
        rot = m.ExtractRotation()
        gq = rot.GetQuat()
        w = float(gq.GetReal())
        imag = gq.GetImaginary()
        quat_wxyz = np.array([w, float(imag[0]), float(imag[1]), float(imag[2])], dtype=float)

        ops = list(xformable.GetOrderedXformOps())
        t_op = None
        r_op = None
        for op in ops:
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate and t_op is None:
                t_op = op
            elif op.GetOpType() == UsdGeom.XformOp.TypeOrient and r_op is None:
                r_op = op
        if t_op is None:
            t_op = xformable.AddTranslateOp()
        if r_op is None:
            r_op = xformable.AddOrientOp()
        t_op.Set(Gf.Vec3d(float(tr[0]), float(tr[1]), float(tr[2])))
        r_op.Set(Gf.Quatd(float(quat_wxyz[0]), Gf.Vec3d(float(quat_wxyz[1]), float(quat_wxyz[2]), float(quat_wxyz[3]))))

    def _sync_visual_shell_to_root(self) -> None:
        """在 PhysX 与 USD 仍短暂不一致时，把 ``root`` 与 ``base_link`` 的 XY 差累加到 **payload 根** 的 translate。

        payload 根由 `_resolve_xlerobot_asset_root_prefix` 解析（通常为引用根 ``.../xlerobot``），
        与 ``articulation.prim`` 常为 ``.../root`` 的情况区分：必须写外层 Xform，才能带动
        ``base_link``、手臂等同层刚体；仅写 ``root`` 子树不会移动兄弟 link。
        """
        if not self._enable_visual_shell_follow_root or not self._rigid_body_map:
            return
        root_body = self._get_link_rigid_body("root")
        base_body = self._get_link_rigid_body("base_link")
        if root_body is None or base_body is None:
            return
        try:
            root_pos = np.asarray(root_body.get_pose()[0], dtype=float).reshape(-1)
            base_pos = np.asarray(base_body.get_pose()[0], dtype=float).reshape(-1)
            if (
                root_pos.size < 3
                or base_pos.size < 3
                or not np.isfinite(root_pos).all()
                or not np.isfinite(base_pos).all()
            ):
                return
            delta_xy = root_pos[:2] - base_pos[:2]
            if float(np.linalg.norm(delta_xy)) < 1e-4:
                return
        except Exception:
            return

        stage = getattr(self.articulation, "stage", None)
        prim0 = getattr(self.articulation, "prim", None)
        if stage is None and prim0 is not None:
            stage = prim0.GetStage()
        if stage is None or prim0 is None or not prim0.IsValid():
            return
        asset_root = self._resolve_xlerobot_asset_root_prefix()
        if not asset_root:
            return
        shell_prim = stage.GetPrimAtPath(asset_root)
        if not shell_prim.IsValid():
            return
        try:
            from pxr import Gf, UsdGeom
            root_xf = UsdGeom.Xformable(shell_prim)
            if not root_xf:
                return
            ops = list(root_xf.GetOrderedXformOps())
            t_op = None
            for op in ops:
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    t_op = op
                    break
            if t_op is None:
                t_op = root_xf.AddTranslateOp()
            t = t_op.Get()
            if t is None:
                t = Gf.Vec3d(0.0, 0.0, 0.0)
            t_op.Set(Gf.Vec3d(float(t[0]) + float(delta_xy[0]), float(t[1]) + float(delta_xy[1]), float(t[2])))
        except Exception:
            return

    def post_physics_step(self) -> None:
        """在每步 ``World.step``（PhysX 积分）之后调用，用于修正被仿真覆盖的 USD/壳体位姿。"""
        if not self._base_joint_body_targets_patched and os.getenv("XLE_SWAP_BASE_JOINT_BODIES", "0") == "1":
            self._base_joint_body_patch_attempts += 1
            # 资产引用有时在 reset 后数帧才完全可见；延迟重试避免永远 tried=0。
            if self._base_joint_body_patch_attempts <= 80:
                self._base_joint_body_targets_patched = self._patch_base_joint_body_targets()
        self._sync_visual_shell_to_root()
        try:
            from omni.physx import get_physx_fabric_interface

            iface = get_physx_fabric_interface()
            fn = getattr(iface, "force_update", None) or getattr(iface, "update", None)
            if callable(fn):
                fn()
        except Exception:
            pass

    def apply_action(self, action: dict):
        if not self.articulation.handles_initialized:
            return
        has_base_cmd = bool(self._holonomic_controller_name and self._holonomic_controller_name in action)
        if has_base_cmd:
            # 导航时先抑制上身关节速度，避免臂关节甩动把底盘带翻。
            self._zero_manipulator_velocities()
            # 同时保持上身初始姿态，避免“自由摆锤”在长距离行走中累积姿态能量。
            if self._enable_manipulator_pose_hold:
                self._apply_manipulator_pose_hold()
        elif self._enable_manipulator_pose_hold:
            self._apply_manipulator_pose_hold()
        for controller_name, controller_action in action.items():
            if controller_name not in self.controllers:
                log.warning(f"unknown controller {controller_name} in action")
                continue
            controller = self.controllers[controller_name]
            control = controller.action_to_control(controller_action)
            self._apply_articulation_control(control)
        if self._holonomic_controller_name and self._holonomic_controller_name not in action:
            self._zero_base_velocities()

    def _apply_articulation_control(self, control) -> None:
        """下发控制到 PhysX。

        Isaac Sim 的 ``SingleArticulation.apply_action`` 对带 ``joint_indices`` 的
        ``ArticulationAction`` 往往不按子集速度生效，底盘会「完全不动」；而若与
        ``set_joint_velocities`` 同一帧各写一遍又会冲突导致炸飞。

        对「仅关节速度 + 显式 joint_indices」走单独的 ``set_joint_velocities``；
        其余情况仍走 ``apply_action``。
        """
        ji = getattr(control, "joint_indices", None)
        jv = getattr(control, "joint_velocities", None)
        jp = getattr(control, "joint_positions", None)
        je = getattr(control, "joint_efforts", None)

        if ji is not None and jv is not None and jp is None and je is None:
            jv_arr = np.asarray(jv, dtype=np.float64).reshape(-1)
            ji_arr = np.asarray(ji, dtype=np.int64).reshape(-1)
            if jv_arr.size == ji_arr.size and jv_arr.size > 0:
                self.articulation.set_joint_velocities(jv_arr, joint_indices=ji_arr)
                return

        self.articulation.apply_action(control)

    def get_obs(self) -> OrderedDict:
        position, orientation = self.get_pose()
        arm_base_position, arm_base_orientation = self._arm_base.get_pose()
        eef_base_position, eef_orientation = self._end_effector.get_pose()
        raw_base_position = None
        raw_root_position = None
        try:
            raw_base_body = self._get_link_rigid_body("base_link")
            if raw_base_body is not None:
                raw_base_position = np.asarray(raw_base_body.get_pose()[0], dtype=float)
        except Exception:
            raw_base_position = None
        try:
            raw_root_body = self._get_link_rigid_body("root")
            if raw_root_body is not None:
                raw_root_position = np.asarray(raw_root_body.get_pose()[0], dtype=float)
        except Exception:
            raw_root_position = None
        joint_positions = self.articulation.get_joint_positions()
        joint_velocities = self.articulation.get_joint_velocities()
        try:
            joint_efforts = self.articulation.get_joint_efforts()
        except Exception:
            # Some Isaac Sim articulation backends do not implement joint efforts.
            joint_efforts = np.full_like(np.asarray(joint_velocities, dtype=np.float64), np.nan)
        eef_position = self._compute_eef_center(
            base_position=eef_base_position,
            base_orientation=eef_orientation,
        )
        obs = {
            "position": position,
            "orientation": orientation,
            "raw_base_position": raw_base_position,
            "raw_root_position": raw_root_position,
            "joint_positions": joint_positions,
            "joint_velocities": joint_velocities,
            "joint_efforts": joint_efforts,
            "arm_base_position": arm_base_position,
            "arm_base_orientation": arm_base_orientation,
            "eef_position": eef_position,
            "eef_orientation": eef_orientation,
            "controllers": {},
            "sensors": {},
        }

        for controller_name, controller in self.controllers.items():
            obs["controllers"][controller_name] = controller.get_obs()
        for sensor_name, sensor in self.sensors.items():
            obs["sensors"][sensor_name] = sensor.get_data()
        return self._make_ordered(obs)

    @classmethod
    def _compute_eef_center(
        cls,
        base_position: np.ndarray,
        base_orientation: np.ndarray,
    ) -> np.ndarray:
        rotation = cls._quat_to_rotmat(base_orientation)
        return np.array(base_position, dtype=float) + rotation @ cls.EEF_CENTER_OFFSET

    @staticmethod
    def _quat_to_rotmat(quat_wxyz: np.ndarray) -> np.ndarray:
        w, x, y, z = [float(v) for v in quat_wxyz]
        norm = np.sqrt(w * w + x * x + y * y + z * z)
        if norm < 1e-8:
            return np.eye(3, dtype=float)
        w /= norm
        x /= norm
        y /= norm
        z /= norm
        return np.array(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
            ],
            dtype=float,
        )

    def set_gains(self):
        dof_names = list(self.articulation.dof_names)
        if not dof_names:
            log.warning("xlerobot: articulation.dof_names empty, skip set_gains")
            return

        def _apply_group_gains(
            joint_names_attr: str,
            default_joint_names,
            kps_attr: str,
            kds_attr: str,
            kp_fill: float,
            kd_fill: float,
        ) -> None:
            configured_names = getattr(self.config, joint_names_attr, None)
            group_names = [n for n in (configured_names or default_joint_names) if n in dof_names]
            if not group_names:
                return
            subset = ArticulationSubset(self.articulation, group_names)
            kps = self._expand_gain_values(
                getattr(self.config, kps_attr, None),
                len(group_names),
                fill_value=kp_fill,
            )
            kds = self._expand_gain_values(
                getattr(self.config, kds_attr, None),
                len(group_names),
                fill_value=kd_fill,
            )
            self.articulation.set_gains(
                kps=kps,
                kds=kds,
                joint_indices=subset.joint_indices,
            )

        # 与全向底盘同帧时不宜过刚；按 IsaacLab 动作分组分别设置更接近原结构
        _apply_group_gains(
            "arm_joint_names",
            XLEROBOT_ARM_JOINT_NAMES,
            "arm_kps",
            "arm_kds",
            kp_fill=28.0,
            kd_fill=6.0,
        )
        _apply_group_gains(
            "gripper_joint_names",
            XLEROBOT_GRIPPER_JOINT_NAMES,
            "gripper_kps",
            "gripper_kds",
            kp_fill=28.0,
            kd_fill=6.0,
        )
        _apply_group_gains(
            "head_joint_names",
            XLEROBOT_HEAD_JOINT_NAMES,
            "head_kps",
            "head_kds",
            kp_fill=28.0,
            kd_fill=6.0,
        )
        # kp=0：允许 Holonomic / 键盘式 JointVelocity 驱动；kd 提高有利于速度跟踪与接地稳定
        _apply_group_gains(
            "base_joint_names",
            XLEROBOT_BASE_JOINT_NAMES,
            "base_kps",
            "base_kds",
            kp_fill=0.0,
            kd_fill=22.0,
        )

        solver_pos_iters = getattr(self.config, "solver_position_iteration_count", 10)
        solver_vel_iters = getattr(self.config, "solver_velocity_iteration_count", 4)
        enable_self_collisions = getattr(self.config, "enable_self_collisions", False)

        self.articulation.set_solver_position_iteration_count(int(solver_pos_iters))
        # 速度迭代为 0 时接触/摩擦解算易飘、穿模；略增有利于「贴地」
        self.articulation.set_solver_velocity_iteration_count(int(solver_vel_iters))
        # 全向底盘行走 demo：默认关闭自碰，减轻双臂在保持姿态时的额外内力
        self.articulation.set_enabled_self_collisions(bool(enable_self_collisions))
