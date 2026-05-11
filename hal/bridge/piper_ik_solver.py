from __future__ import annotations

from pathlib import Path
import importlib.util
from typing import Sequence

import numpy as np

_IMPORT_ERROR: Exception | None = None

try:
    import casadi
    import pinocchio as pin
    from pinocchio import casadi as cpin
    from pinocchio.robot_wrapper import RobotWrapper
except ImportError as exc:
    casadi = None
    pin = None
    cpin = None
    RobotWrapper = None
    _IMPORT_ERROR = exc


def _default_piper_ros_src_dir() -> Path:
    """Prefer ``hal/piper_ros/src``; fall back to legacy ``<repo>/piper_ros/src``."""
    hal_root = Path(__file__).resolve().parents[1]
    bundled = hal_root / "piper_ros" / "src"
    if bundled.is_dir():
        return bundled
    legacy = hal_root.parent / "piper_ros" / "src"
    if legacy.is_dir():
        return legacy
    return bundled


def _normalize_quat_xyzw(quaternion_wxyz: Sequence[float]) -> np.ndarray:
    quat = np.array(
        [
            float(quaternion_wxyz[1]),
            float(quaternion_wxyz[2]),
            float(quaternion_wxyz[3]),
            float(quaternion_wxyz[0]),
        ],
        dtype=float,
    )
    norm = np.linalg.norm(quat)
    if norm < 1e-8:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return quat / norm


class PiperIKSolver:
    EEF_CENTER_OFFSET = np.array([0.0, 0.0, 0.1358], dtype=float)
    DEFAULT_PACKAGE_DIR = _default_piper_ros_src_dir()
    DEFAULT_URDF_PATH = DEFAULT_PACKAGE_DIR / "piper_description" / "urdf" / "piper_description.urdf"

    def __init__(self, urdf_path: str | None = None, package_dir: str | None = None) -> None:
        if not self.is_available():
            raise ImportError(self.describe_unavailable())

        self.urdf_path = str(Path(urdf_path or self.DEFAULT_URDF_PATH).resolve())
        self.package_dir = str(Path(package_dir or self.DEFAULT_PACKAGE_DIR).resolve())

        np.set_printoptions(precision=5, suppress=True, linewidth=200)

        self.robot = RobotWrapper.BuildFromURDF(
            self.urdf_path,
            package_dirs=[self.package_dir],
        )
        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=["joint7", "joint8"],
            reference_configuration=np.zeros(self.robot.model.nq, dtype=float),
        )

        # Use the gripper center defined by the finger joint origins.
        self.reduced_robot.model.addFrame(
            pin.Frame(
                "ee",
                self.reduced_robot.model.getJointId("joint6"),
                pin.SE3(np.eye(3), self.EEF_CENTER_OFFSET),
                pin.FrameType.OP_FRAME,
            )
        )
        self._frame_id = self.reduced_robot.model.getFrameId("ee")

        self.init_data = np.zeros(self.reduced_robot.model.nq, dtype=float)
        self.history_data = np.zeros(self.reduced_robot.model.nq, dtype=float)

        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()

        self.cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1)
        self.cTf = casadi.SX.sym("tf", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

        self.error = casadi.Function(
            "error",
            [self.cq, self.cTf],
            [
                casadi.vertcat(
                    cpin.log6(
                        self.cdata.oMf[self._frame_id].inverse() * cpin.SE3(self.cTf)
                    ).vector
                )
            ],
        )

        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.reduced_robot.model.nq)
        self.param_tf = self.opti.parameter(4, 4)

        error_vec = self.error(self.var_q, self.param_tf)
        pos_error = error_vec[:3]
        ori_error = error_vec[3:]

        weight_position = 1.0
        weight_orientation = 0.1

        total_cost = (
            casadi.sumsqr(weight_position * pos_error)
            + casadi.sumsqr(weight_orientation * ori_error)
        )
        regularization = casadi.sumsqr(self.var_q)

        self.opti.subject_to(
            self.opti.bounded(
                self.reduced_robot.model.lowerPositionLimit,
                self.var_q,
                self.reduced_robot.model.upperPositionLimit,
            )
        )
        self.opti.minimize(20.0 * total_cost + 0.01 * regularization)
        self.opti.solver(
            "ipopt",
            {
                "ipopt": {
                    "print_level": 0,
                    "max_iter": 50,
                    "tol": 1e-4,
                },
                "print_time": False,
            },
        )

    @classmethod
    def is_available(cls) -> bool:
        return all(module is not None for module in (casadi, pin, cpin, RobotWrapper))

    @classmethod
    def describe_unavailable(cls) -> str:
        missing = []
        for name in ("casadi", "pinocchio"):
            if importlib.util.find_spec(name) is None:
                missing.append(name)
        reason = f"missing modules={missing}" if missing else f"module import failed: {_IMPORT_ERROR}"
        return f"Piper IK requires {reason}"

    def solve(
        self,
        position: Sequence[float],
        orientation_wxyz: Sequence[float] = (1.0, 0.0, 0.0, 0.0),
        initial_q: Sequence[float] | None = None,
    ) -> np.ndarray | None:
        if initial_q is not None:
            seed = np.array(initial_q, dtype=float).reshape(-1)
            if seed.shape[0] >= self.reduced_robot.model.nq:
                self.init_data = seed[: self.reduced_robot.model.nq].copy()

        self.opti.set_initial(self.var_q, self.init_data)
        self.opti.set_value(
            self.param_tf,
            self._build_transform(position=position, orientation_wxyz=orientation_wxyz),
        )

        try:
            self.opti.solve_limited()
            solution = np.array(self.opti.value(self.var_q), dtype=float).reshape(-1)
        except Exception:
            return None

        max_diff = float(np.max(np.abs(self.history_data - solution)))
        self.init_data = solution.copy()
        if max_diff > np.deg2rad(30.0):
            self.init_data = np.zeros(self.reduced_robot.model.nq, dtype=float)
        self.history_data = solution.copy()
        return solution

    @staticmethod
    def _build_transform(
        position: Sequence[float],
        orientation_wxyz: Sequence[float],
    ) -> np.ndarray:
        quat_xyzw = _normalize_quat_xyzw(orientation_wxyz)
        rotation = pin.Quaternion(
            float(quat_xyzw[3]),
            float(quat_xyzw[0]),
            float(quat_xyzw[1]),
            float(quat_xyzw[2]),
        ).toRotationMatrix()

        transform = np.eye(4, dtype=float)
        transform[:3, :3] = rotation
        transform[:3, 3] = np.array(position, dtype=float)
        return transform
