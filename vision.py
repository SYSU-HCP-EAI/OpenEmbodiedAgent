from internutopia.core.config import Config, SimConfig
from internutopia.core.gym_env import Env
from internutopia.core.util import has_display
from internutopia_extension import import_extensions
from internutopia_extension.configs.robots.aliengo import (
    AliengoRobotCfg,
    move_to_point_cfg,
    camera_cfg   # ⭐ 正确方式
)
from internutopia_extension.configs.tasks import SingleInferenceTaskCfg

import numpy as np
import cv2

# ================= 路径 =================
SCENE_USD = "/home/zyserver/work/my_project/InternUtopia/internutopia/demo/merom_scene_baked.usd"

SPAWN_PRIM = "/World/env_0/scene/MeromScene/carpet_ctclvd_1/Asset/base_link/visuals"

ROBOT_Z = 1.05

# ================= env =================
headless = not has_display()

config = Config(
    simulator=SimConfig(
        physics_dt=1/240,
        rendering_dt=1/240,
        use_fabric=False,
        headless=headless,
        webrtc=headless
    ),
    task_configs=[
        SingleInferenceTaskCfg(
            scene_asset_path=SCENE_USD,
            robots=[
                AliengoRobotCfg(
                    position=(0, 0, ROBOT_Z),
                    controllers=[move_to_point_cfg],
                    sensors=[camera_cfg],   # ⭐ 核心（正确camera）
                )
            ],
        )
    ],
)

# ================= 初始化 =================
import_extensions()

env = Env(config)
obs, _ = env.reset()

# ================= stage =================
from omni.isaac.core.utils.stage import get_current_stage
stage = get_current_stage()

# ================= 灯光 =================
from pxr import UsdLux, UsdGeom, Gf

dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
dome.CreateIntensityAttr(5000)

sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
sun.CreateIntensityAttr(2000)
UsdGeom.Xformable(stage.GetPrimAtPath("/World/Sun")).AddRotateXYZOp().Set(Gf.Vec3f(315, 0, 35))

print("✅ Lighting added")

# ================= 俯视camera =================
import omni.kit.viewport.utility as vp_utils

top_camera_path = "/World/top_view_camera"
camera_prim = UsdGeom.Camera.Define(stage, top_camera_path)

xform = UsdGeom.Xformable(camera_prim)
xform.AddTranslateOp().Set(Gf.Vec3f(2, 2, 18))
xform.AddRotateXYZOp().Set(Gf.Vec3f(90, 0, 0))

env.step({})

viewport = vp_utils.get_active_viewport()
viewport.set_active_camera(top_camera_path)

print("✅ Top view camera set")

# ================= 工具函数 =================
def get_world_pos(stage, prim_path):
    from pxr import UsdGeom
    prim = stage.GetPrimAtPath(prim_path)
    xform = UsdGeom.Xformable(prim)
    mat = xform.ComputeLocalToWorldTransform(0)
    t = mat.ExtractTranslation()
    return np.array([t[0], t[1], t[2]])

def teleport_robot(stage, pos):
    from omni.isaac.core.articulations import ArticulationView

    robot = ArticulationView(prim_paths_expr="/World/env_0/robots/aliengo")
    robot.initialize()

    robot.set_world_poses(
        np.array([[pos[0], pos[1], 1.2]]),
        np.array([[0,0,0,1]])
    )

# ================= Waypoints =================
WAYPOINTS = {
    2: np.array([3.45022, 6.5689, 0.0]),
    4: np.array([1.73975, 5.9176, 0.0]),
    5: np.array([1.73975, 1.09495, 0.0]),
    6: np.array([3.17464, 1.26846, 0.0]),
    7: np.array([1.78321, 3.62602, 0.0]),
    8: np.array([3.09079, 3.62602, 0.0])
}

PATHS = {
    "1": [WAYPOINTS[2]],
    "2": [WAYPOINTS[4], WAYPOINTS[5], WAYPOINTS[6]],
    "3": [WAYPOINTS[5], WAYPOINTS[7], WAYPOINTS[8]]
}

# ================= 初始化位置 =================
spawn = get_world_pos(stage, SPAWN_PRIM)
spawn[2] = 0.0

env.step({})
teleport_robot(stage, spawn)

current_pos = spawn.copy()

# ================= 状态 =================
current_path = []
current_target_index = 0
moving = False

print("\n� 控制说明：")
print("1 = 沙发")
print("2 = 卧室")
print("3 = 浴室")
print("q = 退出\n")

# ================= 主循环 =================
import sys, select

while env.simulation_app.is_running():

    # ⭐ 默认 idle（必须给action）
    idle_action = {
        move_to_point_cfg.name: [tuple(current_pos.tolist())]
    }

    obs, _, _, _, _ = env.step(action=idle_action)

    # ================= 获取camera图像 =================
    if "sensors" in obs:
        cam_data = obs["sensors"][camera_cfg.name]

        if "rgba" in cam_data:
            img = cam_data["rgba"]

            # ⭐ 关键判断（避免崩溃）
            if img is not None and isinstance(img, np.ndarray) and img.ndim == 3:

                img = img[:, :, :3]
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

                cv2.imshow("Robot Camera", img)
                cv2.waitKey(1)

    # ================= 输入 =================
    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
        cmd = sys.stdin.readline().strip()

        if cmd == "q":
            break

        if cmd in PATHS:
            current_path = PATHS[cmd]
            current_target_index = 0
            moving = True
            print(f"� 开始路径 {cmd}")

    # ================= 移动 =================
    if moving and current_target_index < len(current_path):

        goal = current_path[current_target_index]

        env_action = {
            move_to_point_cfg.name: [tuple(goal.tolist())],
        }

        obs, _, _, _, _ = env.step(action=env_action)

        pos = np.array(obs["position"])
        dist = np.linalg.norm(pos[:2] - goal[:2])

        if dist < 0.4:
            print(f"✅ 到达 waypoint {current_target_index}")
            current_target_index += 1
            current_pos = goal

    elif moving:
        print("� 路径完成")
        moving = False