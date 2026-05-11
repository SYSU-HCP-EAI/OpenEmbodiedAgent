import time
import typing

import carb
import numpy as np
import omni.usd
from pxr import Gf, Usd


def compute_path_bbox(prim_path: str) -> typing.Tuple[carb.Double3, carb.Double3]:
    """
    Compute Bounding Box using omni.usd.UsdContext.compute_path_world_bounding_box
    See https://docs.omniverse.nvidia.com/kit/docs/omni.usd/latest/omni.usd/omni.usd.UsdContext.html#\
    omni.usd.UsdContext.compute_path_world_bounding_box

    Args:
        prim_path: A prim path to compute the bounding box.
    Returns:
        A range (i.e. bounding box) as a minimum point and maximum point.
    """
    return omni.usd.get_context().compute_path_world_bounding_box(prim_path)


def get_pick_position(robot_base_position: np.ndarray, prim_path: str) -> np.ndarray:
    """Get the pick position for a manipulator robots to pick an objects at prim_path.
    The pick position is simply the nearest top vertex of the objects's bounding box.

    Args:
        robot_base_position (np.ndarray): robots base position.
        prim_path (str): prim path of objects to pick.

    Returns:
        np.ndarray: pick position.
    """
    bbox_0, bbox_1 = compute_path_bbox(prim_path)

    x1 = bbox_0[0]
    x2 = bbox_1[0]
    y1 = bbox_0[1]
    y2 = bbox_1[1]
    top_z = bbox_0[2] if bbox_0[2] > bbox_1[2] else bbox_1[2]

    top_vertices = [
        np.array([x1, y1, top_z]),
        np.array([x1, y2, top_z]),
        np.array([x2, y1, top_z]),
        np.array([x2, y2, top_z]),
    ]

    print('================================ Top vertices: ', top_vertices, ' ====================================')

    pick_position = top_vertices[0]
    for vertex in top_vertices:
        if np.linalg.norm(robot_base_position - vertex) < np.linalg.norm(robot_base_position - pick_position):
            pick_position = vertex

    return pick_position


def get_grabbed_able_xform_paths(root_path: str, prim: Usd.Prim, depth: int = 3) -> typing.List[str]:
    """get all prim paths of Xform objects under specified prim.

    Args:
        root_path (str): root path of scenes.
        prim (Usd.Prim): target prim.
        depth (int, optional): expected depth of Xform objects relative to root_path. Defaults to 3.

    Returns:
        typing.List[str]: prim paths.
    """
    paths = []
    if prim is None:
        return paths
    print(f'get_grabbed_able_xform_paths: start to traverse {prim.GetPrimPath()}')
    relative_prim_path = str(prim.GetPrimPath())[len(root_path) :]
    if relative_prim_path.count('/') <= depth:
        for child in prim.GetChildren():
            if child.GetTypeName() == 'Scope':
                paths.extend(get_grabbed_able_xform_paths(root_path, child))
            if child.GetTypeName() == 'Xform':
                paths.append(str(child.GetPrimPath()))

    return paths


def get_world_transform_xform(prim: Usd.Prim) -> typing.Tuple[Gf.Vec3d, Gf.Rotation, Gf.Vec3d]:
    """
    Get the local transformation of a prim using omni.usd.get_world_transform_matrix().
    See https://docs.omniverse.nvidia.com/kit/docs/omni.usd/latest/omni.usd/omni.usd.get_world_transform_matrix.html
    Args:
        prim: The prim to calculate the world transformation.
    Returns:
        A tuple of:
        - Translation vector.
        - Rotation quaternion, i.e. 3d vector plus angle.
        - Scale vector.
    """
    world_transform: Gf.Matrix4d = omni.usd.get_world_transform_matrix(prim)
    translation: Gf.Vec3d = world_transform.ExtractTranslation()
    rotation: Gf.Rotation = world_transform.ExtractRotation()
    scale: Gf.Vec3d = Gf.Vec3d(*(v.GetLength() for v in world_transform.ExtractRotationMatrix()))
    return translation, rotation, scale


def nearest_xform_from_position(
    stage: Usd.Stage, xform_paths: typing.List[str], position: np.ndarray, threshold: float = 0
) -> str:
    """get prim path of nearest Xform objects from the target position.

    Args:
        stage (Usd.Stage): usd stage.
        xform_paths (typing.List[str]): full list of xforms paths.
        position (np.ndarray): target position.
        threshold (float, optional): max distance. Defaults to 0 (unlimited).

    Returns:
        str: prim path of the Xform objects, None if not found.
    """
    start = time.time()
    if threshold == 0:
        threshold = 1000000.0
    min_dist = threshold
    nearest_prim_path = None
    for path in xform_paths:
        prim = stage.GetPrimAtPath(path)
        if prim is not None and prim.IsValid():
            pose = get_world_transform_xform(prim)
            dist = np.linalg.norm(pose[0] - position)
            if dist < min_dist:
                min_dist = dist
                nearest_prim_path = path

    print(f'nearest_xform_from_position costs: {time.time() - start}')
    return nearest_prim_path
