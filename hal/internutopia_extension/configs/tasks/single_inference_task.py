from typing import Optional

from internutopia.core.config.task import TaskCfg


class SingleInferenceTaskCfg(TaskCfg):
    type: Optional[str] = 'SingleInferenceTask'
    # 在场景加载后、机器人创建前为场景内 Mesh 补碰撞（Merom 等烘焙地台）；勿在 env.reset() 之后再改同批几何
    enable_static_scene_mesh_collision_patch: bool = False
