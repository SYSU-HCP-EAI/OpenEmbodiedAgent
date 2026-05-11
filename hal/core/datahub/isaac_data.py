from typing import Any, Dict, List, Optional

from pydantic import BaseModel

from internutopia.core.util import log


class MetaActionData(BaseModel):
    """
    action status in internutopia
    """

    controller: str
    data: Any


class ActionData(BaseModel):
    """
    action status in internutopia
    """

    robot: str
    controllers: List[MetaActionData]


class _IsaacData(BaseModel):
    """
    isaac status in internutopia
    """

    actions: Optional[Dict[str, Dict[str, Any]]] = {}
    obs: Optional[Dict[str, Dict[str, Any]]] = {}
    task_idx_counter: Optional[int] = 0
    finished_tasks: Optional[List[str]] = []


class IsaacData:
    """
    isaac status in internutopia

    There are three types of isaac status:

    * Action
    * Observation
    * Episode

    structure of isaac status like this::

            {
                actions: {
                    "task_0":{
                        robot_1: {
                            cap: param,
                            controller: param,
                        }
                    }
                },
                observations: {
                    "task_0":{
                        robot_1: {
                            obs_1: data,
                            obs_2: data
                        }
                    }
                },
                task_idx_counter: 0
                finished_tasks: []
            }

    """

    data: _IsaacData = _IsaacData(actions={}, obs={})

    def __init__(self) -> None:
        pass

    @classmethod
    def get_all(cls) -> _IsaacData:
        return cls.data

    # Observation
    @classmethod
    def set_obs_data(cls, obs: Dict[str, Dict[str, Any]]) -> None:
        """
        Set isaac observations data

        Args:
            obs (Dict[str, Dict[str, Any]]): obs data with task_name key.
        """
        for task_id, obs_data in obs.items():
            cls.data.obs[task_id] = obs_data

    @classmethod
    def set_obs_data_by_task_name(cls, task_name: str, obs: Dict[str, Dict[str, Any]]) -> None:
        """
        Set isaac observations data by task name

        Args:
            task_name: isaac task name
            obs (Dict[str, Dict[str, Any]]): obs data with robot_name key.
        """
        for robot_name, obs_data in obs.items():
            cls.data.obs[task_name][robot_name] = obs_data

    @classmethod
    def set_obs_by_task_name_and_robot_name(
        cls, task_name: str, robot_name: str, obs: Dict[str, Dict[str, Any]]
    ) -> None:
        """
        Set isaac observation by task name and robot name

        Args:
            task_name: isaac task name
            robot_name: isaac robot name
            obs (Dict[str, Dict[str, Any]]): obs data with task_name key.
        """
        cls.data.obs[task_name][robot_name] = obs

    @classmethod
    def get_obs(cls) -> Dict[str, Dict]:
        """
        Get isaac observation by task name

        Args:
            task_name: isaac task name

        Returns:
            isaac observation data

        """
        return cls.data.obs

    @classmethod
    def get_obs_by_task_name(cls, task_name: str) -> Dict[str, Dict]:
        """
        Get isaac observation by task name

        Args:
            task_name: isaac task name

        Returns:
            isaac observation data

        """
        if task_name in cls.data.obs:
            return cls.data.obs[task_name]
        return {}

    @classmethod
    def get_obs_by_task_name_and_robot_name(cls, task_name: str, robot_name: str) -> Dict[str, Dict]:
        """
        Get isaac observation by task name and robot name

        Args:
            task_name: isaac task name
            robot_name: isaac robot name

        Returns:
            isaac observation data

        """
        if task_name in cls.data.obs:
            if robot_name in cls.data.obs[task_name]:
                return cls.data.obs[task_name][robot_name]
        return {}

    # Action
    @classmethod
    def set_actions(cls, actions: Dict[str, ActionData]) -> None:
        """
        Set action by task_name

        Args:
            actions (Dict[str, ActionData]): action data with task_name key
        """
        for task_name, action in actions.items():
            if cls.data.actions.get(task_name) is None:
                cls.data.actions[task_name] = {}
            cls.data.actions[task_name].update(action.model_dump())

    @classmethod
    def get_action_by_task_name(cls, task_name: str) -> Dict[str, Dict]:
        """
        Get action by task name

        Returns:
            action(dict like {robot_name: {controller_name: param}})
        """
        if task_name in cls.data.actions:
            return cls.data.actions[task_name]
        return {}

    @classmethod
    def get_action_by_task_name_and_robot_name(cls, task_name: str, robot_name: str) -> Dict[str, Dict]:
        """
        Get action by task name and robot name

        Returns:
            action(dict like {robot_name: {controller_name: param}})
        """
        if task_name in cls.data.actions:
            if robot_name in cls.data.actions[task_name]:
                return cls.data.actions[task_name][robot_name]
        return {}

    @classmethod
    def gen_task_idx(cls) -> str:
        """
        Generate a id for isaac task

        Returns:
            task id
        """
        task_idx = cls.data.task_idx_counter
        cls.data.task_idx_counter += 1
        return str(task_idx)

    @classmethod
    def set_episode_finished(cls, task_name: str):
        """
        Set isaac task {task_name} finished

        Args:
            task_name (str): isaac task_name
        """
        if task_name not in cls.data.finished_tasks:
            cls.data.finished_tasks.append(task_name)
            log.info(f'Set isaac task {task_name} finished')
        return

    @classmethod
    def get_episode_finished(cls, task_name: str) -> bool:
        return task_name in cls.data.finished_tasks
