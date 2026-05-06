from abc import ABC, abstractmethod
from functools import wraps

import numpy as np


class BaseInteraction(ABC):
    interactions = {}

    def __init__(self):
        self._type = None

    @property
    def type(self):
        return self._type

    @abstractmethod
    def get_input(self) -> np.ndarray:
        """
        Get input from UID.
        Returns:
            nd.array: Input data.
        """
        raise NotImplementedError

    @classmethod
    def register(cls, name):
        """
        Register an interaction (like keyboard,gamepad or vr controllers) class with its name(decorator).
        Args:
            name(str): name of the robot class.
        """

        def decorator(interaction_class):
            cls.interactions[name] = interaction_class

            @wraps(interaction_class)
            def wrapped_function(*args, **kwargs):
                return interaction_class(*args, **kwargs)

            return wrapped_function

        return decorator
