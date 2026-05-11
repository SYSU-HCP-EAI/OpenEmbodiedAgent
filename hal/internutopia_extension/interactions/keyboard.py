import carb
import numpy as np
import omni

from internutopia.core.util.interaction import BaseInteraction


class KeyboardController:
    def __init__(self):
        self.command = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.subscribe()

    def subscribe(self):
        """
        subscribe to keyboard events
        """
        # subscribe to keyboard events
        app_window = omni.appwindow.get_default_app_window()  # noqa
        key_input = carb.input.acquire_input_interface()  # noqa
        key_input.subscribe_to_keyboard_events(app_window.get_keyboard(), self._sub_keyboard_event)

    def _sub_keyboard_event(self, event, *args, **kwargs):
        """subscribe to keyboard events, map to str"""
        if (
            event.type == carb.input.KeyboardEventType.KEY_PRESS
            or event.type == carb.input.KeyboardEventType.KEY_REPEAT
        ):
            if event.input == carb.input.KeyboardInput.I:
                self.command = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            if event.input == carb.input.KeyboardInput.K:
                self.command = np.array([0.0, 1.0, 0.0, 0.0, 0.0, 0.0])
            if event.input == carb.input.KeyboardInput.J:
                self.command = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
            if event.input == carb.input.KeyboardInput.L:
                self.command = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0])
            if event.input == carb.input.KeyboardInput.U:
                self.command = np.array([0.0, 0.0, 0.0, 0.0, 1.0, 0.0])
            if event.input == carb.input.KeyboardInput.O:
                self.command = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        if event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self.command = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        return True


@BaseInteraction.register('Keyboard')
class KeyboardInteraction(BaseInteraction):
    """Get keyboard input event(i, k, j, l, u, o)"""

    def __init__(self):
        super().__init__()
        self._type = 'Keyboard'
        self.controller = KeyboardController()

    def get_input(self) -> np.ndarray:
        """
        Read input of Keyboard.
        Returns:
            np.ndarray, len == 6, representing (i, k, j, l, u, o) key pressed or not.
        """
        return self.controller.command
