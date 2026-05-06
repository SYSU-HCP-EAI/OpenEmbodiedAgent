from multiprocessing import Event, Queue, shared_memory

import numpy as np

from .Preprocessor import VuerPreprocessor
from .TeleVision import OpenTeleVision


class VuerTeleop:
    def __init__(
        self,
        cert_file='./InternUtopia/mkcert/cert.pem',
        key_file='./InternUtopia/mkcert/key.pem',
        resolution=(720, 1280),
    ):
        self.resolution = resolution
        self.crop_size_w = 0
        self.crop_size_h = 0
        self.resolution_cropped = (self.resolution[0] - self.crop_size_h, self.resolution[1] - 2 * self.crop_size_w)

        self.img_shape = (self.resolution_cropped[0], 2 * self.resolution_cropped[1], 3)
        self.img_height, self.img_width = self.resolution_cropped[:2]

        self.shm = shared_memory.SharedMemory(create=True, size=np.prod(self.img_shape) * np.uint8().itemsize)
        self.img_array = np.ndarray((self.img_shape[0], self.img_shape[1], 3), dtype=np.uint8, buffer=self.shm.buf)
        image_queue = Queue()
        toggle_streaming = Event()
        self.tv = OpenTeleVision(
            self.resolution_cropped, self.shm.name, image_queue, toggle_streaming, cert_file, key_file
        )
        self.processor = VuerPreprocessor()

    def step(self):
        head_mat, left_wrist_mat, right_wrist_mat, left_hand_mat, right_hand_mat = self.processor.process(self.tv)
        begin_move = self.tv.begin_move.value

        return head_mat, left_wrist_mat, right_wrist_mat, left_hand_mat, right_hand_mat, begin_move

    def cleanup(self):
        self.tv.cleanup()
