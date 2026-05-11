import asyncio
import logging
import time
from multiprocessing import Array, Process, Queue, Value, shared_memory
from multiprocessing.synchronize import Event as SyncEvent
from typing import Tuple

import numpy as np
from vuer import Vuer
from vuer.schemas import Hands, ImageBackground

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')


def setup_logging():
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)
    handler = logging.StreamHandler()
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    return logger


class OpenTeleVision:
    def __init__(
        self,
        img_shape: Tuple[int, int],
        shm_name: str,
        queue: Queue,
        toggle_streaming: SyncEvent,
        cert_file='./cert.pem',
        key_file='./key.pem',
        stream_mode='image',
    ):
        # self.app=Vuer()
        self.logger = setup_logging()

        self.img_shape = (img_shape[0], 2 * img_shape[1], 3)
        self.img_height, self.img_width = img_shape[:2]

        self.app = Vuer(host='0.0.0.0', cert=cert_file, key=key_file, queries=dict(grid=False))
        self.app.add_handler('HAND_MOVE')(self.on_hand_move)
        self.app.add_handler('CAMERA_MOVE')(self.on_cam_move)
        if stream_mode == 'image':
            existing_shm = shared_memory.SharedMemory(name=shm_name)
            self.img_array = np.ndarray(
                (self.img_shape[0], self.img_shape[1], 3), dtype=np.uint8, buffer=existing_shm.buf
            )
            self.app.spawn(start=False)(self.main_image)
        else:
            raise ValueError("stream_mode must be 'image'")

        self.left_hand_shared = Array('d', 16, lock=True)
        self.right_hand_shared = Array('d', 16, lock=True)
        self.left_landmarks_shared = Array('d', 75, lock=True)
        self.right_landmarks_shared = Array('d', 75, lock=True)

        self.head_matrix_shared = Array('d', 16, lock=True)
        self.aspect_shared = Value('d', 1.0, lock=True)

        self.begin_move = Value('b', False, lock=True)

        self.process = Process(target=self.run)
        self.process.daemon = True
        self.process.start()

    def run(self):
        # setup_logging()
        self.app.run()

    def cleanup(self):
        self.process.kill()
        self.process.join()

    async def on_cam_move(self, event, session, fps=60):
        # only intercept the ego camera.
        # if event.key != "ego":
        #     return
        try:
            # with self.head_matrix_shared.get_lock():  # Use the lock to ensure thread-safe updates
            #     self.head_matrix_shared[:] = event.value["camera"]["matrix"]
            # with self.aspect_shared.get_lock():
            #     self.aspect_shared.value = event.value['camera']['aspect']
            self.head_matrix_shared[:] = event.value['camera']['matrix']
            self.aspect_shared.value = event.value['camera']['aspect']
        except:  # noqa: E722
            pass
        # self.head_matrix = np.array(event.value["camera"]["matrix"]).reshape(4, 4, order="F")
        # print(np.array(event.value["camera"]["matrix"]).reshape(4, 4, order="F"))
        # print("camera moved", event.value["matrix"].shape, event.value["matrix"])

    async def on_hand_move(self, event, session, fps=60):
        # try:
        #     self.logger.debug(f"begin move: {event.value['leftHand']}")

        #     # with self.left_hand_shared.get_lock():  # Use the lock to ensure thread-safe updates
        #     #     self.left_hand_shared[:] = np.array(event.value["leftHand"]).reshape(4, 4, order="F")

        #     # self.logger.debug(f"left done: {self.begin_move}")

        #     # with self.right_hand_shared.get_lock():
        #     #     self.right_hand_shared[:] = np.array(event.value["rightHand"]).reshape(4, 4, order="F")
        #     # self.logger.debug(f"right done: {self.begin_move}")
        #     # with self.left_landmarks_shared.get_lock():
        #     #     self.left_landmarks_shared[:] = np.array(event.value["leftLandmarks"]).flatten()
        #     # self.logger.debug(f"left landmarks done: {self.begin_move}")
        #     # with self.right_landmarks_shared.get_lock():
        #     #     self.right_landmarks_shared[:] = np.array(event.value["rightLandmarks"]).flatten()
        #     # self.logger.debug("landmarks done")
        #     # with self.begin_move.get_lock():
        #     #     self.begin_move = True
        #     self.logger.debug("begin_move done")
        try:
            # self.logger.debug(f"begin move: {self.begin_move.value}")
            self.begin_move.value = True
            self.left_hand_shared[:] = event.value['leftHand']
            self.right_hand_shared[:] = event.value['rightHand']
            self.left_landmarks_shared[:] = np.array(event.value['leftLandmarks']).flatten()
            self.right_landmarks_shared[:] = np.array(event.value['rightLandmarks']).flatten()
        except:  # noqa: E722
            # sometimes you cannot find hands
            pass

    async def main_image(self, session, fps=60):
        session.upsert @ Hands(fps=fps, stream=True, key='hands', showLeft=False, showRight=False)
        # end_time = time.time()
        while True:
            # start = time.time()
            # print(end_time - start)
            # aspect = self.aspect_shared.value
            display_image = self.img_array

            # session.upsert(
            # ImageBackground(
            #     # Can scale the images down.
            #     display_image[:self.img_height],
            #     # 'jpg' encoding is significantly faster than 'png'.
            #     format="jpeg",
            #     quality=80,
            #     key="left-image",
            #     interpolate=True,
            #     # fixed=True,
            #     aspect=1.778,
            #     distanceToCamera=2,
            #     position=(0, -0.5, -2),
            #     rotation=[0, 0, 0],
            # ),
            # to="bgChildren",
            # )

            session.upsert(
                [
                    ImageBackground(
                        # Can scale the images down.
                        display_image[::2, : self.img_width],
                        # display_image[:self.img_height:2, ::2],
                        # 'jpg' encoding is significantly faster than 'png'.
                        format='jpeg',
                        quality=80,
                        key='left-image',
                        interpolate=True,
                        # fixed=True,
                        aspect=1.66667,
                        # distanceToCamera=0.5,
                        height=8,
                        position=(0, -1, 3),
                        # rotation=[0, 0, 0],
                        layers=1,
                        alphaSrc='./vinette.jpg',
                    ),
                    ImageBackground(
                        # Can scale the images down.
                        display_image[::2, self.img_width :],
                        # display_image[self.img_height::2, ::2],
                        # 'jpg' encoding is significantly faster than 'png'.
                        format='jpeg',
                        quality=80,
                        key='right-image',
                        interpolate=True,
                        # fixed=True,
                        aspect=1.66667,
                        # distanceToCamera=0.5,
                        height=8,
                        position=(0, -1, 3),
                        # rotation=[0, 0, 0],
                        layers=2,
                        alphaSrc='./vinette.jpg',
                    ),
                ],
                to='bgChildren',
            )
            # rest_time = 1/fps - time.time() + start
            # end_time = time.time()
            await asyncio.sleep(0.03)

    @property
    def left_hand(self):
        # with self.left_hand_shared.get_lock():
        #     return np.array(self.left_hand_shared[:]).reshape(4, 4, order="F")
        return np.array(self.left_hand_shared[:]).reshape(4, 4, order='F')

    @property
    def right_hand(self):
        # with self.right_hand_shared.get_lock():
        #     return np.array(self.right_hand_shared[:]).reshape(4, 4, order="F")
        return np.array(self.right_hand_shared[:]).reshape(4, 4, order='F')

    @property
    def left_landmarks(self):
        # with self.left_landmarks_shared.get_lock():
        #     return np.array(self.left_landmarks_shared[:]).reshape(25, 3)
        return np.array(self.left_landmarks_shared[:]).reshape(25, 3)

    @property
    def right_landmarks(self):
        # with self.right_landmarks_shared.get_lock():
        # return np.array(self.right_landmarks_shared[:]).reshape(25, 3)
        return np.array(self.right_landmarks_shared[:]).reshape(25, 3)

    @property
    def head_matrix(self):
        # with self.head_matrix_shared.get_lock():
        #     return np.array(self.head_matrix_shared[:]).reshape(4, 4, order="F")
        return np.array(self.head_matrix_shared[:]).reshape(4, 4, order='F')

    @property
    def aspect(self):
        # with self.aspect_shared.get_lock():
        # return float(self.aspect_shared.value)
        return float(self.aspect_shared.value)


if __name__ == '__main__':
    resolution = (720, 1280)
    crop_size_w = 340  # (resolution[1] - resolution[0]) // 2
    crop_size_h = 270
    resolution_cropped = (resolution[0] - crop_size_h, resolution[1] - 2 * crop_size_w)  # 450 * 600
    img_shape = (2 * resolution_cropped[0], resolution_cropped[1], 3)  # 900 * 600
    img_height, img_width = resolution_cropped[:2]  # 450 * 600
    shm = shared_memory.SharedMemory(create=True, size=np.prod(img_shape) * np.uint8().itemsize)
    shm_name = shm.name
    img_array = np.ndarray((img_shape[0], img_shape[1], 3), dtype=np.uint8, buffer=shm.buf)

    tv = OpenTeleVision(resolution_cropped, cert_file='../cert.pem', key_file='../key.pem')
    while True:
        # print(tv.left_landmarks)
        # print(tv.left_hand)
        # tv.modify_shared_image(random=True)
        time.sleep(1)
