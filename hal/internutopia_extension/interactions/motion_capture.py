import time

import requests


class MocapInteraction:
    def __init__(self, mocap_url):
        self.base_url = mocap_url
        self.is_runing = False
        self.server_start()
        time.sleep(1)  # asynchronous calculation of buffer time

    def server_start(self):
        if self.is_runing:
            return

        response = requests.get(f'{self.base_url}/start')
        if response.status_code == 400:
            self.is_runing = True
            return

        if response.status_code != 200:
            raise RuntimeError(f'server start failed: {response.json()}')

        self.is_runing = True

    def step(self):
        response = requests.get(f'{self.base_url}/results')
        if response.status_code != 200:
            raise RuntimeError(f'mocap server get result failed: {response.json()}')
        cur_mocap_info = response.json()

        return cur_mocap_info

    def server_stop(self):
        if not self.is_runing:
            return

        response = requests.get(f'{self.base_url}/stop')
        if response.status_code == 400:
            self.is_runing = False
            return

        if response.status_code != 200:
            raise RuntimeError(f'server stop failed: {response.json()}')

        self.is_runing = False
