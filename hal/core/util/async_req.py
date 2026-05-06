import asyncio
import logging.config
from threading import Thread
from typing import Tuple

import httpx

LOGGING_CONFIG = {
    'version': 1,
    'handlers': {'default': {'class': 'logging.StreamHandler', 'formatter': 'http', 'stream': 'ext://sys.stderr'}},
    'formatters': {
        'http': {
            'format': '%(levelname)s [%(asctime)s] %(name)s - %(message)s',
            'datefmt': '%Y-%m-%d %H:%M:%S',
        }
    },
    'loggers': {
        'httpx': {
            'handlers': ['default'],
            'level': 'ERROR',
        },
        'httpcore': {
            'handlers': ['default'],
            'level': 'ERROR',
        },
    },
}

logging.config.dictConfig(LOGGING_CONFIG)

message_send_loop = asyncio.new_event_loop()


def start_loop(loop):
    asyncio.set_event_loop(loop)
    loop.run_forever()


class AsyncRequest:
    loop = None
    res_info = {}

    def __init__(self):
        pass

    @classmethod
    def get(cls, request_id, url, **kwargs) -> Tuple[bool, dict]:
        """
        Send a GET request.
        Args:
            request_id (str): request id(unique)
            url (str): url to send.
            **kwargs: other keyword arguments

        Returns:

        """
        key = url + request_id
        if key not in cls.res_info:
            cls.res_info[key] = None
            asyncio.run_coroutine_threadsafe(cls.async_get(request_id, url, **kwargs), message_send_loop)
        elif cls.res_info[key] is not None:
            res = cls.res_info[key].json()
            del cls.res_info[key]
            return True, res
        return False, cls.res_info[key]

    @classmethod
    async def async_get(cls, request_id, url, **kwargs):
        key = url + request_id
        cls.res_info[key] = httpx.get(url, **kwargs)

    @classmethod
    def post(cls, request_id, url, json, **kwargs):
        key = url + request_id
        if key not in cls.res_info:
            cls.res_info[key] = None
        asyncio.run_coroutine_threadsafe(cls.async_post(request_id, url, json, **kwargs), message_send_loop)

    @classmethod
    async def async_post(cls, request_id, url, json, **kwargs):
        key = url + request_id
        cls.res_info[key] = httpx.post(url, json=json, **kwargs)

    @classmethod
    def get_post_res(cls, request_id, url):
        key = url + request_id
        if key in cls.res_info and cls.res_info[key] is not None:
            res = cls.res_info[key].json()
            del cls.res_info[key]
            return True, res
        return False, None

    @classmethod
    def start_loop(cls):
        t = Thread(target=start_loop, args=(message_send_loop,))
        t.daemon = True
        t.start()
