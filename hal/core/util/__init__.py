import os

from internutopia.core.util.async_req import AsyncRequest
from internutopia.core.util.log import log

if not AsyncRequest.loop:
    AsyncRequest.start_loop()


def is_in_container() -> bool:
    try:
        os.stat('/.dockerenv')
        return True
    except FileNotFoundError:
        return False
    except Exception as e:
        log.error(f'Error while checking if in container, assume False: {e}')
        return False


def has_display() -> bool:
    try:
        display = os.environ['DISPLAY']
        if display is not None and display != '':
            return True
        return False
    except KeyError:
        return False
    except Exception as e:
        log.error(f'Error while checking if has display, assume False: {e}')
        return False


def remove_suffix(name: str) -> str:
    """Remove the suffix after the last underscore in the name, if exists."""
    last_underscore_index = name.rfind('_')
    if last_underscore_index > 0 and name[last_underscore_index + 1 :].isdigit():
        return name[:last_underscore_index]
    return name
