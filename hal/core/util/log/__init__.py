import os
from typing import Optional

import toml
from pydantic import BaseModel

from internutopia.core.util.log.logger import Logger


class LogConfig(BaseModel):
    filename: Optional[str] = None
    level: Optional[str] = 'info'
    fmt: Optional[str] = '[%(asctime)s][%(levelname)s] %(pathname)s[line:%(lineno)d] -: %(message)s'


# with open(os.path.join(os.path.split(os.path.realpath(__file__))[0], 'config.ini'), 'r') as f:
#     config = LogConfig(**(toml.loads(f.read())['log']))

log_dict = {'level': 'error', 'fmt': '[%(asctime)s][%(levelname)s] %(pathname)s[line:%(lineno)d] -: %(message)s'}

config = LogConfig(**log_dict)

# Use this rather than `Logger`
log = Logger(
    filename=config.filename,
    level=config.level,
    fmt=config.fmt,
).log
