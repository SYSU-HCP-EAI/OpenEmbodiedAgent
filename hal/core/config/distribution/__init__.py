from typing import Optional

from pydantic import BaseModel, Extra


class DistributionCfg(BaseModel, extra=Extra.allow):
    type: str
    proc_num: Optional[int] = 1
    gpu_num_per_proc: Optional[float] = 1


class RayDistributionCfg(DistributionCfg):
    type: Optional[str] = 'ray'
    head_address: Optional[str] = None
    working_dir: Optional[str] = None
