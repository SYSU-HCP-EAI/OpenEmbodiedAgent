from typing import Any, Dict, Optional

from pydantic import BaseModel


class RewardCfg(BaseModel, extra='allow'):
    """
    Configures reward settings for a system.

    This class defines the structure of reward configurations, which includes specifying the type of reward and optional settings associated with that reward. It is designed to be used as a base model for creating reward configurations in various applications like machine learning algorithms, game development, or any system where rewards are instrumental in guiding behavior.

    Attributes:
        reward_type (str): Specifies the type of reward, determining the nature of the feedback provided by the system.
        reward_settings (Optional[Dict[str, Any]], optional): Additional settings tailored to the specified reward type. Defaults to None. This dictionary can hold a variety of parameters depending on the complexity required for configuring the reward mechanism.
    """

    reward_type: str
    reward_settings: Optional[Dict[str, Any]] = None
