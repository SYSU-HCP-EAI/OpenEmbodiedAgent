from typing import List, Optional

from pydantic import BaseModel, Field


class LogData(BaseModel):
    type: str = Field(..., examples=['agent'], description='message(notification)|agent(response)|user(question)')
    name: Optional[str] = Field(None, examples=['Bob'], description='nickname')
    time: Optional[str] = Field(None, examples=['17:20'], description='time of the message')
    message: str = Field(
        ...,
        examples=['his image shows a blue computer screen with a white background.'],
        description='text content of the message',
    )
    photo: Optional[str] = Field(
        None, examples=['https://static.openxlab.org.cn/puyu/demo/000-2x.jpg'], description='avatar url'
    )


class ChainOfThoughtDataItem(BaseModel):
    type: str = Field(..., examples=['text'], description='time(notification)|message(normal message)')
    value: str = Field(..., examples=['I need to do the following things'], description='text content of the message')


class ChatControlData(BaseModel):
    idx: Optional[int] = Field(-1, examples=[0, 1], description='index of this message in chat')
    type: str = Field(..., examples=['agent'], description='message(notification)|agent(response)|user(question)')
    name: Optional[str] = Field(None, examples=['Agent'], description='nickname')
    time: Optional[str] = Field(None, examples=['18:20'], description='time of the message')
    message: Optional[str] = Field(
        None, examples=['Hi, Understand this picture'], description='text content of the message'
    )
    photo: Optional[str] = Field(
        None, examples=['https://static.openxlab.org.cn/puyu/demo/000-2x.jpg'], description='avatar url'
    )
    img: Optional[str] = Field(
        None,
        examples=['https://static.openxlab.org.cn/puyu/demo/000-2x.jpg'],
        description='image content of the message',
    )
    status: Optional[str] = Field(
        None, examples=['pending'], description='status of the message, legal values: pending'
    )
    at: Optional[List[str]] = Field(None, examples=[['agent1', 'agent2']], description='who the message send to')
    parent_idx: Optional[int] = Field(None, examples=[2], description='index of the parent message in chatbox')


class ModelData:
    data = {
        'log_data': {},
        'chain_of_thought_data': {},
        'chat_control_data': {},
    }

    def __init__(self) -> None:
        pass

    @classmethod
    def get_log_data(cls, task_name: str = 'default'):
        if task_name not in cls.data['log_data']:
            cls.data['log_data'][task_name] = []
        return cls.data['log_data'][task_name]

    @classmethod
    def get_chan_of_thought(cls, task_name: str = 'default'):
        if task_name not in cls.data['chat_control_data']:
            cls.data['chat_control_data'][task_name] = []
        return cls.data['chain_of_thought_data'][task_name]

    @classmethod
    def get_chat_control(cls, task_name: str = 'default', read_inx: int = 0):
        if task_name not in cls.data['chat_control_data']:
            cls.data['chat_control_data'][task_name] = []
        return cls.data['chat_control_data'][task_name][read_inx:]

    @classmethod
    def append_log_data(cls, data: LogData, task_name: str = 'default'):
        if task_name not in cls.data['log_data']:
            cls.data['log_data'][task_name] = []
        cls.data['log_data'][task_name].append(data.dict())

    @classmethod
    def append_chan_of_thought(cls, data: List[ChainOfThoughtDataItem], task_name: str = 'default'):
        if task_name not in cls.data['chat_control_data']:
            cls.data['chat_control_data'][task_name] = []
        cls.data['chain_of_thought_data'][task_name].append([d.dict() for d in data])

    @classmethod
    def append_chat_control(cls, data: ChatControlData, task_name: str = 'default'):
        if task_name not in cls.data['chat_control_data']:
            cls.data['chat_control_data'][task_name] = []
        data.idx = len(cls.data['chat_control_data'])
        cls.data['chat_control_data'][task_name].append(data.dict())

    @classmethod
    def clear(cls, task_name: str = None):
        if task_name is None:
            cls.data['chat_control_data'].clear()
            cls.data['chain_of_thought_data'].clear()
            cls.data['log_data'].clear()
        else:
            cls.data['chat_control_data'][task_name].clear()
            cls.data['chain_of_thought_data'][task_name].clear()
            cls.data['log_data'][task_name].clear()
