from typing import Dict, List

from internutopia.core.datahub import DataHub


class AgentChat:
    """
    Utility for agents communicating with each other in Datahub.
    """

    def __init__(self, task_name: str, robot_name: str):
        self.read_message_index = 0
        self.task_name = task_name
        self.robot_name = robot_name

    def send_message(self, message: str, at: List[str], parent_idx: int, role: str = 'agent'):
        """
        Send a message to Chatbox.

        Args:
            message (str): message to send
            at (List[str]): note which agent need process this messages. None as default(means everyone).
            parent_idx (int): index of parent message(this message is a reply to parent message).
            role (str): role of agent (agent as default, don't change this pls).
        """
        DataHub.send_chat_control(
            nickname=self.robot_name, task_name=self.task_name, role=role, text=message, at=at, parent_idx=parent_idx
        )

    def get_message(self) -> List[Dict]:
        """
        Get unread messages from Chatbox.

        Returns:
            List: list of unread messages.
        """
        new_chat_data = DataHub.get_chat_control(self.task_name, self.read_message_index)
        self.read_message_index += len(new_chat_data)
        return new_chat_data
