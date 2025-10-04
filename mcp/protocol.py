"""MCP Protocol definition for RoArm.

This module defines the data structures and message types for the Model Context
Protocol (MCP), which is used for communication between the `MCPServer` and
`MCPClient`. The protocol is designed to be extensible and supports various
types of messages for controlling and querying a robot arm environment.

The main components defined here are:
- `MCPMessageType`: An enumeration of all possible message types.
- `SpaceType`: An enumeration for `gymnasium.spaces` types.
- `MCPSpace`: A serializable representation of a `gymnasium.space`.
- `MCPMessage`: A base class for all MCP messages, with subclasses for each
  specific message type.
"""

from dataclasses import dataclass, asdict, fields
from enum import Enum
from typing import Dict, List, Optional, Union, Any
import numpy as np
import json


class MCPMessageType(str, Enum):
    """Enumeration of MCP message types.

    This enum defines the valid `type` strings for MCP messages, categorizing
    them into control, info, response, and error messages.
    """
    
    # Control messages sent from client to server
    RESET = "reset"
    STEP = "step"
    RENDER = "render"
    CLOSE = "close"
    
    # Info messages sent from client to server to request information
    ACTION_SPACE = "action_space"
    OBSERVATION_SPACE = "observation_space"
    
    # Response messages sent from server to client
    OBSERVATION = "observation"
    REWARD = "reward"
    TERMINATED = "terminated"
    TRUNCATED = "truncated"
    INFO = "info"
    
    # Error messages sent from server to client
    ERROR = "error"


class SpaceType(str, Enum):
    """Enumeration of `gymnasium.spaces` types supported by MCP.

    This allows for the serialization and deserialization of space definitions
    between the server and client.
    """
    
    DISCRETE = "discrete"
    BOX = "box"
    MULTI_DISCRETE = "multi_discrete"
    MULTI_BINARY = "multi_binary"
    DICT = "dict"


@dataclass
class MCPSpace:
    """A serializable representation of a `gymnasium.spaces` object.

    This class is used to transfer definitions of action and observation spaces
    over the MCP protocol.

    Attributes:
        type (SpaceType): The type of the space (e.g., 'box', 'discrete').
        shape (Optional[List[int]]): The shape of the space (for Box, etc.).
        low (Optional[Union[float, List[float]]]): The lower bound(s) of the space.
        high (Optional[Union[float, List[float]]]): The upper bound(s) of the space.
        n (Optional[int]): The number of discrete actions (for Discrete).
        nvec (Optional[List[int]]): A vector of discrete action counts (for MultiDiscrete).
        spaces (Optional[Dict[str, Any]]): A dictionary of subspaces (for Dict).
    """
    
    type: SpaceType
    shape: Optional[List[int]] = None
    low: Optional[Union[float, List[float], np.ndarray]] = None
    high: Optional[Union[float, List[float], np.ndarray]] = None
    n: Optional[int] = None
    nvec: Optional[List[int]] = None
    spaces: Optional[Dict[str, 'MCPSpace']] = None
    
    def to_dict(self) -> Dict[str, Any]:
        """Converts the MCPSpace object to a JSON-serializable dictionary.

        This method handles the conversion of numpy arrays to lists and recursively
        calls `to_dict` for nested spaces.

        Returns:
            Dict[str, Any]: A dictionary representation of the space.
        """
        result = {"type": self.type.value}
        
        for field in fields(self):
            if field.name == 'type':
                continue
            
            value = getattr(self, field.name)
            if value is None:
                continue

            if field.name in ["low", "high"] and isinstance(value, np.ndarray):
                result[field.name] = value.tolist()
            elif field.name == "spaces" and isinstance(value, dict):
                result[field.name] = {k: v.to_dict() for k, v in value.items()}
            else:
                result[field.name] = value

        return result


class MCPMessage:
    """Base class for all MCP messages.

    Provides common functionality for serialization and deserialization.

    Attributes:
        type (MCPMessageType): The type of the message.
    """
    
    def __init__(self, type: MCPMessageType):
        """Initializes the MCPMessage.

        Args:
            type (MCPMessageType): The type of the message.
        """
        self.type = type
    
    def to_dict(self) -> Dict[str, Any]:
        """Converts the message to a dictionary.

        Returns:
            Dict[str, Any]: A dictionary containing the message type.
        """
        return {"type": self.type.value}
    
    def to_json(self) -> str:
        """Serializes the message to a JSON string.

        Returns:
            str: The JSON representation of the message.
        """
        return json.dumps(self.to_dict())
    
    @staticmethod
    def from_dict(data: Dict[str, Any]) -> 'MCPMessage':
        """Creates an MCPMessage instance from a dictionary.

        This factory method determines the correct message subclass to instantiate
        based on the 'type' field in the input dictionary.

        Args:
            data (Dict[str, Any]): The dictionary to deserialize.

        Returns:
            MCPMessage: An instance of the appropriate MCPMessage subclass.

        Raises:
            ValueError: If the message type is unknown.
        """
        msg_type_str = data.get("type")
        if not msg_type_str:
            raise ValueError("Message data must contain a 'type' field.")

        try:
            msg_type = MCPMessageType(msg_type_str)
        except ValueError:
            raise ValueError(f"Unknown message type: {msg_type_str}")
        
        # Mapping from message type to class and expected arguments
        message_map = {
            MCPMessageType.RESET: (MCPResetMessage, []),
            MCPMessageType.STEP: (MCPStepMessage, ["action"]),
            MCPMessageType.RENDER: (MCPRenderMessage, []),
            MCPMessageType.CLOSE: (MCPCloseMessage, []),
            MCPMessageType.ACTION_SPACE: (MCPActionSpaceMessage, []),
            MCPMessageType.OBSERVATION_SPACE: (MCPObservationSpaceMessage, []),
            MCPMessageType.OBSERVATION: (MCPObservationMessage, ["observation"]),
            MCPMessageType.REWARD: (MCPRewardMessage, ["reward"]),
            MCPMessageType.TERMINATED: (MCPTerminatedMessage, ["terminated"]),
            MCPMessageType.TRUNCATED: (MCPTruncatedMessage, ["truncated"]),
            MCPMessageType.INFO: (MCPInfoMessage, ["info"]),
            MCPMessageType.ERROR: (MCPErrorMessage, ["error"]),
        }

        if msg_type in message_map:
            msg_class, arg_keys = message_map[msg_type]
            args = {key: data.get(key) for key in arg_keys}
            return msg_class(**args)
        else:
            # This case should ideally not be reached if the enum is comprehensive
            raise ValueError(f"Message type '{msg_type}' is defined but not handled in from_dict.")
    
    @staticmethod
    def from_json(json_str: str) -> 'MCPMessage':
        """Creates an MCPMessage instance from a JSON string.

        Args:
            json_str (str): The JSON string to deserialize.

        Returns:
            MCPMessage: An instance of the appropriate MCPMessage subclass.
        """
        return MCPMessage.from_dict(json.loads(json_str))


@dataclass
class MCPResetMessage(MCPMessage):
    """Message to reset the environment."""
    def __init__(self):
        super().__init__(MCPMessageType.RESET)


@dataclass
class MCPStepMessage(MCPMessage):
    """Message to step the environment with a given action.
    
    Attributes:
        action (Any): The action to be executed by the environment.
    """
    action: Any
    def __init__(self, action: Any):
        super().__init__(MCPMessageType.STEP)
        self.action = action
    
    def to_dict(self) -> Dict[str, Any]:
        """Converts the message to a dictionary."""
        d = super().to_dict()
        d["action"] = self.action
        return d


@dataclass
class MCPRenderMessage(MCPMessage):
    """Message to request the environment to render."""
    def __init__(self):
        super().__init__(MCPMessageType.RENDER)


@dataclass
class MCPCloseMessage(MCPMessage):
    """Message to close the environment and release resources."""
    def __init__(self):
        super().__init__(MCPMessageType.CLOSE)


@dataclass
class MCPActionSpaceMessage(MCPMessage):
    """Message to request the environment's action space."""
    def __init__(self):
        super().__init__(MCPMessageType.ACTION_SPACE)


@dataclass
class MCPObservationSpaceMessage(MCPMessage):
    """Message to request the environment's observation space."""
    def __init__(self):
        super().__init__(MCPMessageType.OBSERVATION_SPACE)


@dataclass
class MCPObservationMessage(MCPMessage):
    """Message containing an observation from the environment.
    
    Attributes:
        observation (Any): The observation data from the environment.
    """
    observation: Any
    def __init__(self, observation: Any):
        super().__init__(MCPMessageType.OBSERVATION)
        self.observation = observation

    def to_dict(self) -> Dict[str, Any]:
        """Converts the message to a dictionary."""
        d = super().to_dict()
        d["observation"] = self.observation
        return d


@dataclass
class MCPRewardMessage(MCPMessage):
    """Message containing a reward from the environment.
    
    Attributes:
        reward (float): The reward value.
    """
    reward: float
    def __init__(self, reward: float):
        super().__init__(MCPMessageType.REWARD)
        self.reward = reward

    def to_dict(self) -> Dict[str, Any]:
        """Converts the message to a dictionary."""
        d = super().to_dict()
        d["reward"] = self.reward
        return d


@dataclass
class MCPTerminatedMessage(MCPMessage):
    """Message indicating whether the episode has terminated.
    
    Attributes:
        terminated (bool): True if the episode has ended, False otherwise.
    """
    terminated: bool
    def __init__(self, terminated: bool):
        super().__init__(MCPMessageType.TERMINATED)
        self.terminated = terminated

    def to_dict(self) -> Dict[str, Any]:
        """Converts the message to a dictionary."""
        d = super().to_dict()
        d["terminated"] = self.terminated
        return d


@dataclass
class MCPTruncatedMessage(MCPMessage):
    """Message indicating whether the episode was truncated.
    
    Attributes:
        truncated (bool): True if the episode was truncated, False otherwise.
    """
    truncated: bool
    def __init__(self, truncated: bool):
        super().__init__(MCPMessageType.TRUNCATED)
        self.truncated = truncated

    def to_dict(self) -> Dict[str, Any]:
        """Converts the message to a dictionary."""
        d = super().to_dict()
        d["truncated"] = self.truncated
        return d


@dataclass
class MCPInfoMessage(MCPMessage):
    """Message containing auxiliary diagnostic information.
    
    Attributes:
        info (Dict[str, Any]): A dictionary of supplementary information.
    """
    info: Dict[str, Any]
    def __init__(self, info: Dict[str, Any]):
        super().__init__(MCPMessageType.INFO)
        self.info = info
    
    def to_dict(self) -> Dict[str, Any]:
        """Converts the message to a dictionary."""
        d = super().to_dict()
        d["info"] = self.info
        return d


@dataclass
class MCPErrorMessage(MCPMessage):
    """Message containing an error message.
    
    Attributes:
        error (str): A string describing the error.
    """
    error: str
    def __init__(self, error: str):
        super().__init__(MCPMessageType.ERROR)
        self.error = error

    def to_dict(self) -> Dict[str, Any]:
        """Converts the message to a dictionary."""
        d = super().to_dict()
        d["error"] = self.error
        return d