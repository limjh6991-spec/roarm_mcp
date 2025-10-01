"""
MCP Protocol definition for RoArm.

This module defines the Model Context Protocol messages for the robot arm simulation.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Union, Any
import numpy as np
import json


class MCPMessageType(str, Enum):
    """MCP message types for robot arm control."""
    
    # Control messages
    RESET = "reset"
    STEP = "step"
    RENDER = "render"
    CLOSE = "close"
    
    # Info messages
    ACTION_SPACE = "action_space"
    OBSERVATION_SPACE = "observation_space"
    
    # Response messages
    OBSERVATION = "observation"
    REWARD = "reward"
    TERMINATED = "terminated"
    TRUNCATED = "truncated"
    INFO = "info"
    
    # Error messages
    ERROR = "error"


class SpaceType(str, Enum):
    """Space types supported by the MCP protocol."""
    
    DISCRETE = "discrete"
    BOX = "box"
    MULTI_DISCRETE = "multi_discrete"
    MULTI_BINARY = "multi_binary"
    DICT = "dict"


@dataclass
class MCPSpace:
    """Space definition for MCP protocol."""
    
    type: SpaceType
    shape: Optional[List[int]] = None
    low: Optional[Union[float, List[float]]] = None
    high: Optional[Union[float, List[float]]] = None
    n: Optional[int] = None
    nvec: Optional[List[int]] = None
    spaces: Optional[Dict[str, Any]] = None
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert the space to a dictionary."""
        result = {"type": self.type}
        
        if self.shape is not None:
            result["shape"] = self.shape
        if self.low is not None:
            if isinstance(self.low, (list, np.ndarray)):
                result["low"] = list(map(float, self.low))
            else:
                result["low"] = float(self.low)
        if self.high is not None:
            if isinstance(self.high, (list, np.ndarray)):
                result["high"] = list(map(float, self.high))
            else:
                result["high"] = float(self.high)
        if self.n is not None:
            result["n"] = self.n
        if self.nvec is not None:
            result["nvec"] = self.nvec
        if self.spaces is not None:
            spaces_dict = {}
            for key, space in self.spaces.items():
                if isinstance(space, MCPSpace):
                    spaces_dict[key] = space.to_dict()
                else:
                    spaces_dict[key] = space
            result["spaces"] = spaces_dict
            
        return result


class MCPMessage:
    """Base class for MCP messages."""
    
    def __init__(self, type: MCPMessageType):
        self.type = type
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert the message to a dictionary."""
        return {"type": self.type}
    
    def to_json(self) -> str:
        """Convert the message to JSON."""
        return json.dumps(self.to_dict())
    
    @staticmethod
    def from_dict(data: Dict[str, Any]) -> 'MCPMessage':
        """Create a message from a dictionary."""
        msg_type = MCPMessageType(data["type"])
        
        if msg_type == MCPMessageType.RESET:
            return MCPResetMessage()
        elif msg_type == MCPMessageType.STEP:
            return MCPStepMessage(data.get("action"))
        elif msg_type == MCPMessageType.RENDER:
            return MCPRenderMessage()
        elif msg_type == MCPMessageType.CLOSE:
            return MCPCloseMessage()
        elif msg_type == MCPMessageType.ACTION_SPACE:
            return MCPActionSpaceMessage()
        elif msg_type == MCPMessageType.OBSERVATION_SPACE:
            return MCPObservationSpaceMessage()
        elif msg_type == MCPMessageType.OBSERVATION:
            return MCPObservationMessage(data.get("observation"))
        elif msg_type == MCPMessageType.REWARD:
            return MCPRewardMessage(data.get("reward"))
        elif msg_type == MCPMessageType.TERMINATED:
            return MCPTerminatedMessage(data.get("terminated"))
        elif msg_type == MCPMessageType.TRUNCATED:
            return MCPTruncatedMessage(data.get("truncated"))
        elif msg_type == MCPMessageType.INFO:
            return MCPInfoMessage(data.get("info"))
        elif msg_type == MCPMessageType.ERROR:
            return MCPErrorMessage(data.get("error"))
        else:
            raise ValueError(f"Unknown message type: {msg_type}")
    
    @staticmethod
    def from_json(json_str: str) -> 'MCPMessage':
        """Create a message from JSON."""
        return MCPMessage.from_dict(json.loads(json_str))


class MCPResetMessage(MCPMessage):
    """Message to reset the environment."""
    
    def __init__(self):
        super().__init__(MCPMessageType.RESET)


class MCPStepMessage(MCPMessage):
    """Message to step the environment."""
    
    def __init__(self, action: Any):
        super().__init__(MCPMessageType.STEP)
        self.action = action
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert the message to a dictionary."""
        result = super().to_dict()
        result["action"] = self.action
        return result


class MCPRenderMessage(MCPMessage):
    """Message to render the environment."""
    
    def __init__(self):
        super().__init__(MCPMessageType.RENDER)


class MCPCloseMessage(MCPMessage):
    """Message to close the environment."""
    
    def __init__(self):
        super().__init__(MCPMessageType.CLOSE)


class MCPActionSpaceMessage(MCPMessage):
    """Message to get the action space."""
    
    def __init__(self):
        super().__init__(MCPMessageType.ACTION_SPACE)


class MCPObservationSpaceMessage(MCPMessage):
    """Message to get the observation space."""
    
    def __init__(self):
        super().__init__(MCPMessageType.OBSERVATION_SPACE)


class MCPObservationMessage(MCPMessage):
    """Message with an observation."""
    
    def __init__(self, observation: Any):
        super().__init__(MCPMessageType.OBSERVATION)
        self.observation = observation
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert the message to a dictionary."""
        result = super().to_dict()
        result["observation"] = self.observation
        return result


class MCPRewardMessage(MCPMessage):
    """Message with a reward."""
    
    def __init__(self, reward: float):
        super().__init__(MCPMessageType.REWARD)
        self.reward = reward
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert the message to a dictionary."""
        result = super().to_dict()
        result["reward"] = self.reward
        return result


class MCPTerminatedMessage(MCPMessage):
    """Message with a terminated flag."""
    
    def __init__(self, terminated: bool):
        super().__init__(MCPMessageType.TERMINATED)
        self.terminated = terminated
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert the message to a dictionary."""
        result = super().to_dict()
        result["terminated"] = self.terminated
        return result


class MCPTruncatedMessage(MCPMessage):
    """Message with a truncated flag."""
    
    def __init__(self, truncated: bool):
        super().__init__(MCPMessageType.TRUNCATED)
        self.truncated = truncated
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert the message to a dictionary."""
        result = super().to_dict()
        result["truncated"] = self.truncated
        return result


class MCPInfoMessage(MCPMessage):
    """Message with info."""
    
    def __init__(self, info: Dict[str, Any]):
        super().__init__(MCPMessageType.INFO)
        self.info = info
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert the message to a dictionary."""
        result = super().to_dict()
        result["info"] = self.info
        return result


class MCPErrorMessage(MCPMessage):
    """Message with an error."""
    
    def __init__(self, error: str):
        super().__init__(MCPMessageType.ERROR)
        self.error = error
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert the message to a dictionary."""
        result = super().to_dict()
        result["error"] = self.error
        return result