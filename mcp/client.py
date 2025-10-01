"""
MCP client implementation for RoArm.

This module provides the client implementation for the Model Context Protocol.
"""

import asyncio
import json
import logging
from typing import Any, Dict, List, Optional, Tuple, Union

import numpy as np
import gymnasium as gym
from gymnasium.spaces import Space, Box, Discrete, Dict as DictSpace, MultiDiscrete, MultiBinary

from roarm_mcp.mcp.protocol import (
    MCPMessage, MCPResetMessage, MCPStepMessage, MCPRenderMessage, MCPCloseMessage,
    MCPActionSpaceMessage, MCPObservationSpaceMessage, MCPSpace, SpaceType
)

try:
    import websockets
except ImportError:
    logging.warning("websockets package not found. Install it using 'pip install websockets'")


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class MCPClient(gym.Env):
    """Model Context Protocol client for robot arm control."""

    metadata = {"render_modes": ["human"]}

    def __init__(self, host: str = "localhost", port: int = 8765, timeout: float = 10.0):
        """Initialize the MCP client.
        
        Args:
            host: The host to connect to.
            port: The port to connect to.
            timeout: The timeout for operations in seconds.
        """
        super().__init__()
        
        self.host = host
        self.port = port
        self.timeout = timeout
        self.websocket = None
        self.action_space = None
        self.observation_space = None
        self.connected = False
        self.loop = None

    async def _connect(self) -> None:
        """Connect to the MCP server."""
        if self.connected:
            return
            
        try:
            uri = f"ws://{self.host}:{self.port}"
            logger.info(f"Connecting to MCP server at {uri}")
            self.websocket = await asyncio.wait_for(
                websockets.connect(uri), timeout=self.timeout
            )
            self.connected = True
            
            # Get the action and observation spaces
            await self._get_spaces()
        except Exception as e:
            logger.error(f"Error connecting to MCP server: {e}")
            self.connected = False
            raise

    async def _get_spaces(self) -> None:
        """Get the action and observation spaces."""
        if not self.connected:
            await self._connect()
        
        # Get action space
        action_space_msg = MCPActionSpaceMessage()
        await self.websocket.send(action_space_msg.to_json())
        response = await asyncio.wait_for(self.websocket.recv(), timeout=self.timeout)
        data = json.loads(response)
        self.action_space = self._parse_space(data["space"])
        
        # Get observation space
        observation_space_msg = MCPObservationSpaceMessage()
        await self.websocket.send(observation_space_msg.to_json())
        response = await asyncio.wait_for(self.websocket.recv(), timeout=self.timeout)
        data = json.loads(response)
        self.observation_space = self._parse_space(data["space"])

    def _parse_space(self, space_dict: Dict[str, Any]) -> gym.Space:
        """Parse a space dictionary into a gym.Space.
        
        Args:
            space_dict: The space dictionary to parse.
            
        Returns:
            The parsed gym.Space.
        """
        space_type = SpaceType(space_dict["type"])
        
        if space_type == SpaceType.DISCRETE:
            return gym.spaces.Discrete(space_dict["n"])
        elif space_type == SpaceType.BOX:
            low = np.array(space_dict["low"], dtype=np.float32)
            high = np.array(space_dict["high"], dtype=np.float32)
            return gym.spaces.Box(low, high, dtype=np.float32)
        elif space_type == SpaceType.MULTI_DISCRETE:
            return gym.spaces.MultiDiscrete(space_dict["nvec"])
        elif space_type == SpaceType.MULTI_BINARY:
            return gym.spaces.MultiBinary(space_dict["shape"])
        elif space_type == SpaceType.DICT:
            spaces = {}
            for key, sub_space_dict in space_dict["spaces"].items():
                spaces[key] = self._parse_space(sub_space_dict)
            return gym.spaces.Dict(spaces)
        else:
            raise ValueError(f"Unknown space type: {space_type}")

    def reset(self, seed: Optional[int] = None, options: Optional[Dict[str, Any]] = None) -> Tuple[Any, Dict[str, Any]]:
        """Reset the environment.
        
        Args:
            seed: Random seed.
            options: Reset options.
            
        Returns:
            A tuple of (observation, info).
        """
        if seed is not None:
            super().reset(seed=seed)
            
        if self.loop is None:
            self.loop = asyncio.get_event_loop()
            
        # Create a task to reset the environment
        return self.loop.run_until_complete(self._reset_async())

    async def _reset_async(self) -> Tuple[Any, Dict[str, Any]]:
        """Reset the environment asynchronously.
        
        Returns:
            A tuple of (observation, info).
        """
        if not self.connected:
            await self._connect()
            
        # Send reset message
        reset_msg = MCPResetMessage()
        await self.websocket.send(reset_msg.to_json())
        
        # Wait for observation
        response = await asyncio.wait_for(self.websocket.recv(), timeout=self.timeout)
        data = json.loads(response)
        observation = data["observation"]
        
        # Empty info dictionary
        info = {}
        
        return observation, info

    def step(self, action: Any) -> Tuple[Any, float, bool, bool, Dict[str, Any]]:
        """Take a step in the environment.
        
        Args:
            action: The action to take.
            
        Returns:
            A tuple of (observation, reward, terminated, truncated, info).
        """
        if self.loop is None:
            self.loop = asyncio.get_event_loop()
            
        # Create a task to step the environment
        return self.loop.run_until_complete(self._step_async(action))

    async def _step_async(self, action: Any) -> Tuple[Any, float, bool, bool, Dict[str, Any]]:
        """Take a step in the environment asynchronously.
        
        Args:
            action: The action to take.
            
        Returns:
            A tuple of (observation, reward, terminated, truncated, info).
        """
        if not self.connected:
            await self._connect()
            
        # Send step message
        step_msg = MCPStepMessage(action)
        await self.websocket.send(step_msg.to_json())
        
        # Wait for responses
        observation = None
        reward = 0.0
        terminated = False
        truncated = False
        info = {}
        
        # We expect 5 messages: observation, reward, terminated, truncated, info
        for _ in range(5):
            response = await asyncio.wait_for(self.websocket.recv(), timeout=self.timeout)
            data = json.loads(response)
            msg_type = data["type"]
            
            if msg_type == "observation":
                observation = data["observation"]
            elif msg_type == "reward":
                reward = data["reward"]
            elif msg_type == "terminated":
                terminated = data["terminated"]
            elif msg_type == "truncated":
                truncated = data["truncated"]
            elif msg_type == "info":
                info = data["info"]
            elif msg_type == "error":
                logger.error(f"Error from server: {data['error']}")
                raise RuntimeError(f"Error from server: {data['error']}")
        
        return observation, reward, terminated, truncated, info

    def render(self) -> None:
        """Render the environment."""
        if self.loop is None:
            self.loop = asyncio.get_event_loop()
            
        # Create a task to render the environment
        return self.loop.run_until_complete(self._render_async())

    async def _render_async(self) -> None:
        """Render the environment asynchronously."""
        if not self.connected:
            await self._connect()
            
        # Send render message
        render_msg = MCPRenderMessage()
        await self.websocket.send(render_msg.to_json())
        
        # No response expected for render

    def close(self) -> None:
        """Close the environment."""
        if self.loop is None:
            self.loop = asyncio.get_event_loop()
            
        # Create a task to close the environment
        return self.loop.run_until_complete(self._close_async())

    async def _close_async(self) -> None:
        """Close the environment asynchronously."""
        if not self.connected:
            return
            
        # Send close message
        close_msg = MCPCloseMessage()
        await self.websocket.send(close_msg.to_json())
        
        # Close the websocket
        await self.websocket.close()
        self.connected = False