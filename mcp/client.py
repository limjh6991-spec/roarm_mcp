"""MCP client implementation for RoArm.

This module provides the client-side implementation for the Model Context Protocol
(MCP). The `MCPClient` class acts as a `gymnasium.Env`, allowing an agent to
interact with a remote robot environment managed by an `MCPServer`.
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
    """A Gymnasium environment client for the Model Context Protocol.

    This class provides a standard `gym.Env` interface for interacting with a
    remote environment via WebSockets. It handles the connection to the
    `MCPServer`, message serialization/deserialization, and the asynchronous
    communication flow, presenting it synchronously to the agent.

    Attributes:
        host (str): The hostname or IP address of the MCPServer.
        port (int): The port number of the MCPServer.
        timeout (float): The timeout in seconds for WebSocket operations.
        websocket (Optional[websockets.WebSocketClientProtocol]): The active
            WebSocket connection instance.
        action_space (Optional[gym.Space]): The action space of the remote
            environment.
        observation_space (Optional[gym.Space]): The observation space of the
            remote environment.
        connected (bool): A flag indicating if the client is connected.
        loop (Optional[asyncio.AbstractEventLoop]): The asyncio event loop used
            for running async operations.
    """

    metadata = {"render_modes": ["human"]}

    def __init__(self, host: str = "localhost", port: int = 8765, timeout: float = 10.0):
        """Initializes the MCPClient.

        Args:
            host (str): The host to connect to. Defaults to "localhost".
            port (int): The port to connect to. Defaults to 8765.
            timeout (float): The timeout for operations in seconds. Defaults to 10.0.
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
        """Establishes a WebSocket connection to the MCP server.

        This private method connects to the server, retrieves the action and
        observation spaces, and sets the `connected` flag.

        Raises:
            Exception: If the connection to the server fails.
        """
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
        """Retrieves action and observation spaces from the server."""
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
        """Parses a space dictionary from the server into a `gym.Space` object.

        Args:
            space_dict (Dict[str, Any]): A dictionary describing the space,
                received from the server.

        Returns:
            gym.Space: The corresponding `gymnasium.spaces` object.

        Raises:
            ValueError: If the space type specified in the dictionary is unknown.
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
        """Resets the remote environment.

        This method sends a 'RESET' message to the server and waits for the
        initial observation.

        Args:
            seed (Optional[int]): The random seed for the environment.
            options (Optional[Dict[str, Any]]): Additional options for resetting
                the environment.

        Returns:
            Tuple[Any, Dict[str, Any]]: A tuple containing the initial
            observation and an empty info dictionary.
        """
        if seed is not None:
            super().reset(seed=seed)
            
        if self.loop is None:
            self.loop = asyncio.get_event_loop()
            
        # Create a task to reset the environment
        return self.loop.run_until_complete(self._reset_async())

    async def _reset_async(self) -> Tuple[Any, Dict[str, Any]]:
        """Asynchronously resets the environment.

        Returns:
            Tuple[Any, Dict[str, Any]]: A tuple containing the initial
            observation and an empty info dictionary.
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
        """Takes a step in the remote environment.

        This method sends a 'STEP' message with the given action to the server
        and waits for the resulting state information.

        Args:
            action (Any): The action to take in the environment.

        Returns:
            Tuple[Any, float, bool, bool, Dict[str, Any]]: A tuple containing
            the observation, reward, terminated flag, truncated flag, and info
            dictionary.
        """
        if self.loop is None:
            self.loop = asyncio.get_event_loop()
            
        # Create a task to step the environment
        return self.loop.run_until_complete(self._step_async(action))

    async def _step_async(self, action: Any) -> Tuple[Any, float, bool, bool, Dict[str, Any]]:
        """Asynchronously takes a step in the environment.

        Args:
            action (Any): The action to take.

        Returns:
            Tuple[Any, float, bool, bool, Dict[str, Any]]: A tuple containing
            the observation, reward, terminated flag, truncated flag, and info
            dictionary.

        Raises:
            RuntimeError: If an error message is received from the server.
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
        """Requests the server to render the environment.

        This method sends a 'RENDER' message. No response is expected.
        """
        if self.loop is None:
            self.loop = asyncio.get_event_loop()
            
        # Create a task to render the environment
        self.loop.run_until_complete(self._render_async())

    async def _render_async(self) -> None:
        """Asynchronously requests the server to render the environment."""
        if not self.connected:
            await self._connect()
            
        # Send render message
        render_msg = MCPRenderMessage()
        await self.websocket.send(render_msg.to_json())
        
        # No response expected for render

    def close(self) -> None:
        """Closes the connection to the server.

        This method sends a 'CLOSE' message and terminates the WebSocket
        connection.
        """
        if self.loop is None and not self.connected:
            return

        if self.loop is None:
            self.loop = asyncio.get_event_loop()
            
        # Create a task to close the environment
        self.loop.run_until_complete(self._close_async())

    async def _close_async(self) -> None:
        """Asynchronously closes the connection to the server."""
        if not self.connected:
            return
            
        # Send close message
        close_msg = MCPCloseMessage()
        try:
            await self.websocket.send(close_msg.to_json())
        except websockets.exceptions.ConnectionClosed:
            logger.warning("Connection already closed when sending close message.")
        finally:
            # Close the websocket
            await self.websocket.close()
            self.connected = False