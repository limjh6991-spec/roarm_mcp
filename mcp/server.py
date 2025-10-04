"""MCP server implementation for RoArm.

This module provides the server implementation for the Model Context Protocol (MCP),
designed to facilitate communication between a robot arm control environment (like
NVIDIA Isaac Sim) and a client agent. The server handles client connections,
processes MCP messages, and interacts with a registered environment handler.

The server is built on top of the `websockets` library for asynchronous
communication.
"""

import asyncio
import json
import logging
import websockets
from typing import Any, Dict, Optional, Callable, Awaitable, List, Tuple, Union
import traceback

from roarm_mcp.mcp.protocol import (
    MCPMessage, MCPResetMessage, MCPStepMessage, MCPRenderMessage, MCPCloseMessage,
    MCPActionSpaceMessage, MCPObservationSpaceMessage, MCPObservationMessage,
    MCPRewardMessage, MCPTerminatedMessage, MCPTruncatedMessage, MCPInfoMessage,
    MCPErrorMessage
)


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class MCPServer:
    """Model Context Protocol server for robot arm control.

    This class implements a WebSocket-based server that listens for client
    connections and facilitates interaction with a simulated or physical
    robot environment. It manages client connections, message passing, and
    delegates environment-specific tasks to an `MCPEnvironmentHandler`.

    Attributes:
        host (str): The hostname or IP address the server binds to.
        port (int): The port number the server listens on.
        server (Optional[websockets.WebSocketServer]): The underlying WebSocket
            server instance.
        clients (set): A set of currently connected WebSocket clients.
        env_handler (Optional[MCPEnvironmentHandler]): The handler for the
            robot environment.
        running (bool): A flag indicating if the server is currently running.
    """

    def __init__(self, host: str = "localhost", port: int = 8765):
        """Initializes the MCPServer.

        Args:
            host (str): The hostname to bind the server to. Defaults to "localhost".
            port (int): The port to bind the server to. Defaults to 8765.
        """
        self.host = host
        self.port = port
        self.server = None
        self.clients = set()
        self.env_handler = None
        self.running = False

    async def start(self) -> None:
        """Starts the MCP server.

        This method initializes the WebSocket server and begins listening for
        incoming client connections. If the server is already running, a warning
        is logged and the method returns.
        """
        if self.server is not None:
            logger.warning("Server is already running")
            return

        logger.info(f"Starting MCP server on {self.host}:{self.port}")
        self.server = await websockets.serve(
            self.handle_client, self.host, self.port
        )
        self.running = True

    async def stop(self) -> None:
        """Stops the MCP server.

        This method gracefully shuts down the WebSocket server and stops
        listening for new connections. If the server is not running, a warning
        is logged and the method returns.
        """
        if self.server is None:
            logger.warning("Server is not running")
            return

        logger.info("Stopping MCP server")
        self.server.close()
        await self.server.wait_closed()
        self.server = None
        self.running = False

    def register_env_handler(self, handler: 'MCPEnvironmentHandler') -> None:
        """Registers an environment handler.

        The environment handler is responsible for implementing the logic for
        interacting with the robot environment (e.g., resetting the simulation,
        executing actions).

        Args:
            handler (MCPEnvironmentHandler): The environment handler instance
                to register with the server.
        """
        self.env_handler = handler
        logger.info("Environment handler registered")

    async def handle_client(
        self, websocket: websockets.WebSocketServerProtocol, path: str
    ) -> None:
        """Handles a new client connection.

        This method is the main entry point for a new client connection. It
        manages the client's lifecycle, listening for incoming messages and
        processing them accordingly.

        Args:
            websocket (websockets.WebSocketServerProtocol): The WebSocket
                connection object for the client.
            path (str): The requested path for the WebSocket connection.
        """
        if self.env_handler is None:
            logger.error("No environment handler registered")
            await websocket.close(1011, "No environment handler registered")
            return

        self.clients.add(websocket)
        logger.info(f"Client connected: {websocket.remote_address}")
        
        try:
            async for message in websocket:
                try:
                    await self.process_message(websocket, message)
                except Exception as e:
                    logger.error(f"Error processing message: {e}")
                    logger.error(traceback.format_exc())
                    await self.send_error(websocket, str(e))
        except websockets.exceptions.ConnectionClosed:
            logger.info(f"Client disconnected: {websocket.remote_address}")
        finally:
            self.clients.remove(websocket)

    async def process_message(
        self, websocket: websockets.WebSocketServerProtocol, message: str
    ) -> None:
        """Processes a single message from a client.

        This method deserializes the message, identifies its type, and calls
        the appropriate handler function.

        Args:
            websocket (websockets.WebSocketServerProtocol): The WebSocket
                connection object for the client.
            message (str): The raw message string received from the client.
        """
        if self.env_handler is None:
            await self.send_error(websocket, "No environment handler registered")
            return

        try:
            data = json.loads(message)
            msg = MCPMessage.from_dict(data)
            
            if isinstance(msg, MCPResetMessage):
                await self.handle_reset(websocket)
            elif isinstance(msg, MCPStepMessage):
                await self.handle_step(websocket, msg.action)
            elif isinstance(msg, MCPRenderMessage):
                await self.handle_render(websocket)
            elif isinstance(msg, MCPCloseMessage):
                await self.handle_close(websocket)
            elif isinstance(msg, MCPActionSpaceMessage):
                await self.handle_action_space(websocket)
            elif isinstance(msg, MCPObservationSpaceMessage):
                await self.handle_observation_space(websocket)
            else:
                await self.send_error(websocket, f"Unsupported message type: {msg.type}")
        except json.JSONDecodeError:
            await self.send_error(websocket, "Invalid JSON")
        except Exception as e:
            logger.error(f"Error processing message: {e}")
            logger.error(traceback.format_exc())
            await self.send_error(websocket, str(e))

    async def handle_reset(
        self, websocket: websockets.WebSocketServerProtocol
    ) -> None:
        """Handles a 'RESET' message.

        This method calls the environment handler's `reset` method and sends
        the initial observation back to the client.

        Args:
            websocket (websockets.WebSocketServerProtocol): The client's
                WebSocket connection.
        """
        observation = await self.env_handler.reset()
        await self.send_observation(websocket, observation)

    async def handle_step(
        self, websocket: websockets.WebSocketServerProtocol, action: Any
    ) -> None:
        """Handles a 'STEP' message.

        This method passes the action from the client to the environment
        handler's `step` method and sends the resulting observation, reward,
        terminated, truncated, and info data back to the client.

        Args:
            websocket (websockets.WebSocketServerProtocol): The client's
                WebSocket connection.
            action (Any): The action to be executed in the environment.
        """
        observation, reward, terminated, truncated, info = await self.env_handler.step(action)
        
        await self.send_observation(websocket, observation)
        await self.send_reward(websocket, reward)
        await self.send_terminated(websocket, terminated)
        await self.send_truncated(websocket, truncated)
        await self.send_info(websocket, info)

    async def handle_render(
        self, websocket: websockets.WebSocketServerProtocol
    ) -> None:
        """Handles a 'RENDER' message.

        This method calls the environment handler's `render` method. No response
        is sent to the client.

        Args:
            websocket (websockets.WebSocketServerProtocol): The client's
                WebSocket connection.
        """
        await self.env_handler.render()
        # No response needed for render

    async def handle_close(
        self, websocket: websockets.WebSocketServerProtocol
    ) -> None:
        """Handles a 'CLOSE' message.

        This method calls the environment handler's `close` method. No response
        is sent to the client.

        Args:
            websocket (websockets.WebSocketServerProtocol): The client's
                WebSocket connection.
        """
        await self.env_handler.close()
        # No response needed for close

    async def handle_action_space(
        self, websocket: websockets.WebSocketServerProtocol
    ) -> None:
        """Handles an 'ACTION_SPACE' message.

        This method retrieves the action space from the environment handler and
        sends it to the client.

        Args:
            websocket (websockets.WebSocketServerProtocol): The client's
                WebSocket connection.
        """
        action_space = await self.env_handler.get_action_space()
        await websocket.send(json.dumps({"type": "action_space", "space": action_space.to_dict()}))

    async def handle_observation_space(
        self, websocket: websockets.WebSocketServerProtocol
    ) -> None:
        """Handles an 'OBSERVATION_SPACE' message.

        This method retrieves the observation space from the environment
        handler and sends it to the client.

        Args:
            websocket (websockets.WebSocketServerProtocol): The client's
                WebSocket connection.
        """
        observation_space = await self.env_handler.get_observation_space()
        await websocket.send(json.dumps({"type": "observation_space", "space": observation_space.to_dict()}))

    async def send_observation(
        self, websocket: websockets.WebSocketServerProtocol, observation: Any
    ) -> None:
        """Sends an observation message to a client.

        Args:
            websocket (websockets.WebSocketServerProtocol): The client's
                WebSocket connection.
            observation (Any): The observation data to send.
        """
        msg = MCPObservationMessage(observation)
        await websocket.send(msg.to_json())

    async def send_reward(
        self, websocket: websockets.WebSocketServerProtocol, reward: float
    ) -> None:
        """Sends a reward message to a client.

        Args:
            websocket (websockets.WebSocketServerProtocol): The client's
                WebSocket connection.
            reward (float): The reward value to send.
        """
        msg = MCPRewardMessage(reward)
        await websocket.send(msg.to_json())

    async def send_terminated(
        self, websocket: websockets.WebSocketServerProtocol, terminated: bool
    ) -> None:
        """Sends a terminated message to a client.

        Args:
            websocket (websockets.WebSocketServerProtocol): The client's
                WebSocket connection.
            terminated (bool): The terminated flag to send.
        """
        msg = MCPTerminatedMessage(terminated)
        await websocket.send(msg.to_json())

    async def send_truncated(
        self, websocket: websockets.WebSocketServerProtocol, truncated: bool
    ) -> None:
        """Sends a truncated message to a client.

        Args:
            websocket (websockets.WebSocketServerProtocol): The client's
                WebSocket connection.
            truncated (bool): The truncated flag to send.
        """
        msg = MCPTruncatedMessage(truncated)
        await websocket.send(msg.to_json())

    async def send_info(
        self, websocket: websockets.WebSocketServerProtocol, info: Dict[str, Any]
    ) -> None:
        """Sends an info message to a client.

        Args:
            websocket (websockets.WebSocketServerProtocol): The client's
                WebSocket connection.
            info (Dict[str, Any]): The info dictionary to send.
        """
        msg = MCPInfoMessage(info)
        await websocket.send(msg.to_json())

    async def send_error(
        self, websocket: websockets.WebSocketServerProtocol, error: str
    ) -> None:
        """Sends an error message to a client.

        Args:
            websocket (websockets.WebSocketServerProtocol): The client's
                WebSocket connection.
            error (str): The error message to send.
        """
        msg = MCPErrorMessage(error)
        await websocket.send(msg.to_json())


class MCPEnvironmentHandler:
    """Abstract base class for MCP environment handlers.

    This class defines the interface that an environment handler must implement
    to be compatible with the `MCPServer`. The handler is responsible for all
    direct interactions with the simulation or robot hardware.
    """

    async def reset(self) -> Any:
        """Resets the environment to an initial state.

        Returns:
            Any: The initial observation after the reset.

        Raises:
            NotImplementedError: If the method is not implemented by a subclass.
        """
        raise NotImplementedError

    async def step(self, action: Any) -> Tuple[Any, float, bool, bool, Dict[str, Any]]:
        """Executes one time step in the environment.

        Args:
            action (Any): The action to be performed in the environment.

        Returns:
            Tuple[Any, float, bool, bool, Dict[str, Any]]: A tuple containing:
                - observation (Any): The observation of the environment's state.
                - reward (float): The reward returned after taking the action.
                - terminated (bool): Whether the episode has ended.
                - truncated (bool): Whether the episode was truncated.
                - info (Dict[str, Any]): A dictionary with auxiliary diagnostic
                  information.

        Raises:
            NotImplementedError: If the method is not implemented by a subclass.
        """
        raise NotImplementedError

    async def render(self) -> None:
        """Renders the environment.

        The exact behavior of this method depends on the specific environment
        implementation (e.g., it could update a GUI, save an image, etc.).

        Raises:
            NotImplementedError: If the method is not implemented by a subclass.
        """
        raise NotImplementedError

    async def close(self) -> None:
        """Performs any necessary cleanup for the environment.

        This method should be called when the environment is no longer needed.

        Raises:
            NotImplementedError: If the method is not implemented by a subclass.
        """
        raise NotImplementedError

    async def get_action_space(self) -> Any:
        """Returns the action space of the environment.

        Returns:
            Any: An object representing the action space (e.g., a `gym.spaces`
            object).

        Raises:
            NotImplementedError: If the method is not implemented by a subclass.
        """
        raise NotImplementedError

    async def get_observation_space(self) -> Any:
        """Returns the observation space of the environment.

        Returns:
            Any: An object representing the observation space (e.g., a
            `gym.spaces` object).

        Raises:
            NotImplementedError: If the method is not implemented by a subclass.
        """
        raise NotImplementedError