"""
MCP server implementation for RoArm.

This module provides the server implementation for the Model Context Protocol.
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
    """Model Context Protocol server for robot arm control."""

    def __init__(self, host: str = "localhost", port: int = 8765):
        """Initialize the MCP server.
        
        Args:
            host: The host to bind to.
            port: The port to bind to.
        """
        self.host = host
        self.port = port
        self.server = None
        self.clients = set()
        self.env_handler = None
        self.running = False

    async def start(self) -> None:
        """Start the MCP server."""
        if self.server is not None:
            logger.warning("Server is already running")
            return

        logger.info(f"Starting MCP server on {self.host}:{self.port}")
        self.server = await websockets.serve(
            self.handle_client, self.host, self.port
        )
        self.running = True

    async def stop(self) -> None:
        """Stop the MCP server."""
        if self.server is None:
            logger.warning("Server is not running")
            return

        logger.info("Stopping MCP server")
        self.server.close()
        await self.server.wait_closed()
        self.server = None
        self.running = False

    def register_env_handler(self, handler) -> None:
        """Register an environment handler.
        
        Args:
            handler: The environment handler to register.
        """
        self.env_handler = handler
        logger.info("Environment handler registered")

    async def handle_client(
        self, websocket: websockets.WebSocketServerProtocol, path: str
    ) -> None:
        """Handle a client connection.
        
        Args:
            websocket: The websocket connection.
            path: The connection path.
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
        """Process a message from a client.
        
        Args:
            websocket: The websocket connection.
            message: The message to process.
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
        """Handle a reset message.
        
        Args:
            websocket: The websocket connection.
        """
        observation = await self.env_handler.reset()
        await self.send_observation(websocket, observation)

    async def handle_step(
        self, websocket: websockets.WebSocketServerProtocol, action: Any
    ) -> None:
        """Handle a step message.
        
        Args:
            websocket: The websocket connection.
            action: The action to take.
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
        """Handle a render message.
        
        Args:
            websocket: The websocket connection.
        """
        await self.env_handler.render()
        # No response needed for render

    async def handle_close(
        self, websocket: websockets.WebSocketServerProtocol
    ) -> None:
        """Handle a close message.
        
        Args:
            websocket: The websocket connection.
        """
        await self.env_handler.close()
        # No response needed for close

    async def handle_action_space(
        self, websocket: websockets.WebSocketServerProtocol
    ) -> None:
        """Handle an action space message.
        
        Args:
            websocket: The websocket connection.
        """
        action_space = await self.env_handler.get_action_space()
        await websocket.send(json.dumps({"type": "action_space", "space": action_space.to_dict()}))

    async def handle_observation_space(
        self, websocket: websockets.WebSocketServerProtocol
    ) -> None:
        """Handle an observation space message.
        
        Args:
            websocket: The websocket connection.
        """
        observation_space = await self.env_handler.get_observation_space()
        await websocket.send(json.dumps({"type": "observation_space", "space": observation_space.to_dict()}))

    async def send_observation(
        self, websocket: websockets.WebSocketServerProtocol, observation: Any
    ) -> None:
        """Send an observation to a client.
        
        Args:
            websocket: The websocket connection.
            observation: The observation to send.
        """
        msg = MCPObservationMessage(observation)
        await websocket.send(msg.to_json())

    async def send_reward(
        self, websocket: websockets.WebSocketServerProtocol, reward: float
    ) -> None:
        """Send a reward to a client.
        
        Args:
            websocket: The websocket connection.
            reward: The reward to send.
        """
        msg = MCPRewardMessage(reward)
        await websocket.send(msg.to_json())

    async def send_terminated(
        self, websocket: websockets.WebSocketServerProtocol, terminated: bool
    ) -> None:
        """Send a terminated flag to a client.
        
        Args:
            websocket: The websocket connection.
            terminated: The terminated flag to send.
        """
        msg = MCPTerminatedMessage(terminated)
        await websocket.send(msg.to_json())

    async def send_truncated(
        self, websocket: websockets.WebSocketServerProtocol, truncated: bool
    ) -> None:
        """Send a truncated flag to a client.
        
        Args:
            websocket: The websocket connection.
            truncated: The truncated flag to send.
        """
        msg = MCPTruncatedMessage(truncated)
        await websocket.send(msg.to_json())

    async def send_info(
        self, websocket: websockets.WebSocketServerProtocol, info: Dict[str, Any]
    ) -> None:
        """Send info to a client.
        
        Args:
            websocket: The websocket connection.
            info: The info to send.
        """
        msg = MCPInfoMessage(info)
        await websocket.send(msg.to_json())

    async def send_error(
        self, websocket: websockets.WebSocketServerProtocol, error: str
    ) -> None:
        """Send an error to a client.
        
        Args:
            websocket: The websocket connection.
            error: The error to send.
        """
        msg = MCPErrorMessage(error)
        await websocket.send(msg.to_json())


class MCPEnvironmentHandler:
    """Base class for environment handlers."""

    async def reset(self) -> Any:
        """Reset the environment.
        
        Returns:
            The initial observation.
        """
        raise NotImplementedError

    async def step(self, action: Any) -> Tuple[Any, float, bool, bool, Dict[str, Any]]:
        """Take a step in the environment.
        
        Args:
            action: The action to take.
            
        Returns:
            A tuple of (observation, reward, terminated, truncated, info).
        """
        raise NotImplementedError

    async def render(self) -> None:
        """Render the environment."""
        raise NotImplementedError

    async def close(self) -> None:
        """Close the environment."""
        raise NotImplementedError

    async def get_action_space(self) -> Any:
        """Get the action space.
        
        Returns:
            The action space.
        """
        raise NotImplementedError

    async def get_observation_space(self) -> Any:
        """Get the observation space.
        
        Returns:
            The observation space.
        """
        raise NotImplementedError