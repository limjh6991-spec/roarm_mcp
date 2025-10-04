"""MCP environment handler for RoArm.

This module provides the integration layer between the `MCPServer` and the
various robot arm environments defined in `roarm_mcp.envs`. It adapts the
synchronous `gymnasium.Env` interface of the robot environments to the
asynchronous `MCPEnvironmentHandler` interface required by the server.
"""

import asyncio
import logging
import numpy as np
from typing import Any, Dict, List, Optional, Tuple, Union

from roarm_mcp.mcp.server import MCPEnvironmentHandler
from roarm_mcp.mcp.protocol import MCPSpace
from roarm_mcp.envs.robot_env import RobotArmEnv, JointPositionEnv, EndEffectorPositionEnv

# Initialize logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class RobotArmMCPHandler(MCPEnvironmentHandler):
    """An MCP environment handler for robot arm simulations.

    This class implements the `MCPEnvironmentHandler` interface, wrapping a
    `RobotArmEnv` instance. It translates the asynchronous calls from the
    `MCPServer` into synchronous calls to the environment, running them in a
    separate thread to avoid blocking the server's asyncio event loop.

    Attributes:
        env (RobotArmEnv): The underlying robot arm environment instance.
    """
    
    def __init__(self, env: RobotArmEnv):
        """Initializes the RobotArmMCPHandler.

        Args:
            env (RobotArmEnv): The robot arm environment to be managed by this
                handler.
        """
        self.env = env
        
    async def reset(self) -> Any:
        """Resets the environment.

        This method calls the environment's `reset` method in a thread-safe
        manner and returns the initial observation.

        Returns:
            Any: The initial observation from the environment.
        """
        # Run in executor to avoid blocking the event loop
        loop = asyncio.get_event_loop()
        observation, _ = await loop.run_in_executor(None, self.env.reset)
        return observation
        
    async def step(self, action: Any) -> Tuple[Any, float, bool, bool, Dict[str, Any]]:
        """Takes a step in the environment.

        This method converts the action to a NumPy array, calls the
        environment's `step` method in a thread-safe manner, and returns the
        step result.

        Args:
            action (Any): The action to take in the environment.

        Returns:
            Tuple[Any, float, bool, bool, Dict[str, Any]]: A tuple containing
            the observation, reward, terminated flag, truncated flag, and info
            dictionary.
        """
        # Convert action to numpy array if it's a list
        if isinstance(action, list):
            action = np.array(action, dtype=np.float32)
            
        # Run in executor to avoid blocking the event loop
        loop = asyncio.get_event_loop()
        step_result = await loop.run_in_executor(None, lambda: self.env.step(action))
        
        observation, reward, terminated, truncated, info = step_result
        return observation, reward, terminated, truncated, info
        
    async def render(self) -> None:
        """Renders the environment.

        This method calls the environment's `render` method in a thread-safe
        manner.
        """
        # Run in executor to avoid blocking the event loop
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(None, self.env.render)
        
    async def close(self) -> None:
        """Closes the environment.

        This method calls the environment's `close` method in a thread-safe
        manner to release resources.
        """
        # Run in executor to avoid blocking the event loop
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(None, self.env.close)
        
    async def get_action_space(self) -> MCPSpace:
        """Gets the action space of the environment.

        Returns:
            MCPSpace: A serializable representation of the environment's
            action space.
        """
        return self.env.get_action_space()
        
    async def get_observation_space(self) -> MCPSpace:
        """Gets the observation space of the environment.

        Returns:
            MCPSpace: A serializable representation of the environment's
            observation space.
        """
        return self.env.get_observation_space()


def create_handler(
    env_type: str = "joint_position",
    robot_type: str = "ur10",
    headless: bool = False
) -> RobotArmMCPHandler:
    """Factory function to create a `RobotArmMCPHandler`.

    This function simplifies the process of creating and configuring a robot
    arm environment and its corresponding MCP handler.

    Args:
        env_type (str): The type of environment to create. Supported values are
            "joint_position" and "end_effector". Defaults to "joint_position".
        robot_type (str): The type of robot to use in the environment.
            Supported values are "ur10" and "franka". Defaults to "ur10".
        headless (bool): Whether to run the environment in headless mode (without
            a graphical interface). Defaults to False.

    Returns:
        RobotArmMCPHandler: An initialized handler ready to be registered with
        an `MCPServer`.

    Raises:
        ValueError: If an unsupported `env_type` is provided.
    """
    logger.info(f"Creating '{env_type}' environment with '{robot_type}' robot.")
    # Create environment
    if env_type == "joint_position":
        env = JointPositionEnv(robot_type=robot_type, headless=headless)
    elif env_type == "end_effector":
        env = EndEffectorPositionEnv(robot_type=robot_type, headless=headless)
    else:
        raise ValueError(f"Unsupported environment type: {env_type}")
        
    # Create handler
    handler = RobotArmMCPHandler(env)
    
    return handler