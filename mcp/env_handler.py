"""
MCP environment handler for RoArm.

This module provides the integration between the MCP server and robot arm environments.
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
    """MCP environment handler for robot arm environments."""
    
    def __init__(self, env: RobotArmEnv):
        """Initialize the MCP environment handler.
        
        Args:
            env: The robot arm environment.
        """
        self.env = env
        
    async def reset(self) -> Any:
        """Reset the environment.
        
        Returns:
            The initial observation.
        """
        # Run in executor to avoid blocking the event loop
        loop = asyncio.get_event_loop()
        observation = await loop.run_in_executor(None, self.env.reset)
        return observation
        
    async def step(self, action: Any) -> Tuple[Any, float, bool, bool, Dict[str, Any]]:
        """Take a step in the environment.
        
        Args:
            action: The action to take.
            
        Returns:
            A tuple of (observation, reward, terminated, truncated, info).
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
        """Render the environment."""
        # Run in executor to avoid blocking the event loop
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(None, self.env.render)
        
    async def close(self) -> None:
        """Close the environment."""
        # Run in executor to avoid blocking the event loop
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(None, self.env.close)
        
    async def get_action_space(self) -> MCPSpace:
        """Get the action space.
        
        Returns:
            The action space.
        """
        return self.env.get_action_space()
        
    async def get_observation_space(self) -> MCPSpace:
        """Get the observation space.
        
        Returns:
            The observation space.
        """
        return self.env.get_observation_space()


def create_handler(
    env_type: str = "joint_position",
    robot_type: str = "ur10",
    headless: bool = False
) -> RobotArmMCPHandler:
    """Create an MCP environment handler for a robot arm environment.
    
    Args:
        env_type: The type of environment to create ("joint_position" or "end_effector").
        robot_type: The type of robot to use ("ur10" or "franka").
        headless: Whether to run in headless mode.
        
    Returns:
        The MCP environment handler.
    """
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