"""
Environments for RoArm MCP.

This module provides reinforcement learning environments for robot arm control.
"""

import logging
import gym
import gymnasium
import numpy as np
from typing import Any, Dict, List, Optional, Tuple, Union

from roarm_mcp.isaac_sim.simulator import IsaacSimRobotEnv
from roarm_mcp.robot.arms import UR10Robot, FrankaRobot
from roarm_mcp.robot.controller import PositionController
from roarm_mcp.mcp.protocol import MCPSpace, SpaceType

# Initialize logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class RobotArmEnv:
    """Base class for robot arm environments."""
    
    def __init__(self):
        """Initialize the robot arm environment."""
        self.observation_space = None
        self.action_space = None
        
    def reset(self) -> np.ndarray:
        """Reset the environment.
        
        Returns:
            The initial observation.
        """
        raise NotImplementedError
        
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        """Take a step in the environment.
        
        Args:
            action: The action to take.
            
        Returns:
            A tuple of (observation, reward, terminated, truncated, info).
        """
        raise NotImplementedError
        
    def render(self) -> None:
        """Render the environment."""
        raise NotImplementedError
        
    def close(self) -> None:
        """Close the environment."""
        raise NotImplementedError
        
    def get_action_space(self) -> MCPSpace:
        """Get the action space.
        
        Returns:
            The action space.
        """
        if isinstance(self.action_space, gymnasium.spaces.Box):
            return MCPSpace(
                type=SpaceType.BOX,
                shape=list(self.action_space.shape),
                low=self.action_space.low.tolist(),
                high=self.action_space.high.tolist()
            )
        elif isinstance(self.action_space, gymnasium.spaces.Discrete):
            return MCPSpace(
                type=SpaceType.DISCRETE,
                n=self.action_space.n
            )
        elif isinstance(self.action_space, gymnasium.spaces.MultiDiscrete):
            return MCPSpace(
                type=SpaceType.MULTI_DISCRETE,
                nvec=self.action_space.nvec.tolist()
            )
        elif isinstance(self.action_space, gymnasium.spaces.MultiBinary):
            return MCPSpace(
                type=SpaceType.MULTI_BINARY,
                shape=list(self.action_space.shape)
            )
        elif isinstance(self.action_space, gymnasium.spaces.Dict):
            spaces = {}
            for key, space in self.action_space.spaces.items():
                spaces[key] = self.get_space(space)
            return MCPSpace(
                type=SpaceType.DICT,
                spaces=spaces
            )
        else:
            raise ValueError(f"Unsupported action space type: {type(self.action_space)}")
        
    def get_observation_space(self) -> MCPSpace:
        """Get the observation space.
        
        Returns:
            The observation space.
        """
        if isinstance(self.observation_space, gymnasium.spaces.Box):
            return MCPSpace(
                type=SpaceType.BOX,
                shape=list(self.observation_space.shape),
                low=self.observation_space.low.tolist(),
                high=self.observation_space.high.tolist()
            )
        elif isinstance(self.observation_space, gymnasium.spaces.Discrete):
            return MCPSpace(
                type=SpaceType.DISCRETE,
                n=self.observation_space.n
            )
        elif isinstance(self.observation_space, gymnasium.spaces.MultiDiscrete):
            return MCPSpace(
                type=SpaceType.MULTI_DISCRETE,
                nvec=self.observation_space.nvec.tolist()
            )
        elif isinstance(self.observation_space, gymnasium.spaces.MultiBinary):
            return MCPSpace(
                type=SpaceType.MULTI_BINARY,
                shape=list(self.observation_space.shape)
            )
        elif isinstance(self.observation_space, gymnasium.spaces.Dict):
            spaces = {}
            for key, space in self.observation_space.spaces.items():
                spaces[key] = self.get_space(space)
            return MCPSpace(
                type=SpaceType.DICT,
                spaces=spaces
            )
        else:
            raise ValueError(f"Unsupported observation space type: {type(self.observation_space)}")
    
    def get_space(self, space: gymnasium.spaces.Space) -> MCPSpace:
        """Convert a gym.Space to an MCPSpace.
        
        Args:
            space: The gym.Space to convert.
            
        Returns:
            The converted MCPSpace.
        """
        if isinstance(space, gymnasium.spaces.Box):
            return MCPSpace(
                type=SpaceType.BOX,
                shape=list(space.shape),
                low=space.low.tolist(),
                high=space.high.tolist()
            )
        elif isinstance(space, gymnasium.spaces.Discrete):
            return MCPSpace(
                type=SpaceType.DISCRETE,
                n=space.n
            )
        elif isinstance(space, gymnasium.spaces.MultiDiscrete):
            return MCPSpace(
                type=SpaceType.MULTI_DISCRETE,
                nvec=space.nvec.tolist()
            )
        elif isinstance(space, gymnasium.spaces.MultiBinary):
            return MCPSpace(
                type=SpaceType.MULTI_BINARY,
                shape=list(space.shape)
            )
        elif isinstance(space, gymnasium.spaces.Dict):
            spaces = {}
            for key, sub_space in space.spaces.items():
                spaces[key] = self.get_space(sub_space)
            return MCPSpace(
                type=SpaceType.DICT,
                spaces=spaces
            )
        else:
            raise ValueError(f"Unsupported space type: {type(space)}")


class JointPositionEnv(RobotArmEnv):
    """Environment for joint position control."""
    
    def __init__(
        self,
        robot_type: str = "ur10",
        headless: bool = False,
        time_step: float = 1.0 / 60.0
    ):
        """Initialize the joint position control environment.
        
        Args:
            robot_type: The type of robot to use ("ur10" or "franka").
            headless: Whether to run in headless mode.
            time_step: The simulation time step.
        """
        super().__init__()
        
        # Create robot environment
        self.robot_env = IsaacSimRobotEnv(
            robot_usd_path=f"/Isaac/Robots/{robot_type.upper()}/{robot_type.lower()}.usd",
            robot_name=robot_type.lower(),
            headless=headless,
            time_step=time_step
        )
        
        # Create robot
        if robot_type.lower() == "ur10":
            self.robot = UR10Robot(robot_env=self.robot_env)
            self.num_joints = 6
        elif robot_type.lower() == "franka":
            self.robot = FrankaRobot(robot_env=self.robot_env)
            self.num_joints = 7
        else:
            raise ValueError(f"Unsupported robot type: {robot_type}")
        
        # Get joint limits
        self.joint_limits = self.robot.get_joint_limits()
        
        # Define action and observation spaces
        self.action_space = gymnasium.spaces.Box(
            low=self.joint_limits["lower"],
            high=self.joint_limits["upper"],
            dtype=np.float32
        )
        
        # Observation space includes joint positions, velocities, and target position
        self.observation_space = gymnasium.spaces.Dict({
            "joint_positions": gymnasium.spaces.Box(
                low=self.joint_limits["lower"],
                high=self.joint_limits["upper"],
                dtype=np.float32
            ),
            "joint_velocities": gymnasium.spaces.Box(
                low=-np.ones(self.num_joints) * 10.0,
                high=np.ones(self.num_joints) * 10.0,
                dtype=np.float32
            ),
            "target_position": gymnasium.spaces.Box(
                low=self.joint_limits["lower"],
                high=self.joint_limits["upper"],
                dtype=np.float32
            )
        })
        
        # Initialize state
        self.target_position = self.robot.get_home_position()
        self.steps = 0
        self.max_steps = 1000
        
    def reset(self) -> np.ndarray:
        """Reset the environment.
        
        Returns:
            The initial observation.
        """
        # Reset simulation
        self.robot_env.reset()
        
        # Move robot to home position
        self.robot.move_to_home()
        
        # Set random target position
        self.target_position = self.sample_target_position()
        
        # Reset steps
        self.steps = 0
        
        # Get observation
        return self._get_observation()
        
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        """Take a step in the environment.
        
        Args:
            action: The action to take (target joint position).
            
        Returns:
            A tuple of (observation, reward, terminated, truncated, info).
        """
        # Clip action to joint limits
        action = np.clip(action, self.joint_limits["lower"], self.joint_limits["upper"])
        
        # Set joint positions
        self.robot.controller.move_to_joint_positions(action)
        
        # Step simulation
        self.robot_env.step()
        
        # Increment steps
        self.steps += 1
        
        # Get observation
        observation = self._get_observation()
        
        # Calculate reward
        reward = self._calculate_reward(action)
        
        # Check if done
        terminated = False
        truncated = self.steps >= self.max_steps
        
        # Info
        info = {
            "steps": self.steps,
            "target_position": self.target_position.tolist()
        }
        
        return observation, reward, terminated, truncated, info
        
    def render(self) -> None:
        """Render the environment."""
        self.robot_env.render()
        
    def close(self) -> None:
        """Close the environment."""
        self.robot_env.close()
        
    def _get_observation(self) -> Dict[str, np.ndarray]:
        """Get the current observation.
        
        Returns:
            The current observation.
        """
        joint_positions = self.robot.controller.get_joint_positions()
        joint_velocities = self.robot.controller.get_joint_velocities()
        
        return {
            "joint_positions": joint_positions,
            "joint_velocities": joint_velocities,
            "target_position": self.target_position
        }
        
    def _calculate_reward(self, action: np.ndarray) -> float:
        """Calculate the reward.
        
        Args:
            action: The action taken.
            
        Returns:
            The reward.
        """
        # Get current joint positions
        joint_positions = self.robot.controller.get_joint_positions()
        
        # Calculate position error
        position_error = np.linalg.norm(self.target_position - joint_positions)
        
        # Calculate action magnitude (for penalizing large actions)
        action_magnitude = np.linalg.norm(action)
        
        # Reward for reaching target (negative error) and penalty for large actions
        reward = -position_error - 0.01 * action_magnitude
        
        return reward
        
    def sample_target_position(self) -> np.ndarray:
        """Sample a random target position.
        
        Returns:
            The sampled target position.
        """
        return np.random.uniform(
            self.joint_limits["lower"],
            self.joint_limits["upper"]
        )


class EndEffectorPositionEnv(RobotArmEnv):
    """Environment for end-effector position control."""
    
    def __init__(
        self,
        robot_type: str = "ur10",
        headless: bool = False,
        time_step: float = 1.0 / 60.0
    ):
        """Initialize the end-effector position control environment.
        
        Args:
            robot_type: The type of robot to use ("ur10" or "franka").
            headless: Whether to run in headless mode.
            time_step: The simulation time step.
        """
        super().__init__()
        
        # Create robot environment
        self.robot_env = IsaacSimRobotEnv(
            robot_usd_path=f"/Isaac/Robots/{robot_type.upper()}/{robot_type.lower()}.usd",
            robot_name=robot_type.lower(),
            headless=headless,
            time_step=time_step
        )
        
        # Create robot
        if robot_type.lower() == "ur10":
            self.robot = UR10Robot(robot_env=self.robot_env)
            self.num_joints = 6
        elif robot_type.lower() == "franka":
            self.robot = FrankaRobot(robot_env=self.robot_env)
            self.num_joints = 7
        else:
            raise ValueError(f"Unsupported robot type: {robot_type}")
        
        # Get joint limits
        self.joint_limits = self.robot.get_joint_limits()
        
        # Define action space (3D position delta)
        self.action_space = gymnasium.spaces.Box(
            low=-0.1 * np.ones(3),  # 10 cm per step in each direction
            high=0.1 * np.ones(3),
            dtype=np.float32
        )
        
        # Define observation space
        self.observation_space = gymnasium.spaces.Dict({
            "joint_positions": gymnasium.spaces.Box(
                low=self.joint_limits["lower"],
                high=self.joint_limits["upper"],
                dtype=np.float32
            ),
            "joint_velocities": gymnasium.spaces.Box(
                low=-np.ones(self.num_joints) * 10.0,
                high=np.ones(self.num_joints) * 10.0,
                dtype=np.float32
            ),
            "end_effector_position": gymnasium.spaces.Box(
                low=-np.ones(3) * 2.0,  # 2m in each direction
                high=np.ones(3) * 2.0,
                dtype=np.float32
            ),
            "target_position": gymnasium.spaces.Box(
                low=-np.ones(3) * 2.0,
                high=np.ones(3) * 2.0,
                dtype=np.float32
            )
        })
        
        # Initialize state
        self.target_position = np.array([0.5, 0.0, 0.5])  # Default target position
        self.steps = 0
        self.max_steps = 1000
        
    def reset(self) -> Dict[str, np.ndarray]:
        """Reset the environment.
        
        Returns:
            The initial observation.
        """
        # Reset simulation
        self.robot_env.reset()
        
        # Move robot to home position
        self.robot.move_to_home()
        
        # Set random target position
        self.target_position = self.sample_target_position()
        
        # Reset steps
        self.steps = 0
        
        # Get observation
        return self._get_observation()
        
    def step(self, action: np.ndarray) -> Tuple[Dict[str, np.ndarray], float, bool, bool, Dict[str, Any]]:
        """Take a step in the environment.
        
        Args:
            action: The action to take (delta end-effector position).
            
        Returns:
            A tuple of (observation, reward, terminated, truncated, info).
        """
        # Get current end-effector position
        current_position = self.robot_env.get_end_effector_position()
        
        # Calculate target end-effector position
        target_position = current_position + action
        
        # Move to target position (this would require inverse kinematics in a real implementation)
        # For now, this is a placeholder
        # In a real implementation, you would:
        # 1. Use inverse kinematics to convert target end-effector position to joint positions
        # 2. Use the controller to move to those joint positions
        
        # Step simulation
        self.robot_env.step()
        
        # Increment steps
        self.steps += 1
        
        # Get observation
        observation = self._get_observation()
        
        # Calculate reward
        reward = self._calculate_reward(action)
        
        # Check if done
        position_error = np.linalg.norm(self.target_position - self.robot_env.get_end_effector_position())
        terminated = position_error < 0.01  # 1 cm tolerance
        truncated = self.steps >= self.max_steps
        
        # Info
        info = {
            "steps": self.steps,
            "target_position": self.target_position.tolist(),
            "position_error": float(position_error)
        }
        
        return observation, reward, terminated, truncated, info
        
    def render(self) -> None:
        """Render the environment."""
        self.robot_env.render()
        
    def close(self) -> None:
        """Close the environment."""
        self.robot_env.close()
        
    def _get_observation(self) -> Dict[str, np.ndarray]:
        """Get the current observation.
        
        Returns:
            The current observation.
        """
        joint_positions = self.robot.controller.get_joint_positions()
        joint_velocities = self.robot.controller.get_joint_velocities()
        end_effector_position = self.robot_env.get_end_effector_position()
        
        return {
            "joint_positions": joint_positions,
            "joint_velocities": joint_velocities,
            "end_effector_position": end_effector_position,
            "target_position": self.target_position
        }
        
    def _calculate_reward(self, action: np.ndarray) -> float:
        """Calculate the reward.
        
        Args:
            action: The action taken.
            
        Returns:
            The reward.
        """
        # Get current end-effector position
        end_effector_position = self.robot_env.get_end_effector_position()
        
        # Calculate position error
        position_error = np.linalg.norm(self.target_position - end_effector_position)
        
        # Calculate action magnitude (for penalizing large actions)
        action_magnitude = np.linalg.norm(action)
        
        # Reward for reaching target (negative error) and penalty for large actions
        reward = -position_error - 0.1 * action_magnitude
        
        return reward
        
    def sample_target_position(self) -> np.ndarray:
        """Sample a random target position within the robot's workspace.
        
        Returns:
            The sampled target position.
        """
        # This would depend on the specific robot and its workspace
        # For now, we'll use a simple heuristic
        return np.array([
            np.random.uniform(0.3, 0.7),    # x: 30-70 cm in front of the robot
            np.random.uniform(-0.5, 0.5),   # y: 50 cm to the left and right
            np.random.uniform(0.2, 0.8)     # z: 20-80 cm above the table
        ])