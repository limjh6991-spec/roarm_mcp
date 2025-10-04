"""Reinforcement learning environments for RoArm MCP.

This module provides `gymnasium.Env`-style environments for controlling robot
arms in the Isaac Sim simulator. It includes a base class `RobotArmEnv` and
concrete implementations for different control tasks, such as joint position
control and end-effector position control.
"""

import logging
import gymnasium as gym
import numpy as np
from typing import Any, Dict, List, Optional, Tuple, Union

from roarm_mcp.isaac_sim.simulator import IsaacSimRobotEnv
from roarm_mcp.robot.arms import UR10Robot, FrankaRobot
from roarm_mcp.robot.controller import PositionController
from roarm_mcp.mcp.protocol import MCPSpace, SpaceType

# Initialize logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class RobotArmEnv(gym.Env):
    """Abstract base class for robot arm reinforcement learning environments.

    This class defines the common interface for all robot arm environments,
    including methods for resetting, stepping, rendering, and closing the
    environment. It also provides helper methods for converting `gymnasium.spaces`
    to the serializable `MCPSpace` format for communication over the MCP protocol.

    Attributes:
        observation_space (Optional[gym.Space]): The observation space.
        action_space (Optional[gym.Space]): The action space.
    """
    
    def __init__(self):
        """Initializes the robot arm environment."""
        super().__init__()
        self.observation_space = None
        self.action_space = None
        
    def reset(self, *, seed: Optional[int] = None, options: Optional[Dict[str, Any]] = None) -> Tuple[np.ndarray, Dict[str, Any]]:
        """Resets the environment to an initial state.

        Args:
            seed (Optional[int]): The seed that is used to initialize the
                environment's random number generator.
            options (Optional[Dict[str, Any]]): Additional options for resetting
                the environment.

        Returns:
            Tuple[np.ndarray, Dict[str, Any]]: A tuple containing the initial
            observation and an info dictionary.

        Raises:
            NotImplementedError: If the method is not implemented by a subclass.
        """
        super().reset(seed=seed)
        raise NotImplementedError
        
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        """Runs one timestep of the environment's dynamics.

        Args:
            action (np.ndarray): An action provided by the agent.

        Returns:
            Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]: A tuple of
            (observation, reward, terminated, truncated, info).

        Raises:
            NotImplementedError: If the method is not implemented by a subclass.
        """
        raise NotImplementedError
        
    def render(self) -> None:
        """Renders the environment.
        
        Raises:
            NotImplementedError: If the method is not implemented by a subclass.
        """
        raise NotImplementedError
        
    def close(self) -> None:
        """Performs any necessary cleanup.
        
        Raises:
            NotImplementedError: If the method is not implemented by a subclass.
        """
        raise NotImplementedError

    def _gym_space_to_mcp_space(self, space: gym.Space) -> MCPSpace:
        """Converts a `gymnasium.spaces.Space` object to an `MCPSpace` object.

        This is a helper function to serialize space definitions for the MCP protocol.

        Args:
            space (gym.Space): The Gymnasium space to convert.

        Returns:
            MCPSpace: The serializable MCPSpace representation.

        Raises:
            ValueError: If the space type is not supported.
        """
        if isinstance(space, gym.spaces.Box):
            return MCPSpace(
                type=SpaceType.BOX,
                shape=list(space.shape),
                low=space.low,
                high=space.high
            )
        elif isinstance(space, gym.spaces.Discrete):
            return MCPSpace(type=SpaceType.DISCRETE, n=space.n)
        elif isinstance(space, gym.spaces.MultiDiscrete):
            return MCPSpace(type=SpaceType.MULTI_DISCRETE, nvec=space.nvec)
        elif isinstance(space, gym.spaces.MultiBinary):
            return MCPSpace(type=SpaceType.MULTI_BINARY, shape=list(space.shape))
        elif isinstance(space, gym.spaces.Dict):
            return MCPSpace(
                type=SpaceType.DICT,
                spaces={k: self._gym_space_to_mcp_space(v) for k, v in space.spaces.items()}
            )
        else:
            raise ValueError(f"Unsupported space type: {type(space)}")

    def get_action_space(self) -> MCPSpace:
        """Gets the serializable action space of the environment.

        Returns:
            MCPSpace: The `MCPSpace` representation of the action space.
        """
        return self._gym_space_to_mcp_space(self.action_space)
        
    def get_observation_space(self) -> MCPSpace:
        """Gets the serializable observation space of the environment.

        Returns:
            MCPSpace: The `MCPSpace` representation of the observation space.
        """
        return self._gym_space_to_mcp_space(self.observation_space)


class JointPositionEnv(RobotArmEnv):
    """An environment for joint-space position control.

    In this environment, the agent's task is to provide target joint positions
    to move the robot arm to a randomly sampled target joint configuration.

    Attributes:
        robot_env (IsaacSimRobotEnv): The underlying Isaac Sim environment.
        robot (Union[UR10Robot, FrankaRobot]): The robot instance.
        num_joints (int): The number of joints in the robot arm.
        joint_limits (Dict[str, np.ndarray]): The robot's joint limits.
        target_position (np.ndarray): The target joint configuration for the
            current episode.
        steps (int): The number of steps taken in the current episode.
        max_steps (int): The maximum number of steps per episode.
    """
    
    def __init__(
        self,
        robot_type: str = "ur10",
        headless: bool = False,
        time_step: float = 1.0 / 60.0
    ):
        """Initializes the joint position control environment.

        Args:
            robot_type (str): The type of robot to use ("ur10" or "franka").
                Defaults to "ur10".
            headless (bool): Whether to run Isaac Sim in headless mode.
                Defaults to False.
            time_step (float): The simulation time step. Defaults to 1.0/60.0.
        """
        super().__init__()
        
        # Create robot environment
        self.robot_env = IsaacSimRobotEnv(
            headless=headless,
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
        self.action_space = gym.spaces.Box(
            low=self.joint_limits["lower"],
            high=self.joint_limits["upper"],
            dtype=np.float32
        )
        
        obs_low = np.concatenate([self.joint_limits["lower"], -np.ones(self.num_joints) * 10.0])
        obs_high = np.concatenate([self.joint_limits["upper"], np.ones(self.num_joints) * 10.0])
        self.observation_space = gym.spaces.Box(low=obs_low, high=obs_high, dtype=np.float32)

        # Initialize state
        self.target_position = self.robot.get_home_position()
        self.steps = 0
        self.max_steps = 1000
        
    def reset(self, *, seed: Optional[int] = None, options: Optional[Dict[str, Any]] = None) -> Tuple[np.ndarray, Dict[str, Any]]:
        """Resets the environment for a new episode."""
        super().reset(seed=seed)
        self.robot_env.reset()
        self.robot.move_to_home()
        self.target_position = self.action_space.sample()
        self.steps = 0
        
        observation = self._get_observation()
        info = {"target_position": self.target_position}
        return observation, info
        
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        """Takes a step in the environment."""
        action = np.clip(action, self.action_space.low, self.action_space.high)
        self.robot.controller.move_to_joint_positions(action)
        self.robot_env.step()
        self.steps += 1
        
        observation = self._get_observation()
        reward = self._calculate_reward()
        terminated = np.linalg.norm(self.target_position - observation[:self.num_joints]) < 0.05
        truncated = self.steps >= self.max_steps
        info = {"target_position": self.target_position}
        
        return observation, reward, terminated, truncated, info
        
    def render(self) -> None:
        """Renders the environment."""
        self.robot_env.render()
        
    def close(self) -> None:
        """Closes the environment."""
        self.robot_env.close()
        
    def _get_observation(self) -> np.ndarray:
        """Gets the current observation."""
        joint_positions = self.robot.controller.get_joint_positions()
        joint_velocities = self.robot.controller.get_joint_velocities()
        return np.concatenate([joint_positions, joint_velocities])
        
    def _calculate_reward(self) -> float:
        """Calculates the reward."""
        joint_positions = self.robot.controller.get_joint_positions()
        position_error = np.linalg.norm(self.target_position - joint_positions)
        return -position_error


class EndEffectorPositionEnv(RobotArmEnv):
    """An environment for task-space position control.

    In this environment, the agent's task is to provide target joint positions
    to move the robot's end-effector to a randomly sampled Cartesian coordinate.

    Note: This environment requires a robust inverse kinematics (IK) solver,
    which is currently a placeholder.

    Attributes:
        robot_env (IsaacSimRobotEnv): The underlying Isaac Sim environment.
        robot (Union[UR10Robot, FrankaRobot]): The robot instance.
        num_joints (int): The number of joints in the robot arm.
        target_position (np.ndarray): The target end-effector position for the
            current episode.
        steps (int): The number of steps taken in the current episode.
        max_steps (int): The maximum number of steps per episode.
    """
    
    def __init__(
        self,
        robot_type: str = "ur10",
        headless: bool = False,
        time_step: float = 1.0 / 60.0
    ):
        """Initializes the end-effector position control environment.

        Args:
            robot_type (str): The type of robot to use ("ur10" or "franka").
                Defaults to "ur10".
            headless (bool): Whether to run Isaac Sim in headless mode.
                Defaults to False.
            time_step (float): The simulation time step. Defaults to 1.0/60.0.
        """
        super().__init__()
        
        self.robot_env = IsaacSimRobotEnv(headless=headless)
        
        if robot_type.lower() == "ur10":
            self.robot = UR10Robot(robot_env=self.robot_env)
            self.num_joints = 6
        elif robot_type.lower() == "franka":
            self.robot = FrankaRobot(robot_env=self.robot_env)
            self.num_joints = 7
        else:
            raise ValueError(f"Unsupported robot type: {robot_type}")
        
        # Action space is target joint positions
        self.action_space = gym.spaces.Box(
            low=self.robot.get_joint_limits()["lower"],
            high=self.robot.get_joint_limits()["upper"],
            dtype=np.float32
        )
        
        # Observation space
        self.observation_space = gym.spaces.Dict({
            "joint_positions": gym.spaces.Box(low=-np.inf, high=np.inf, shape=(self.num_joints,), dtype=np.float32),
            "end_effector_position": gym.spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32),
            "target_position": gym.spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32),
        })
        
        self.target_position = np.array([0.5, 0.0, 0.5])
        self.steps = 0
        self.max_steps = 1000
        
    def reset(self, *, seed: Optional[int] = None, options: Optional[Dict[str, Any]] = None) -> Tuple[Dict[str, np.ndarray], Dict[str, Any]]:
        """Resets the environment for a new episode."""
        super().reset(seed=seed)
        self.robot_env.reset()
        self.robot.move_to_home()
        self.target_position = self.observation_space["target_position"].sample()
        self.steps = 0
        
        observation = self._get_observation()
        info = {"target_position": self.target_position}
        return observation, info
        
    def step(self, action: np.ndarray) -> Tuple[Dict[str, np.ndarray], float, bool, bool, Dict[str, Any]]:
        """Takes a step in the environment."""
        # Here, the action is the target joint positions.
        # A more advanced agent would use an IK solver to find the joint positions
        # that lead to a desired end-effector position.
        action = np.clip(action, self.action_space.low, self.action_space.high)
        self.robot.controller.move_to_joint_positions(action)
        self.robot_env.step()
        self.steps += 1
        
        observation = self._get_observation()
        reward = self._calculate_reward()
        position_error = np.linalg.norm(self.target_position - observation["end_effector_position"])
        terminated = position_error < 0.02
        truncated = self.steps >= self.max_steps
        info = {"target_position": self.target_position, "position_error": position_error}
        
        return observation, reward, terminated, truncated, info
        
    def render(self) -> None:
        """Renders the environment."""
        self.robot_env.render()
        
    def close(self) -> None:
        """Closes the environment."""
        self.robot_env.close()
        
    def _get_observation(self) -> Dict[str, np.ndarray]:
        """Gets the current observation."""
        return {
            "joint_positions": self.robot.controller.get_joint_positions(),
            "end_effector_position": self.robot_env.get_end_effector_position(),
            "target_position": self.target_position,
        }
        
    def _calculate_reward(self) -> float:
        """Calculates the reward."""
        end_effector_pos = self.robot_env.get_end_effector_position()
        position_error = np.linalg.norm(self.target_position - end_effector_pos)
        return -position_error