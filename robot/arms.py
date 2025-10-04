"""Specific robot arm implementations for RoArm MCP.

This module provides concrete implementations for various robot arm models,
such as the UR10 and Franka Emika Panda. Each class encapsulates the robot's
specific properties, such as its USD path, joint limits, and home position.
These classes interface with the `IsaacSimRobotEnv` for simulation and use a
`PositionController` for motion control.
"""

import os
import logging
import numpy as np
from typing import Any, Dict, List, Optional, Tuple, Union

from roarm_mcp.isaac_sim.simulator import IsaacSimRobotEnv
from roarm_mcp.robot.controller import RobotArmController, PositionController

# Initialize logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class UR10Robot:
    """Represents the UR10 robot arm.

    This class encapsulates the configuration and control for the Universal
    Robots UR10 model. It defines the robot's USD path, name, joint limits,
    and provides methods for controlling it within the Isaac Sim environment.

    Attributes:
        usd_path (str): The path to the robot's USD file for simulation.
        robot_name (str): The name assigned to the robot in the simulation.
        robot_env (IsaacSimRobotEnv): The simulation environment for the robot.
        controller (PositionController): The controller for executing joint
            position commands.
        joint_limits (Dict[str, np.ndarray]): A dictionary containing the
            lower and upper joint limits.
    """
    
    def __init__(
        self,
        robot_env: Optional[IsaacSimRobotEnv] = None,
        usd_path: str = "/Isaac/Robots/UR10/ur10.usd",
        robot_name: str = "ur10"
    ):
        """Initializes the UR10 robot arm.

        Args:
            robot_env (Optional[IsaacSimRobotEnv]): An existing robot
                environment. If None, a new one is created. Defaults to None.
            usd_path (str): The path to the USD file for the UR10 robot.
                Defaults to "/Isaac/Robots/UR10/ur10.usd".
            robot_name (str): The name of the robot in the simulation.
                Defaults to "ur10".
        """
        self.usd_path = usd_path
        self.robot_name = robot_name
        
        # Create environment if not provided
        if robot_env is None:
            self.robot_env = IsaacSimRobotEnv(
                robot_usd_path=self.usd_path,
                robot_name=self.robot_name
            )
        else:
            self.robot_env = robot_env
            
        # Create controller
        self.controller = PositionController(self.robot_env)
        
        # Define joint limits
        self.joint_limits = {
            "lower": np.array([-np.pi, -np.pi, -np.pi, -np.pi, -np.pi, -np.pi]),
            "upper": np.array([np.pi, np.pi, np.pi, np.pi, np.pi, np.pi])
        }
        
    def get_home_position(self) -> np.ndarray:
        """Gets the predefined home joint positions for the UR10 robot.

        Returns:
            np.ndarray: The home position as a NumPy array of joint angles
            in radians.
        """
        return np.array([0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0])
    
    def move_to_home(self) -> None:
        """Moves the robot to its predefined home position."""
        home_position = self.get_home_position()
        self.controller.move_to_joint_positions(home_position)
        
    def get_joint_limits(self) -> Dict[str, np.ndarray]:
        """Gets the joint limits of the robot.

        Returns:
            Dict[str, np.ndarray]: A dictionary with "lower" and "upper"
            keys, each mapping to a NumPy array of joint limits in radians.
        """
        return self.joint_limits
        
    def get_end_effector_link_name(self) -> str:
        """Gets the name of the end-effector link for this robot.

        This is used to identify the correct link for end-effector position
        control.

        Returns:
            str: The name of the end-effector link ("tool0").
        """
        return "tool0"


class FrankaRobot:
    """Represents the Franka Emika Panda robot arm.

    This class encapsulates the configuration and control for the Franka Emika
    Panda model. It defines the robot's USD path, name, joint limits, and
    provides methods for controlling it within the Isaac Sim environment.

    Attributes:
        usd_path (str): The path to the robot's USD file for simulation.
        robot_name (str): The name assigned to the robot in the simulation.
        robot_env (IsaacSimRobotEnv): The simulation environment for the robot.
        controller (PositionController): The controller for executing joint
            position commands.
        joint_limits (Dict[str, np.ndarray]): A dictionary containing the
            lower and upper joint limits.
    """
    
    def __init__(
        self,
        robot_env: Optional[IsaacSimRobotEnv] = None,
        usd_path: str = "/Isaac/Robots/Franka/franka.usd",
        robot_name: str = "franka"
    ):
        """Initializes the Franka robot arm.

        Args:
            robot_env (Optional[IsaacSimRobotEnv]): An existing robot
                environment. If None, a new one is created. Defaults to None.
            usd_path (str): The path to the USD file for the Franka robot.
                Defaults to "/Isaac/Robots/Franka/franka.usd".
            robot_name (str): The name of the robot in the simulation.
                Defaults to "franka".
        """
        self.usd_path = usd_path
        self.robot_name = robot_name
        
        # Create environment if not provided
        if robot_env is None:
            self.robot_env = IsaacSimRobotEnv(
                robot_usd_path=self.usd_path,
                robot_name=self.robot_name
            )
        else:
            self.robot_env = robot_env
            
        # Create controller
        self.controller = PositionController(self.robot_env)
        
        # Define joint limits
        self.joint_limits = {
            "lower": np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]),
            "upper": np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
        }
        
    def get_home_position(self) -> np.ndarray:
        """Gets the predefined home joint positions for the Franka robot.

        Returns:
            np.ndarray: The home position as a NumPy array of joint angles
            in radians.
        """
        return np.array([0.0, -0.3, 0.0, -1.8, 0.0, 1.5, 0.0])
    
    def move_to_home(self) -> None:
        """Moves the robot to its predefined home position."""
        home_position = self.get_home_position()
        self.controller.move_to_joint_positions(home_position)
        
    def get_joint_limits(self) -> Dict[str, np.ndarray]:
        """Gets the joint limits of the robot.

        Returns:
            Dict[str, np.ndarray]: A dictionary with "lower" and "upper"
            keys, each mapping to a NumPy array of joint limits in radians.
        """
        return self.joint_limits
        
    def get_end_effector_link_name(self) -> str:
        """Gets the name of the end-effector link for this robot.

        This is used to identify the correct link for end-effector position
        control.

        Returns:
            str: The name of the end-effector link ("panda_hand").
        """
        return "panda_hand"