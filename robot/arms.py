"""
Specific robot arm implementations for RoArm MCP.

This module provides specific robot arm implementations for different models.
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
    """UR10 robot arm implementation."""
    
    def __init__(
        self,
        robot_env: Optional[IsaacSimRobotEnv] = None,
        usd_path: str = "/Isaac/Robots/UR10/ur10.usd",
        robot_name: str = "ur10"
    ):
        """Initialize the UR10 robot arm.
        
        Args:
            robot_env: The robot environment. If None, a new environment will be created.
            usd_path: The path to the USD file for the UR10 robot.
            robot_name: The name of the robot.
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
        """Get the home position of the robot.
        
        Returns:
            The home position as a numpy array.
        """
        return np.array([0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0])
    
    def move_to_home(self) -> None:
        """Move the robot to the home position."""
        home_position = self.get_home_position()
        self.controller.move_to_joint_positions(home_position)
        
    def get_joint_limits(self) -> Dict[str, np.ndarray]:
        """Get the joint limits of the robot.
        
        Returns:
            A dictionary with "lower" and "upper" joint limits.
        """
        return self.joint_limits
        
    def get_end_effector_link_name(self) -> str:
        """Get the name of the end-effector link.
        
        Returns:
            The name of the end-effector link.
        """
        return "tool0"


class FrankaRobot:
    """Franka Emika Panda robot arm implementation."""
    
    def __init__(
        self,
        robot_env: Optional[IsaacSimRobotEnv] = None,
        usd_path: str = "/Isaac/Robots/Franka/franka.usd",
        robot_name: str = "franka"
    ):
        """Initialize the Franka robot arm.
        
        Args:
            robot_env: The robot environment. If None, a new environment will be created.
            usd_path: The path to the USD file for the Franka robot.
            robot_name: The name of the robot.
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
        """Get the home position of the robot.
        
        Returns:
            The home position as a numpy array.
        """
        return np.array([0.0, -0.3, 0.0, -1.8, 0.0, 1.5, 0.0])
    
    def move_to_home(self) -> None:
        """Move the robot to the home position."""
        home_position = self.get_home_position()
        self.controller.move_to_joint_positions(home_position)
        
    def get_joint_limits(self) -> Dict[str, np.ndarray]:
        """Get the joint limits of the robot.
        
        Returns:
            A dictionary with "lower" and "upper" joint limits.
        """
        return self.joint_limits
        
    def get_end_effector_link_name(self) -> str:
        """Get the name of the end-effector link.
        
        Returns:
            The name of the end-effector link.
        """
        return "panda_hand"