"""Isaac Sim integration for RoArm MCP.

This module provides the core classes for integrating with NVIDIA Isaac Sim.
It defines a base simulation environment (`IsaacSimEnv`) and a specialized
environment for robot arm control (`IsaacSimRobotEnv`). These classes handle
the setup of the simulation context, loading of assets, and provide an API for
interacting with the simulated world and its contents.

Note: This module requires a running instance of NVIDIA Isaac Sim and the
necessary Python environment to be available.
"""

import os
import logging
import numpy as np
from typing import Any, Dict, List, Optional, Tuple, Union

# Initialize logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Flag to check if Isaac Sim is available
ISAAC_SIM_AVAILABLE = False

# Try to import Isaac Sim modules
try:
    import omni
    import omni.isaac.core.utils.nucleus as nucleus_utils
    import omni.isaac.core.utils.stage as stage_utils
    from omni.isaac.core import SimulationContext
    from omni.isaac.core.world import World
    from omni.isaac.core.articulations import ArticulationView
    from pxr import Gf, UsdGeom, Sdf, UsdPhysics

    ISAAC_SIM_AVAILABLE = True
    logger.info("Isaac Sim modules imported successfully")
except ImportError as e:
    logger.warning(f"Failed to import Isaac Sim modules: {e}. Isaac Sim functionality will be unavailable.")


class IsaacSimEnv:
    """Base class for managing an NVIDIA Isaac Sim environment.

    This class handles the fundamental setup of an Isaac Sim simulation,
    including initializing the simulation context, creating a world, and
    providing methods for stepping, resetting, and closing the simulation.

    Attributes:
        simulation_context (Optional[SimulationContext]): The main simulation
            context object from Isaac Sim.
        world (Optional[World]): The world object that contains all simulation
            elements.
        headless (bool): Flag indicating if the simulation is running in
            headless mode.
    """
    
    def __init__(
        self,
        headless: bool = False,
    ):
        """Initializes the Isaac Sim environment.

        Args:
            headless (bool): Whether to run the simulation in headless mode
                (without a GUI). Defaults to False.

        Raises:
            ImportError: If the required Isaac Sim modules are not available.
        """
        if not ISAAC_SIM_AVAILABLE:
            raise ImportError("Isaac Sim modules are not available. This class requires a running Isaac Sim instance.")
            
        self.headless = headless
        
        # Isaac Sim components
        self.simulation_context = SimulationContext(
            stage_units_in_meters=1.0,
            physics_dt=1.0 / 60.0,
            rendering_dt=1.0 / 60.0 if not headless else 0.0,
        )
        self.world = World(
            stage_units_in_meters=1.0,
            physics_dt=1.0 / 60.0,
            rendering_dt=1.0 / 60.0 if not headless else 0.0
        )
        
        self.world.reset()
        logger.info("Isaac Sim environment initialized.")
        
    def reset(self) -> None:
        """Resets the simulation world."""
        if self.world:
            self.world.reset()
            logger.info("Simulation world has been reset.")
        else:
            logger.warning("Cannot reset: world is not initialized.")
    
    def step(self) -> None:
        """Performs one simulation step."""
        if self.world:
            self.world.step(render=not self.headless)
        else:
            logger.warning("Cannot step: world is not initialized.")
    
    def close(self) -> None:
        """Stops the simulation and closes the Isaac Sim application."""
        if self.simulation_context:
            self.simulation_context.stop()
            logger.info("Simulation stopped.")
    
    def render(self) -> None:
        """Triggers a render of the current simulation frame.

        Note: This has no effect if the simulation is running in headless mode.
        """
        if self.headless:
            logger.warning("Cannot render in headless mode.")
            return
            
        if self.world:
            self.world.render()
        else:
            logger.warning("Cannot render: world is not initialized.")


class IsaacSimRobotEnv(IsaacSimEnv):
    """An Isaac Sim environment specialized for robot arm control.

    This class extends `IsaacSimEnv` to include functionality for loading a
    robot from a USD file and interacting with its articulation (joints).

    Attributes:
        robot_usd_path (str): The path to the robot's USD file.
        robot_name (str): The name given to the robot in the simulation stage.
        robot_articulation (Optional[ArticulationView]): The view for
            interacting with the robot's joints.
    """
    
    def __init__(
        self,
        robot_usd_path: str = "",
        robot_name: str = "robot",
        headless: bool = False,
    ):
        """Initializes the Isaac Sim robot environment.

        Args:
            robot_usd_path (str): The file path to the robot's USD model.
                Defaults to "".
            robot_name (str): The name to assign to the robot in the simulation.
                Defaults to "robot".
            headless (bool): Whether to run in headless mode. Defaults to False.
        """
        super().__init__(headless)
        
        self.robot_usd_path = robot_usd_path
        self.robot_name = robot_name
        self.robot_articulation = None
        
        if self.robot_usd_path:
            self._load_robot()
        
    def _load_robot(self) -> None:
        """Loads the robot from the specified USD path into the simulation."""
        try:
            stage = self.world.stage
            robot_prim_path = f"/{self.robot_name}"
            
            # Add the robot to the scene
            robot_prim = stage.DefinePrim(robot_prim_path, "Xform")
            robot_prim.GetReferences().AddReference(self.robot_usd_path)
            
            # Create an ArticulationView to control the robot
            self.robot_articulation = ArticulationView(
                prim_paths_expr=robot_prim_path,
                name=f"{self.robot_name}_view"
            )
            self.world.scene.add(self.robot_articulation)
            self.world.reset() # Reset to apply changes
            
            logger.info(f"Robot '{self.robot_name}' loaded from '{self.robot_usd_path}'.")
        except Exception as e:
            logger.error(f"Failed to load robot: {e}", exc_info=True)
            
    def get_joint_positions(self) -> np.ndarray:
        """Gets the current positions of the robot's joints.

        Returns:
            np.ndarray: A NumPy array of joint positions in radians. Returns an
            empty array if the robot is not loaded.
        """
        if self.robot_articulation:
            return self.robot_articulation.get_joint_positions()[0]
        logger.warning("Cannot get joint positions: robot not loaded.")
        return np.array([])
    
    def get_joint_velocities(self) -> np.ndarray:
        """Gets the current velocities of the robot's joints.

        Returns:
            np.ndarray: A NumPy array of joint velocities in rad/s. Returns an
            empty array if the robot is not loaded.
        """
        if self.robot_articulation:
            return self.robot_articulation.get_joint_velocities()[0]
        logger.warning("Cannot get joint velocities: robot not loaded.")
        return np.array([])
    
    def set_joint_positions(self, positions: np.ndarray) -> None:
        """Sets the target positions for the robot's joints.

        Args:
            positions (np.ndarray): The target joint positions in radians.
        """
        if self.robot_articulation:
            self.robot_articulation.set_joint_position_targets(positions)
        else:
            logger.warning("Cannot set joint positions: robot not loaded.")
    
    def set_joint_velocities(self, velocities: np.ndarray) -> None:
        """Sets the target velocities for the robot's joints.

        Args:
            velocities (np.ndarray): The target joint velocities in rad/s.
        """
        if self.robot_articulation:
            self.robot_articulation.set_joint_velocity_targets(velocities)
        else:
            logger.warning("Cannot set joint velocities: robot not loaded.")
    
    def apply_joint_efforts(self, efforts: np.ndarray) -> None:
        """Applies efforts (torques) to the robot's joints.

        Args:
            efforts (np.ndarray): The joint efforts to apply.
        """
        if self.robot_articulation:
            self.robot_articulation.set_joint_efforts(efforts)
        else:
            logger.warning("Cannot apply joint efforts: robot not loaded.")
    
    def get_end_effector_position(self) -> np.ndarray:
        """Gets the world position of the robot's end-effector.

        Note: This is a placeholder. A real implementation would need to
        identify the end-effector link from the robot's articulation.

        Returns:
            np.ndarray: The (x, y, z) position of the end-effector.
        """
        if self.robot_articulation:
            # Placeholder: This needs to be implemented based on the specific robot
            # and its end-effector link name.
            logger.warning("get_end_effector_position is a placeholder.")
            return np.array([0.0, 0.0, 0.0])
        logger.warning("Cannot get end-effector position: robot not loaded.")
        return np.array([0.0, 0.0, 0.0])
    
    def get_end_effector_orientation(self) -> np.ndarray:
        """Gets the world orientation of the robot's end-effector.

        Note: This is a placeholder. A real implementation would need to
        identify the end-effector link from the robot's articulation.

        Returns:
            np.ndarray: The orientation as a quaternion (w, x, y, z).
        """
        if self.robot_articulation:
            # Placeholder: This needs to be implemented based on the specific robot
            # and its end-effector link name.
            logger.warning("get_end_effector_orientation is a placeholder.")
            return np.array([1.0, 0.0, 0.0, 0.0])
        logger.warning("Cannot get end-effector orientation: robot not loaded.")
        return np.array([1.0, 0.0, 0.0, 0.0])