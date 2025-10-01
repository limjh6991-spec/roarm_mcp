"""
Isaac Sim integration for RoArm MCP.

This module provides the integration with NVIDIA Isaac Sim for robot arm control.
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
    from omni.isaac.core.objects import DynamicCuboid
    from omni.isaac.core.utils.extensions import enable_extension
    from omni.isaac.core.utils.physics import ArticulationView
    from omni.isaac.core.prims import RigidPrimView, XFormPrimView
    from pxr import Gf, UsdGeom, Sdf

    ISAAC_SIM_AVAILABLE = True
    logger.info("Isaac Sim modules imported successfully")
except ImportError as e:
    logger.warning(f"Failed to import Isaac Sim modules: {e}")
    logger.warning("Isaac Sim functionality will not be available")


class IsaacSimEnv:
    """Base class for Isaac Sim environments."""
    
    def __init__(
        self,
        sim_params: Optional[Dict[str, Any]] = None,
        headless: bool = False,
        time_step: float = 1.0 / 60.0,
        physics_steps_per_frame: int = 1
    ):
        """Initialize the Isaac Sim environment.
        
        Args:
            sim_params: Simulation parameters.
            headless: Whether to run in headless mode.
            time_step: The simulation time step.
            physics_steps_per_frame: The number of physics steps per frame.
        """
        if not ISAAC_SIM_AVAILABLE:
            raise ImportError("Isaac Sim modules are not available")
            
        self.sim_params = sim_params or {}
        self.headless = headless
        self.time_step = time_step
        self.physics_steps_per_frame = physics_steps_per_frame
        
        # Isaac Sim components
        self.simulation_context = None
        self.world = None
        
        # Initialize the simulation
        self._initialize_sim()
        
    def _initialize_sim(self) -> None:
        """Initialize the Isaac Sim simulation."""
        # Enable required extensions
        self._enable_extensions()
        
        # Create simulation context
        self.simulation_context = SimulationContext(
            stage_units_in_meters=1.0,
            physics_dt=self.time_step,
            rendering_dt=self.time_step,
            backend="torch",
            execution_type="async",
            sim_params=self.sim_params
        )
        
        # Create world
        self.world = World(
            stage_units_in_meters=1.0,
            physics_dt=self.time_step,
            rendering_dt=self.time_step
        )
        
        # Reset the world to initialize physics
        self.world.reset()
        
        logger.info("Isaac Sim environment initialized")
        
    def _enable_extensions(self) -> None:
        """Enable required extensions."""
        extensions = [
            "omni.isaac.ros_bridge",
            "omni.isaac.core",
            "omni.isaac.py_rep",
            "omni.replicator.isaac"
        ]
        
        for extension in extensions:
            try:
                enable_extension(extension)
                logger.info(f"Enabled extension: {extension}")
            except Exception as e:
                logger.warning(f"Failed to enable extension {extension}: {e}")
    
    def reset(self) -> None:
        """Reset the simulation."""
        if self.world is not None:
            self.world.reset()
            logger.info("Simulation reset")
        else:
            logger.warning("Cannot reset: world not initialized")
    
    def step(self) -> None:
        """Step the simulation."""
        if self.world is not None:
            self.world.step(render=not self.headless)
        else:
            logger.warning("Cannot step: world not initialized")
    
    def close(self) -> None:
        """Close the simulation."""
        if self.simulation_context is not None:
            self.simulation_context.stop()
            logger.info("Simulation stopped")
    
    def render(self) -> None:
        """Render the simulation."""
        if self.headless:
            logger.warning("Cannot render in headless mode")
            return
            
        if self.world is not None:
            self.world.render()
        else:
            logger.warning("Cannot render: world not initialized")
    
    def load_stage(self, stage_path: str) -> bool:
        """Load a USD stage.
        
        Args:
            stage_path: The path to the USD stage.
            
        Returns:
            True if the stage was loaded successfully, False otherwise.
        """
        try:
            # Open the USD stage
            stage_utils.open_stage(stage_path)
            logger.info(f"Loaded stage: {stage_path}")
            
            # Reset the world to initialize physics with the new stage
            self.world.reset()
            
            return True
        except Exception as e:
            logger.error(f"Failed to load stage {stage_path}: {e}")
            return False
    
    def add_ground_plane(
        self,
        name: str = "ground_plane",
        size: float = 10.0,
        color: Tuple[float, float, float] = (0.3, 0.3, 0.3)
    ) -> None:
        """Add a ground plane to the simulation.
        
        Args:
            name: The name of the ground plane.
            size: The size of the ground plane.
            color: The color of the ground plane.
        """
        # Create a ground plane using USD
        stage = omni.usd.get_context().get_stage()
        
        # Create ground plane
        plane = UsdGeom.Xform.Define(stage, Sdf.Path(f"/{name}"))
        plane_mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(f"/{name}/mesh"))
        
        # Set plane properties
        vertices = np.array([
            [-size, -size, 0],
            [size, -size, 0],
            [size, size, 0],
            [-size, size, 0]
        ])
        
        indices = np.array([0, 1, 2, 0, 2, 3], dtype=np.int32)
        
        plane_mesh.CreatePointsAttr().Set(vertices.tolist())
        plane_mesh.CreateFaceVertexIndicesAttr().Set(indices.tolist())
        plane_mesh.CreateFaceVertexCountsAttr().Set([3, 3])
        
        # Set color
        color_attr = UsdGeom.Primvar(plane_mesh.GetPrim().CreateAttribute(
            "primvars:displayColor", Sdf.ValueTypeNames.Color3f, True
        ))
        color_attr.SetInterpolation("constant")
        color_attr.Set([Gf.Vec3f(*color)])
        
        # Add collision
        collision_api = UsdPhysics.CollisionAPI.Apply(plane_mesh.GetPrim())
        
        logger.info(f"Added ground plane: {name}")


class IsaacSimRobotEnv(IsaacSimEnv):
    """Isaac Sim environment with robot support."""
    
    def __init__(
        self,
        robot_usd_path: str,
        robot_name: str = "robot",
        sim_params: Optional[Dict[str, Any]] = None,
        headless: bool = False,
        time_step: float = 1.0 / 60.0,
        physics_steps_per_frame: int = 1
    ):
        """Initialize the Isaac Sim robot environment.
        
        Args:
            robot_usd_path: The path to the robot USD file.
            robot_name: The name of the robot.
            sim_params: Simulation parameters.
            headless: Whether to run in headless mode.
            time_step: The simulation time step.
            physics_steps_per_frame: The number of physics steps per frame.
        """
        super().__init__(sim_params, headless, time_step, physics_steps_per_frame)
        
        self.robot_usd_path = robot_usd_path
        self.robot_name = robot_name
        self.robot = None
        self.robot_articulation = None
        
        # Load the robot
        self._load_robot()
        
    def _load_robot(self) -> None:
        """Load the robot."""
        try:
            # Check if the robot USD file exists
            if not os.path.exists(self.robot_usd_path):
                # Try to find it in the Nucleus server
                nucleus_server = nucleus_utils.get_nucleus_server()
                if nucleus_server:
                    usd_path = nucleus_utils.find_file(self.robot_usd_path, nucleus_server)
                    if usd_path:
                        self.robot_usd_path = usd_path
                    else:
                        logger.error(f"Could not find robot USD file: {self.robot_usd_path}")
                        return
                else:
                    logger.error(f"Robot USD file not found: {self.robot_usd_path}")
                    return
            
            # Add the robot to the scene
            stage = omni.usd.get_context().get_stage()
            robot_path = f"/{self.robot_name}"
            robot_prim = stage.DefinePrim(robot_path, "Xform")
            robot_prim.GetReferences().AddReference(self.robot_usd_path)
            
            # Reset the world to initialize physics
            self.world.reset()
            
            # Get the robot articulation
            self.robot_articulation = ArticulationView(
                prim_paths_expr=robot_path,
                name=self.robot_name
            )
            self.world.scene.add(self.robot_articulation)
            
            logger.info(f"Robot loaded: {self.robot_name} from {self.robot_usd_path}")
        except Exception as e:
            logger.error(f"Failed to load robot: {e}")
            
    def get_joint_positions(self) -> np.ndarray:
        """Get the current joint positions.
        
        Returns:
            The joint positions as a numpy array.
        """
        if self.robot_articulation is None:
            logger.warning("Cannot get joint positions: robot not loaded")
            return np.array([])
            
        return self.robot_articulation.get_joint_positions()
    
    def get_joint_velocities(self) -> np.ndarray:
        """Get the current joint velocities.
        
        Returns:
            The joint velocities as a numpy array.
        """
        if self.robot_articulation is None:
            logger.warning("Cannot get joint velocities: robot not loaded")
            return np.array([])
            
        return self.robot_articulation.get_joint_velocities()
    
    def set_joint_positions(self, positions: np.ndarray) -> None:
        """Set the joint positions.
        
        Args:
            positions: The joint positions to set.
        """
        if self.robot_articulation is None:
            logger.warning("Cannot set joint positions: robot not loaded")
            return
            
        self.robot_articulation.set_joint_positions(positions)
    
    def set_joint_velocities(self, velocities: np.ndarray) -> None:
        """Set the joint velocities.
        
        Args:
            velocities: The joint velocities to set.
        """
        if self.robot_articulation is None:
            logger.warning("Cannot set joint velocities: robot not loaded")
            return
            
        self.robot_articulation.set_joint_velocities(velocities)
    
    def apply_joint_efforts(self, efforts: np.ndarray) -> None:
        """Apply joint efforts.
        
        Args:
            efforts: The joint efforts to apply.
        """
        if self.robot_articulation is None:
            logger.warning("Cannot apply joint efforts: robot not loaded")
            return
            
        self.robot_articulation.apply_joint_efforts(efforts)
    
    def get_end_effector_position(self) -> np.ndarray:
        """Get the end-effector position.
        
        Returns:
            The end-effector position as a numpy array.
        """
        if self.robot_articulation is None:
            logger.warning("Cannot get end-effector position: robot not loaded")
            return np.array([0.0, 0.0, 0.0])
            
        # This would depend on the specific robot model and how the end-effector is defined
        # For now, we'll return a placeholder implementation
        # In a real implementation, you would get the position of the end-effector link
        return np.array([0.0, 0.0, 0.0])
    
    def get_end_effector_orientation(self) -> np.ndarray:
        """Get the end-effector orientation.
        
        Returns:
            The end-effector orientation as a numpy array (quaternion).
        """
        if self.robot_articulation is None:
            logger.warning("Cannot get end-effector orientation: robot not loaded")
            return np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion
            
        # This would depend on the specific robot model and how the end-effector is defined
        # For now, we'll return a placeholder implementation
        # In a real implementation, you would get the orientation of the end-effector link
        return np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion