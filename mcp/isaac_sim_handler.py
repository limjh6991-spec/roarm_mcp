"""
MCP Isaac Sim environment handler for RoArm.

이 모듈은 Isaac Sim 5.0 PhysX Tensors 솔루션을 MCP 프로토콜과 통합합니다.
"""

import asyncio
import logging
import numpy as np
import torch
from typing import Any, Dict, List, Optional, Tuple, Union

# Isaac Sim imports
import isaacsim
from isaacsim import SimulationApp

import sys
import os
sys.path.insert(0, '/home/roarm_m3/dev_roarm/roarm_mcp')

from mcp.server import MCPEnvironmentHandler
from mcp.protocol import MCPSpace

# Initialize logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class IsaacSimMCPHandler(MCPEnvironmentHandler):
    """MCP environment handler for Isaac Sim 5.0 PhysX Tensors integration."""
    
    def __init__(self, robot_type: str = "ur10", headless: bool = True):
        """Initialize the Isaac Sim MCP handler.
        
        Args:
            robot_type: Type of robot ("ur10" or "franka")
            headless: Whether to run Isaac Sim in headless mode
        """
        self.robot_type = robot_type
        self.headless = headless
        self.simulation_app = None
        self.world = None
        self.simulation_view = None
        self.articulation_views = {}
        self.robot_paths = {}
        self.is_initialized = False
        
    async def initialize(self) -> None:
        """Initialize Isaac Sim environment."""
        if self.is_initialized:
            logger.warning("Isaac Sim already initialized")
            return
            
        logger.info("🤖 Isaac Sim MCP Handler - 초기화 중...")
        
        # Run initialization in executor to avoid blocking
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(None, self._initialize_isaac_sim)
        
        self.is_initialized = True
        logger.info("✅ Isaac Sim MCP Handler 초기화 완료")
        
    def _initialize_isaac_sim(self) -> None:
        """Initialize Isaac Sim in sync context."""
        # Create simulation app
        config = {"headless": self.headless}
        self.simulation_app = SimulationApp(config)
        
        # Import Isaac Sim modules after app creation
        from isaacsim.core.api.world import World
        import omni.physics.tensors as tensors
        from pxr import Gf
        
        # Create world
        self.world = World(stage_units_in_meters=1.0)
        
        # Setup environment
        self._setup_environment()
        
        # Load robots
        self._load_robots()
        
        # Initialize physics
        self._initialize_physics()
        
    def _setup_environment(self) -> None:
        """Setup basic environment (lighting, ground plane)."""
        from pxr import UsdGeom, UsdLux, Gf
        
        stage = self.world.stage
        
        # Add lighting
        light_prim = UsdLux.DistantLight.Define(stage, "/World/DistantLight")
        light_prim.CreateIntensityAttr(500)
        
        # Add ground plane
        plane_geom = UsdGeom.Mesh.Define(stage, "/World/GroundPlane")
        plane_geom.CreatePointsAttr([
            (-2, -2, 0), (2, -2, 0), (2, 2, 0), (-2, 2, 0)
        ])
        plane_geom.CreateFaceVertexCountsAttr([4])
        plane_geom.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        
    def _load_robots(self) -> None:
        """Load robot models based on robot_type."""
        from pxr import Usd, UsdGeom
        
        stage = self.world.stage
        
        # Isaac Sim 5.0 cloud asset URLs
        base_url = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0"
        
        if self.robot_type == "ur10":
            robot_url = f"{base_url}/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
            robot_path = "/World/UR10"
        elif self.robot_type == "franka":
            robot_url = f"{base_url}/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
            robot_path = "/World/Franka"
        else:
            raise ValueError(f"Unsupported robot type: {self.robot_type}")
            
        # Create robot prim and add reference
        robot_prim = UsdGeom.Xform.Define(stage, robot_path)
        robot_prim.GetPrim().GetReferences().AddReference(robot_url)
        
        # Store robot path
        self.robot_paths[self.robot_type] = robot_path
        
        logger.info(f"📡 {self.robot_type.upper()} 로봇 로드: {robot_url}")
        
    def _initialize_physics(self) -> None:
        """Initialize PhysX Tensors and create ArticulationViews."""
        # Wait for USD references to load
        import time
        time.sleep(3)
        
        # Clean up nested ArticulationRoots
        self._cleanup_articulation_roots()
        
        # Reset world
        self.world.reset()
        
        # Update frames for initialization
        for _ in range(5):
            self.world.step(render=False)
            
        # Create SimulationView
        import omni.physics.tensors as tensors
        self.simulation_view = tensors.create_simulation_view("torch")
        self.simulation_view.set_subspace_roots("/World")
        
        # Create ArticulationView for the robot
        root_path = self._get_articulation_root_path()
        articulation_view = self.simulation_view.create_articulation_view(root_path)
        
        self.articulation_views[self.robot_type] = articulation_view
        
        # Initialize views
        for _ in range(3):
            self.world.step(render=False)
            
        logger.info(f"✅ {self.robot_type.upper()} ArticulationView 생성 완료")
        
    def _cleanup_articulation_roots(self) -> None:
        """Clean up nested ArticulationRoots following the solution pattern."""
        from pxr import UsdPhysics
        
        stage = self.world.stage
        robot_path = self.robot_paths[self.robot_type]
        
        # Find all ArticulationRoot prims under the robot
        articulation_roots = []
        
        def traverse_prims(prim_path):
            prim = stage.GetPrimAtPath(prim_path)
            if prim and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                articulation_roots.append(str(prim_path))
            
            for child in prim.GetChildren():
                traverse_prims(child.GetPath())
                
        traverse_prims(robot_path)
        
        # Keep only the first root, remove others
        if len(articulation_roots) > 1:
            primary_root = articulation_roots[0]
            for root_path in articulation_roots[1:]:
                prim = stage.GetPrimAtPath(root_path)
                if prim:
                    prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
                    logger.info(f"🔧 중첩 ArticulationRoot 제거: {root_path}")
                    
        logger.info(f"✅ ArticulationRoot 정리 완료: {articulation_roots[0] if articulation_roots else 'None'}")
        
    def _get_articulation_root_path(self) -> str:
        """Get the ArticulationRoot path for the robot."""
        from pxr import UsdPhysics
        
        stage = self.world.stage
        robot_path = self.robot_paths[self.robot_type]
        
        def find_articulation_root(prim_path):
            prim = stage.GetPrimAtPath(prim_path)
            if prim and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                return str(prim_path)
            
            for child in prim.GetChildren():
                result = find_articulation_root(child.GetPath())
                if result:
                    return result
            return None
            
        root_path = find_articulation_root(robot_path)
        if not root_path:
            raise RuntimeError(f"ArticulationRoot not found for {self.robot_type}")
            
        return root_path
        
    async def reset(self) -> np.ndarray:
        """Reset the environment.
        
        Returns:
            The initial observation (joint positions).
        """
        if not self.is_initialized:
            await self.initialize()
            
        # Run reset in executor
        loop = asyncio.get_event_loop()
        observation = await loop.run_in_executor(None, self._sync_reset)
        return observation
        
    def _sync_reset(self) -> np.ndarray:
        """Synchronous reset implementation."""
        # Reset world
        self.world.reset()
        
        # Step simulation
        for _ in range(5):
            self.world.step(render=False)
            
        # Get initial observation
        articulation_view = self.articulation_views[self.robot_type]
        positions = articulation_view.get_dof_positions()
        
        # Convert torch tensor to numpy
        if isinstance(positions, torch.Tensor):
            positions = positions.cpu().numpy()
            
        return positions.flatten()
        
    async def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        """Take a step in the environment.
        
        Args:
            action: The action to take (joint position targets).
            
        Returns:
            A tuple of (observation, reward, terminated, truncated, info).
        """
        if not self.is_initialized:
            await self.initialize()
            
        # Run step in executor
        loop = asyncio.get_event_loop()
        result = await loop.run_in_executor(None, self._sync_step, action)
        return result
        
    def _sync_step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        """Synchronous step implementation."""
        articulation_view = self.articulation_views[self.robot_type]
        
        # Convert action to torch tensor
        action_tensor = torch.from_numpy(action).float()
        if action_tensor.dim() == 1:
            action_tensor = action_tensor.unsqueeze(0)  # Add batch dimension
            
        # Create indices for Isaac Sim 5.0 API
        indices = torch.arange(articulation_view.count, dtype=torch.int32)
        
        # Apply action
        articulation_view.set_dof_position_targets(action_tensor, indices=indices)
        
        # Step simulation
        self.world.step(render=False)
        
        # Get new observation
        positions = articulation_view.get_dof_positions()
        velocities = articulation_view.get_dof_velocities()
        
        # Convert to numpy
        if isinstance(positions, torch.Tensor):
            positions = positions.cpu().numpy()
        if isinstance(velocities, torch.Tensor):
            velocities = velocities.cpu().numpy()
            
        observation = positions.flatten()
        
        # Simple reward (negative distance to target)
        reward = -np.linalg.norm(observation - action)
        
        # Episode management (simple time-based)
        terminated = False
        truncated = False
        
        info = {
            "positions": positions.tolist(),
            "velocities": velocities.flatten().tolist(),
            "robot_type": self.robot_type
        }
        
        return observation, reward, terminated, truncated, info
        
    async def render(self) -> Optional[np.ndarray]:
        """Render the environment (Isaac Sim handles rendering internally)."""
        return None
        
    async def close(self) -> None:
        """Close the environment."""
        if self.simulation_app:
            loop = asyncio.get_event_loop()
            await loop.run_in_executor(None, self._sync_close)
            
    def _sync_close(self) -> None:
        """Synchronous close implementation."""
        if self.simulation_app:
            self.simulation_app.close()
            self.simulation_app = None
            logger.info("✅ Isaac Sim 환경 종료 완료")
            
    async def get_action_space(self) -> MCPSpace:
        """Get the action space definition."""
        if not self.is_initialized:
            await self.initialize()
            
        articulation_view = self.articulation_views[self.robot_type]
        dof_count = articulation_view.get_dof_positions().shape[-1]
        
        return MCPSpace(
            type="box",
            low=[-3.14] * dof_count,  # Joint limits (simplified)
            high=[3.14] * dof_count,
            shape=[dof_count],
            dtype="float32"
        )
        
    async def get_observation_space(self) -> MCPSpace:
        """Get the observation space definition."""
        if not self.is_initialized:
            await self.initialize()
            
        articulation_view = self.articulation_views[self.robot_type]
        dof_count = articulation_view.get_dof_positions().shape[-1]
        
        return MCPSpace(
            type="box",
            low=[-3.14] * dof_count,  # Joint position limits (simplified)
            high=[3.14] * dof_count,
            shape=[dof_count],
            dtype="float32"
        )