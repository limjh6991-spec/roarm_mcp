#!/usr/bin/env python3
"""Solution for setting up multi-robot simulations with PhysX Tensors in Isaac Sim.

This script demonstrates the correct procedure for loading multiple robots into
an Isaac Sim 5.0 environment, addressing common issues encountered with physics
and control. It serves as a reference implementation for:

A) Correctly handling nested Articulation Roots by ensuring only one root
   exists per robot's prim subtree.
B) Following the correct initialization order:
   1. Fix Articulation Roots.
   2. Call `world.reset()`.
   3. Allow for update frames.
   4. Create simulation views.
C) Using the `omni.physics.tensors.create_simulation_view()` and its
   `create_articulation_view()` method for robust physics-based control.
"""

import os
import sys
import time
from typing import Dict, Optional, List
import traceback

# Isaac Sim imports
try:
    from isaacsim import SimulationApp
except ImportError:
    print("Error: `isaacsim` module not found. Please ensure this script is run with Isaac Sim's Python environment.")
    sys.exit(1)

# Import torch for tensor operations
import torch

print("🤖 RoArm MCP - Isaac Sim 5.0 PhysX Tensors Solution")
print(f"Python Version: {sys.version}")
print(f"Working Directory: {os.getcwd()}")

def main():
    """Main function to set up and run the Isaac Sim multi-robot simulation."""
    print("✅ Initializing Isaac Sim 5.0 SimulationApp...")
    
    # Create simulation app (headless mode for server use)
    simulation_app = SimulationApp({
        "headless": True,
        "width": 1280,
        "height": 720
    })
    
    try:
        # Import Isaac Sim modules after app creation
        from pxr import Usd, UsdPhysics, UsdGeom, Gf, Sdf
        from isaacsim.core.world import World
        from isaacsim.core.utils.prims import get_prim_at_path, create_prim
        from isaacsim.core.utils.stage import get_current_stage
        import omni.physics.tensors as tensors
        import omni.kit.app
        
        print("✅ Isaac Sim 5.0 SimulationApp initialized successfully.")
        print("✅ Isaac Sim modules imported successfully.")
        
        # Check for Isaac Sim 5.0 cloud assets
        isaac_assets_base = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0"
        print(f"✅ Using Isaac Sim Assets from: {isaac_assets_base}")
        
        # Robot asset URLs for Isaac Sim 5.0
        robot_assets = {
            "UR10": f"{isaac_assets_base}/Isaac/Robots/UniversalRobots/ur10/ur10.usd",
            "Franka": f"{isaac_assets_base}/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        }
        
        print("🔧 Preparing robot assets...")
        for name, url in robot_assets.items():
            print(f"  📡 {name}: {url}")
        
        # Initialize World with physics
        print("🔧 Setting up Physics Scene...")
        stage = get_current_stage()
        my_world = World(stage_units_in_meters=1.0)
        print("✅ Isaac Sim World initialized successfully.")
        
        print("\n🌟 Setting up simulation environment...")
        create_prim(prim_path="/World/GroundPlane", prim_type="Cube", position=(0.0, 0.0, -0.5))
        print("  - Ground plane added.")
        
        # Load robot models with proper structure
        print("\n🤖 Loading robot models...")
        
        def ensure_offset_and_reference(prim_path, asset_url, position=(0, 0, 0)):
            """Creates an Xform prim and adds a USD reference to load a robot asset.

            Args:
                prim_path (str): The desired path for the robot prim.
                asset_url (str): The URL or path to the robot's USD file.
                position (tuple, optional): The (x, y, z) position to place the robot.
                                            Defaults to (0, 0, 0).

            Returns:
                Usd.Prim: The created robot prim.
            """
            robot_prim = create_prim(prim_path=prim_path, prim_type="Xform", position=position)
            robot_prim.GetReferences().AddReference(asset_url)
            print(f"  ✅ Reference added: {asset_url}")
            return robot_prim
        
        # Robot configurations with positions
        robot_configs = {
            "UR10": {"prim_path": "/World/UR10", "asset_url": robot_assets["UR10"], "position": (-1.0, 0.0, 0.0)},
            "Franka": {"prim_path": "/World/Franka", "asset_url": robot_assets["Franka"], "position": (1.0, 0.0, 0.0)}
        }
        
        robot_prims = {}
        for robot_name, config in robot_configs.items():
            robot_prim = ensure_offset_and_reference(config["prim_path"], config["asset_url"], config["position"])
            robot_prims[robot_name] = robot_prim
            print(f"  - {robot_name} robot loaded and positioned.")
        
        print("\n⏳ Waiting for USD references to load...")
        for i in range(5):
            my_world.step()
            time.sleep(0.1)
        
        print(f"\n🔧 Step A: Cleaning up nested ArticulationRoots...")
        
        def list_articulation_roots_under(stage, base_path: str) -> List[Usd.Prim]:
            """Finds all prims with an ArticulationRootAPI under a given path.

            Args:
                stage (Usd.Stage): The current USD stage.
                base_path (str): The base path to search under.

            Returns:
                List[Usd.Prim]: A list of prims with the ArticulationRootAPI.
            """
            roots = [p for p in stage.Traverse() if p.GetPath().pathString.startswith(base_path) and p.HasAPI(UsdPhysics.ArticulationRootAPI)]
            roots.sort(key=lambda r: len(r.GetPath().pathString))
            return roots
        
        def collapse_to_single_root(stage, base_path: str) -> Optional[str]:
            """Ensures only one ArticulationRoot exists in a subtree by removing nested ones.

            Args:
                stage (Usd.Stage): The current USD stage.
                base_path (str): The base path of the robot's subtree.

            Returns:
                Optional[str]: The path of the final, single articulation root.
            """
            roots = list_articulation_roots_under(stage, base_path)
            print(f"    🔍 Found {len(roots)} ArticulationRoots under {base_path}.")
            if not roots:
                print(f"    ⚠️ No existing ArticulationRoot found. Applying to base prim.")
                base_prim = get_prim_at_path(base_path)
                if base_prim.IsValid():
                    UsdPhysics.ArticulationRootAPI.Apply(base_prim)
                    return base_path
                return None
            
            keep_root = roots[0]
            print(f"    ✅ Keeping root: {keep_root.GetPath()}")
            for extra_root in roots[1:]:
                print(f"    🗑️  Removing extra root: {extra_root.GetPath()}")
                extra_root.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            return keep_root.GetPath().pathString
        
        articulation_roots = {}
        for robot_name, config in robot_configs.items():
            root_path = collapse_to_single_root(stage, config["prim_path"])
            if root_path:
                articulation_roots[robot_name] = root_path
                print(f"    ✅ {robot_name}: Final root = {root_path}")
        
        print(f"\n🔄 Step B: Resetting World...")
        if articulation_roots:
            my_world.reset()
            print("  ✅ World reset.")
            for i in range(3):
                omni.kit.app.get_app().update()
                time.sleep(0.02)
            print("  ✅ Update frames processed.")
        
        print(f"\n🔧 Step C: Creating PhysX Tensors SimulationView...")
        articulation_views = {}
        try:
            sim_view = tensors.create_simulation_view("torch")
            print("  ✅ SimulationView created.")
            for robot_name, root_path in articulation_roots.items():
                try:
                    print(f"    🔧 Creating ArticulationView for {robot_name} at {root_path}")
                    view = sim_view.create_articulation_view(root_path)
                    articulation_views[robot_name] = view
                    print(f"    ✅ {robot_name}: ArticulationView created successfully.")
                except Exception as e:
                    print(f"    ❌ Failed to create ArticulationView for {robot_name}: {e}")
                    traceback.print_exc()
            omni.kit.app.get_app().update()
        except Exception as e:
            print(f"  ❌ Failed to create SimulationView: {e}")
            traceback.print_exc()
            return
        
        print(f"\n🎯 Step D: Testing robot control...")
        initialized_robots = []
        for robot_name, view in articulation_views.items():
            try:
                print(f"\n  🤖 Testing {robot_name}:")
                if view.count > 0:
                    print(f"    - DOF count: {view.count}")
                    positions = view.get_dof_positions()
                    print(f"    - Initial positions (shape): {positions.shape}")
                    
                    robot_indices = torch.arange(view.count, dtype=torch.int32, device="cuda:0")
                    view.set_dof_position_targets(torch.zeros_like(positions), indices=robot_indices)
                    print(f"    ✅ Control command sent successfully.")
                    initialized_robots.append(robot_name)
                else:
                    print(f"    ⚠️ DOF count is 0. Cannot test control.")
            except Exception as e:
                print(f"    ❌ Failed to test control for {robot_name}: {e}")
        
        if initialized_robots:
            print(f"\n🚀 Running final simulation steps...")
            for step in range(5):
                my_world.step()
                print(f"    Step {step+1}/5 complete.")
        
        print(f"\n📊 Final Summary:")
        print(f"  - Robots Loaded: {len(robot_prims)}")
        print(f"  - Articulation Roots Cleaned: {len(articulation_roots)}")
        print(f"  - Articulation Views Created: {len(articulation_views)}")
        print(f"  - Successfully Initialized & Controlled: {len(initialized_robots)}")
        
        if len(initialized_robots) == len(robot_configs):
            print("\n🎉 All robots initialized successfully! PhysX Tensors integration is complete!")
        else:
            print(f"\n⚠️ Some robots failed to initialize. Success: {initialized_robots}")
        
    except Exception as e:
        print(f"❌ An error occurred in the main function: {e}")
        traceback.print_exc()
    
    finally:
        # Clean shutdown
        simulation_app.close()
        print("Simulation application closed.")

if __name__ == "__main__":
    main()