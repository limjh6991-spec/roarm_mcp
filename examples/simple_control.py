#!/usr/bin/env python3
"""A simple (legacy) example of direct robot arm control.

!!! WARNING: This script appears to be from a different project or an older
!!! version of this repository. The imports (e.g., `robot_arm.arm_interface`)
!!! and logic do not align with the current `roarm-mcp` architecture, which
!!! uses a client-server model based on the Model Context Protocol (MCP).

This script likely demonstrates how to control a robot arm either in a direct
simulation mode or by connecting to physical hardware via a serial port.
It is NOT a client for the MCPServer.

For correct usage of the current system, please refer to:
  - `examples/run_server.py`: To start the main MCP server.
  - `examples/isaac_sim_client.py`: A high-level client using the gym environment.
  - `examples/sample_client.py`: A low-level client for direct websocket communication.

This file is being documented for archival purposes but is not considered a
functional example for the `roarm-mcp` project.
"""

import sys
import os
import yaml
import argparse
import time
from pathlib import Path

# Add project root directory to path
project_root = Path(__file__).parent.parent
sys.path.append(str(project_root))

# --- WARNING: The following imports are likely to fail as they do not ---
# --- match the current project structure.                             ---
try:
    from robot_arm.arm_interface import ArmInterface
    from robot_arm.simulation.gym_env import RoArmEnv
    from robot_arm.simulation.visualization import RobotVisualizer
    from robot_arm.hardware.serial_interface import SerialArmInterface
except ImportError as e:
    print(f"Failed to import legacy modules: {e}")
    print("This script is likely incompatible with the current project structure.")
    # To prevent a hard crash, we create dummy classes
    class RoArmEnv: pass
    class SerialArmInterface: pass
    class RobotVisualizer: pass


def parse_args() -> argparse.Namespace:
    """Parses command-line arguments.

    Returns:
        argparse.Namespace: The parsed command-line arguments.
    """
    parser = argparse.ArgumentParser(description="A simple (legacy) robot arm control example.")
    parser.add_argument("--sim", action="store_true", help="Use simulation mode.")
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0", help="Serial port for hardware connection.")
    parser.add_argument("--visualize", action="store_true", help="Enable visualization.")
    return parser.parse_args()

def main():
    """The main function to run the control example."""
    args = parse_args()
    
    # Load configuration
    # Note: This config path may also be from the legacy structure.
    config_path = project_root / "config" / "robot_config.yaml"
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
    except FileNotFoundError:
        print(f"Warning: Configuration file not found at {config_path}. Using default values.")
        config = {}

    # --- Create robot arm interface based on mode ---
    if args.sim:
        print("Initializing robot arm in simulation mode...")
        # This section demonstrates interaction with a direct gym environment
        try:
            env = RoArmEnv(render_mode="human" if args.visualize else None)
            obs, _ = env.reset()
            
            # Perform some basic actions
            for _ in range(100):
                action = [0.01, 0.01, 0.01, 0.0, 0.0, 0.0]  # Example of a simple movement
                obs, reward, terminated, truncated, info = env.step(action)
                print(f"Reward: {reward}, Distance to target: {info.get('distance_to_target', 'N/A')}")
                time.sleep(0.01)
                if terminated or truncated:
                    break
        except NameError:
            print("Could not run simulation because 'RoArmEnv' failed to import.")
            
    else:
        print(f"Initializing robot arm in hardware mode... Port: {args.port}")
        # This section demonstrates interaction with a physical robot arm
        try:
            arm = SerialArmInterface(port=args.port)
            
            if arm.connect():
                print("Robot arm connected successfully!")

                # Get current joint positions
                joint_pos = arm.get_joint_positions()
                print(f"Current joint positions: {joint_pos}")

                # Get current cartesian position
                cart_pos = arm.get_cartesian_position()
                print(f"Current cartesian position: {cart_pos}")

                # Simple joint move
                print("Moving joints...")
                new_joint_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Neutral position
                arm.move_joints(new_joint_pos, speed=0.5)
                time.sleep(2.0)
                
                # Gripper control
                print("Opening gripper...")
                arm.gripper_control(0.08, speed=0.5)  # Open fully
                time.sleep(1.0)

                print("Closing gripper...")
                arm.gripper_control(0.0, speed=0.5)  # Close fully
                time.sleep(1.0)

                # Cartesian move
                print("Moving to cartesian position...")
                new_cart_pos = [0.2, 0.0, 0.3, 0.0, 3.14, 0.0]  # Example position
                arm.move_cartesian(new_cart_pos, speed=0.5)
                time.sleep(2.0)

                # Check robot status
                status = arm.get_status()
                print(f"Robot status: {status}")

                arm.disconnect()
                print("Robot arm disconnected.")

                if args.visualize:
                    print("Visualizing robot arm poses...")
                    visualizer = RobotVisualizer()
                    visualizer.plot_robot(joint_pos, new_cart_pos[:3])

            else:
                print("Failed to connect to the robot arm!")

        except NameError:
            print("Could not run hardware mode because 'SerialArmInterface' failed to import.")

if __name__ == "__main__":
    main()