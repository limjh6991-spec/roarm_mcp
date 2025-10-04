#!/usr/bin/env python3
"""A sample client for the Isaac Sim MCP integration.

This script demonstrates how to create a low-level client to communicate with
an MCP server, specifically one that bundles the response to a 'step' action
into a single message. This contrasts with `sample_client.py`, which expects
five separate messages.

This client is suitable for interacting with servers like the one started by
`simple_isaac_sim_server.py`.

The client performs the following actions:
  1. Connects to the specified WebSocket server URL.
  2. Defines an `IsaacSimMCPClient` class to encapsulate communication logic.
  3. Runs a number of test episodes, performing random actions.
  4. Gracefully disconnects upon completion or interruption.

Usage:
    python -m examples.isaac_sim_client --server-url <ws_url> --episodes <n>
"""

import asyncio
import json
import logging
import numpy as np
import argparse
import websockets
from typing import Dict, Any, Tuple
import traceback

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


class IsaacSimMCPClient:
    """A client for interacting with an Isaac Sim MCP server.

    This class handles the WebSocket connection and message passing for
    controlling a simulated robot environment.

    Attributes:
        server_url (str): The WebSocket URL of the server.
        websocket (Optional[websockets.WebSocketClientProtocol]): The active
            WebSocket connection instance.
    """
    
    def __init__(self, server_url: str):
        """Initializes the Isaac Sim MCP client.
        
        Args:
            server_url (str): The WebSocket URL of the MCP server.
        """
        self.server_url = server_url
        self.websocket = None
        
    async def connect(self) -> None:
        """Establishes a connection to the MCP server."""
        logger.info(f"🔌 Connecting to server: {self.server_url}")
        self.websocket = await websockets.connect(self.server_url)
        logger.info("✅ Successfully connected to server.")
        
    async def disconnect(self) -> None:
        """Closes the connection to the server."""
        if self.websocket:
            await self.websocket.close()
            logger.info("🔌 Disconnected from server.")
            
    async def send_message(self, message: Dict[str, Any]) -> Dict[str, Any]:
        """Sends a JSON message to the server and awaits a JSON response.
        
        Args:
            message (Dict[str, Any]): The message to send as a dictionary.
            
        Returns:
            Dict[str, Any]: The server's response as a dictionary.

        Raises:
            RuntimeError: If the client is not connected to the server.
        """
        if not self.websocket:
            raise RuntimeError("Client is not connected to the server.")
            
        await self.websocket.send(json.dumps(message))
        response = await self.websocket.recv()
        return json.loads(response)
        
    async def reset_environment(self) -> np.ndarray:
        """Sends a 'reset' message to the environment.
        
        Returns:
            np.ndarray: The initial observation from the environment.
        
        Raises:
            RuntimeError: If the server returns an unexpected response.
        """
        logger.info("🔄 Resetting environment...")
        message = {"type": "reset"}
        response = await self.send_message(message)
        
        if response.get("type") == "observation":
            observation = np.array(response["data"])
            logger.info(f"✅ Initial observation received with shape: {observation.shape}")
            return observation
        else:
            raise RuntimeError(f"Unexpected response during reset: {response}")
            
    async def step_environment(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        """Sends a 'step' message with an action to the environment.
        
        Args:
            action (np.ndarray): The action to perform.
            
        Returns:
            A tuple containing (observation, reward, terminated, truncated, info).
        """
        message = {"type": "step", "action": action.tolist()}
        response = await self.send_message(message)
        
        observation = np.array(response.get("observation", []))
        reward = response.get("reward", 0.0)
        terminated = response.get("terminated", False)
        truncated = response.get("truncated", False)
        info = response.get("info", {})
        
        return observation, reward, terminated, truncated, info
        
    async def get_action_space(self) -> Dict[str, Any]:
        """Requests the environment's action space."""
        message = {"type": "action_space"}
        response = await self.send_message(message)
        return response.get("space", {})
        
    async def get_observation_space(self) -> Dict[str, Any]:
        """Requests the environment's observation space."""
        message = {"type": "observation_space"}
        response = await self.send_message(message)
        return response.get("space", {})


async def run_test_episode(client: IsaacSimMCPClient, steps: int, robot_type: str) -> None:
    """Runs a single test episode with random actions.
    
    Args:
        client (IsaacSimMCPClient): An initialized and connected client.
        steps (int): The number of steps to perform in the episode.
        robot_type (str): The type of robot being controlled (e.g., "ur10", "franka").
    """
    logger.info(f"🤖 Starting test episode for {robot_type.upper()} robot.")
    logger.info(f"📏 Running for {steps} steps.")
    
    try:
        action_space = await client.get_action_space()
        obs_space = await client.get_observation_space()
        logger.info(f"📊 Action Space: {action_space}")
        logger.info(f"📊 Observation Space: {obs_space}")
        
        dof_count = len(action_space.get("high", []))
        if dof_count == 0:
            dof_count = 6 if robot_type == "ur10" else 9
            logger.warning(f"Could not determine DOF count from action space. Defaulting to {dof_count}.")
        else:
            logger.info(f"🔧 Determined robot DOF count: {dof_count}")
        
    except Exception as e:
        logger.error(f"⚠️ Failed to get space information: {e}. Defaulting DOF count.")
        dof_count = 6 if robot_type == "ur10" else 9
            
    try:
        initial_obs = await client.reset_environment()
    except Exception as e:
        logger.error(f"❌ Failed to reset environment: {e}")
        return
        
    total_reward = 0.0
    for step in range(steps):
        try:
            # Generate a random action (small, incremental movements)
            action = np.random.uniform(-0.1, 0.1, size=dof_count)
            obs, reward, terminated, truncated, info = await client.step_environment(action)
            total_reward += reward
            
            if step % 10 == 0 or step == steps - 1:
                logger.info(
                    f"🎯 Step {step+1:3d}: Reward={reward:6.3f}, "
                    f"Total Reward={total_reward:8.3f}, Obs Shape={obs.shape}"
                )
                
            if terminated or truncated:
                logger.info(f"🏁 Episode finished early at step {step+1}. Terminated: {terminated}, Truncated: {truncated}")
                break
                
        except Exception as e:
            logger.error(f"❌ An error occurred during step {step+1}: {e}")
            break
            
    logger.info(f"🏆 Episode complete! Final reward: {total_reward:.3f}")


async def main():
    """Parses arguments and runs the main client logic."""
    parser = argparse.ArgumentParser(
        description="A test client for the Isaac Sim MCP server.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("--server-url", type=str, default="ws://localhost:8765", help="MCP server WebSocket URL.")
    parser.add_argument("--episodes", type=int, default=3, help="Number of episodes to run.")
    parser.add_argument("--steps", type=int, default=50, help="Number of steps per episode.")
    parser.add_argument("--robot-type", type=str, choices=["ur10", "franka"], default="ur10", help="Type of robot.")
    args = parser.parse_args()

    client = IsaacSimMCPClient(args.server_url)
    
    try:
        await client.connect()
        for i in range(args.episodes):
            logger.info(f"\n🎬 --- Starting Episode {i + 1}/{args.episodes} ---")
            await run_test_episode(client, args.steps, args.robot_type)
            if i < args.episodes - 1:
                logger.info("⏰ Waiting before next episode...")
                await asyncio.sleep(2)
                
    except KeyboardInterrupt:
        logger.info("🛑 User interrupted the client.")
    except Exception as e:
        logger.error(f"❌ A critical error occurred: {e}")
        logger.error(traceback.format_exc())
    finally:
        await client.disconnect()
        logger.info("👋 Client has shut down.")


if __name__ == "__main__":
    asyncio.run(main())