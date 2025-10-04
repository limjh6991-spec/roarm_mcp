#!/usr/bin/env python3
"""A sample client for testing the RoArm MCP server.

This script provides a low-level example of how to interact with the MCPServer
using the `websockets` library directly. It does not use the `MCPClient`
gym environment, but instead manually constructs and sends MCP messages in JSON
format.

This is useful for debugging the server or for clients that do not wish to use
the Gymnasium interface.

The client performs the following actions:
  1. Connects to the specified MCP server.
  2. Requests and prints the action and observation spaces.
  3. Runs a specified number of episodes.
  4. In each episode, it resets the environment.
  5. It then enters a loop, sending random actions and receiving responses.
  6. After each 'STEP' message, it expects and waits for five separate
     responses: 'observation', 'reward', 'terminated', 'truncated', and 'info'.
  7. It gracefully closes the connection upon completion.

Usage:
    python -m examples.sample_client --host <host> --port <port>
"""

import argparse
import asyncio
import json
import logging
import numpy as np
from typing import Any, Dict, List, Optional, Tuple, Union

try:
    import websockets
except ImportError:
    print("The 'websockets' package is not found. Please install it using: pip install websockets")
    exit(1)

from roarm_mcp.mcp.protocol import (
    MCPResetMessage, MCPStepMessage, MCPRenderMessage, MCPCloseMessage,
    MCPActionSpaceMessage, MCPObservationSpaceMessage
)

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


async def run_client(host: str, port: int, num_episodes: int, steps_per_episode: int, render: bool = True) -> None:
    """Connects to the MCP server and runs a series of random actions.

    Args:
        host (str): The hostname or IP address of the MCP server.
        port (int): The port number of the MCP server.
        num_episodes (int): The number of episodes to run.
        steps_per_episode (int): The number of steps to take in each episode.
        render (bool): Whether to send 'RENDER' messages to the server.
    """
    uri = f"ws://{host}:{port}"
    logger.info(f"Connecting to MCP server at {uri}")
    
    try:
        async with websockets.connect(uri, ping_interval=None) as websocket:
            logger.info("Successfully connected to MCP server.")
            
            # --- Get Action and Observation Spaces ---
            logger.info("Requesting action and observation spaces...")

            # Request and receive action space
            await websocket.send(MCPActionSpaceMessage().to_json())
            response = await websocket.recv()
            action_space_data = json.loads(response)
            logger.info(f"Received Action Space: {action_space_data}")
            
            # Request and receive observation space
            await websocket.send(MCPObservationSpaceMessage().to_json())
            response = await websocket.recv()
            observation_space_data = json.loads(response)
            logger.info(f"Received Observation Space: {observation_space_data}")

            # --- Run Episodes ---
            for episode in range(num_episodes):
                logger.info(f"--- Starting Episode {episode + 1}/{num_episodes} ---")
                
                # Reset the environment
                await websocket.send(MCPResetMessage().to_json())
                response = await websocket.recv() # The first response after reset is the initial observation
                observation = json.loads(response)["observation"]
                logger.info(f"Initial observation received.")
                
                total_reward = 0.0
                for step in range(steps_per_episode):
                    # --- Generate a random action based on the action space ---
                    space_type = action_space_data.get("space", {}).get("type")
                    if space_type == "box":
                        low = np.array(action_space_data["space"]["low"])
                        high = np.array(action_space_data["space"]["high"])
                        action = np.random.uniform(low, high).tolist()
                    elif space_type == "discrete":
                        action = np.random.randint(0, action_space_data["space"]["n"])
                    else:
                        logger.warning(f"Unsupported action space type '{space_type}'. Defaulting to action 0.")
                        action = 0

                    # --- Send STEP message ---
                    await websocket.send(MCPStepMessage(action).to_json())

                    # --- Receive the 5-part response for a step ---
                    # The server sends five separate messages for each step.
                    step_responses = {}
                    for _ in range(5):
                        response = await websocket.recv()
                        data = json.loads(response)
                        msg_type = data.get("type")
                        if msg_type in ["observation", "reward", "terminated", "truncated", "info"]:
                            step_responses[msg_type] = data[msg_type]
                        elif msg_type == "error":
                            logger.error(f"Received error from server: {data['error']}")
                            break
                    
                    if "reward" not in step_responses:
                        logger.error("Did not receive a reward from the server. Exiting step loop.")
                        break

                    total_reward += step_responses.get("reward", 0)

                    logger.info(f"Step {step + 1}/{steps_per_episode} | "
                                f"Reward: {step_responses.get('reward', 0):.4f} | "
                                f"Total Reward: {total_reward:.4f}")

                    # Optionally send a render message
                    if render:
                        await websocket.send(MCPRenderMessage().to_json())

                    # Check if the episode has ended
                    if step_responses.get("terminated") or step_responses.get("truncated"):
                        logger.info(f"Episode finished at step {step + 1}. "
                                    f"Terminated: {step_responses.get('terminated')}, "
                                    f"Truncated: {step_responses.get('truncated')}")
                        break
                
                logger.info(f"--- Episode {episode + 1} finished with total reward {total_reward:.4f} ---")
            
            # --- Close the environment on the server ---
            await websocket.send(MCPCloseMessage().to_json())
            logger.info("Sent 'CLOSE' message to server.")

    except (websockets.exceptions.ConnectionClosedError, ConnectionRefusedError) as e:
        logger.error(f"Connection to {uri} failed: {e}")
    except Exception as e:
        logger.error(f"An unexpected error occurred: {e}", exc_info=True)


def main() -> None:
    """Parses command-line arguments and runs the asynchronous client."""
    parser = argparse.ArgumentParser(
        description="A sample low-level client for the RoArm MCP server.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("--host", type=str, default="localhost", help="The server's hostname or IP address.")
    parser.add_argument("--port", type=int, default=8765, help="The server's port number.")
    parser.add_argument("--episodes", type=int, default=3, help="Number of episodes to run.")
    parser.add_argument("--steps", type=int, default=50, help="Number of steps per episode.")
    parser.add_argument("--no-render", action="store_true", help="Disable sending render requests to the server.")
    
    args = parser.parse_args()
    
    try:
        asyncio.run(run_client(args.host, args.port, args.episodes, args.steps, not args.no_render))
    except KeyboardInterrupt:
        logger.info("Client terminated by user.")


if __name__ == "__main__":
    main()