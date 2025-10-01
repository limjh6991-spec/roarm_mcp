#!/usr/bin/env python3
"""
Sample client for testing the RoArm MCP server.

This script demonstrates how to connect to the MCP server and control a robot arm.
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
    print("websockets package not found. Install it using 'pip install websockets'")
    exit(1)

try:
    import gymnasium as gym
    from gymnasium.spaces import Box, Discrete, Dict as DictSpace
except ImportError:
    print("gymnasium package not found. Install it using 'pip install gymnasium'")
    exit(1)

from roarm_mcp.mcp.protocol import (
    MCPMessage, MCPResetMessage, MCPStepMessage, MCPRenderMessage, MCPCloseMessage,
    MCPActionSpaceMessage, MCPObservationSpaceMessage
)

# Initialize logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


async def run_client(host: str, port: int, num_episodes: int, steps_per_episode: int, render: bool = True) -> None:
    """Run the MCP client.
    
    Args:
        host: The host to connect to.
        port: The port to connect to.
        num_episodes: The number of episodes to run.
        steps_per_episode: The number of steps per episode.
        render: Whether to render the environment.
    """
    uri = f"ws://{host}:{port}"
    logger.info(f"Connecting to MCP server at {uri}")
    
    async with websockets.connect(uri) as websocket:
        logger.info("Connected to MCP server")
        
        # Get action and observation spaces
        logger.info("Getting action and observation spaces...")
        
        # Get action space
        action_space_msg = MCPActionSpaceMessage()
        await websocket.send(action_space_msg.to_json())
        response = await websocket.recv()
        action_space_data = json.loads(response)
        logger.info(f"Action space: {action_space_data}")
        
        # Get observation space
        observation_space_msg = MCPObservationSpaceMessage()
        await websocket.send(observation_space_msg.to_json())
        response = await websocket.recv()
        observation_space_data = json.loads(response)
        logger.info(f"Observation space: {observation_space_data}")
        
        # Run episodes
        for episode in range(num_episodes):
            logger.info(f"Starting episode {episode+1}/{num_episodes}")
            
            # Reset environment
            reset_msg = MCPResetMessage()
            await websocket.send(reset_msg.to_json())
            response = await websocket.recv()
            observation_data = json.loads(response)
            observation = observation_data["observation"]
            logger.info(f"Initial observation: {observation}")
            
            # Run steps
            total_reward = 0.0
            for step in range(steps_per_episode):
                # Sample random action
                if action_space_data["space"]["type"] == "box":
                    low = np.array(action_space_data["space"]["low"])
                    high = np.array(action_space_data["space"]["high"])
                    action = np.random.uniform(low, high).tolist()
                elif action_space_data["space"]["type"] == "discrete":
                    action = np.random.randint(0, action_space_data["space"]["n"])
                else:
                    logger.warning(f"Unsupported action space type: {action_space_data['space']['type']}")
                    action = 0
                
                # Take step
                step_msg = MCPStepMessage(action)
                await websocket.send(step_msg.to_json())
                
                # Get responses
                observation = None
                reward = 0.0
                terminated = False
                truncated = False
                info = {}
                
                # We expect 5 messages: observation, reward, terminated, truncated, info
                for _ in range(5):
                    response = await websocket.recv()
                    data = json.loads(response)
                    msg_type = data["type"]
                    
                    if msg_type == "observation":
                        observation = data["observation"]
                    elif msg_type == "reward":
                        reward = data["reward"]
                    elif msg_type == "terminated":
                        terminated = data["terminated"]
                    elif msg_type == "truncated":
                        truncated = data["truncated"]
                    elif msg_type == "info":
                        info = data["info"]
                    elif msg_type == "error":
                        logger.error(f"Error from server: {data['error']}")
                        break
                
                total_reward += reward
                
                logger.info(f"Step {step+1}/{steps_per_episode} - Reward: {reward:.4f} - Total: {total_reward:.4f}")
                
                # Render if requested
                if render:
                    render_msg = MCPRenderMessage()
                    await websocket.send(render_msg.to_json())
                
                # Check if episode is done
                if terminated or truncated:
                    logger.info(f"Episode finished early: terminated={terminated}, truncated={truncated}")
                    break
            
            logger.info(f"Episode {episode+1} finished with total reward {total_reward:.4f}")
        
        # Close environment
        close_msg = MCPCloseMessage()
        await websocket.send(close_msg.to_json())
        logger.info("Environment closed")


async def main() -> None:
    """Run the main function."""
    parser = argparse.ArgumentParser(description="Sample client for RoArm MCP server")
    parser.add_argument("--host", type=str, default="localhost", help="Host to connect to")
    parser.add_argument("--port", type=int, default=8765, help="Port to connect to")
    parser.add_argument("--episodes", type=int, default=5, help="Number of episodes to run")
    parser.add_argument("--steps", type=int, default=100, help="Number of steps per episode")
    parser.add_argument("--no-render", action="store_true", help="Disable rendering")
    
    args = parser.parse_args()
    
    await run_client(args.host, args.port, args.episodes, args.steps, not args.no_render)


if __name__ == "__main__":
    asyncio.run(main())