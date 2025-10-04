#!/usr/bin/env python3
"""Example script to run the RoArm Model Context Protocol (MCP) server.

This script initializes and starts an MCPServer, which listens for client
connections and facilitates interaction with a simulated robot arm environment.
It demonstrates how to:
  - Parse command-line arguments for server configuration.
  - Create an MCPServer instance.
  - Create a robot environment handler using the `create_handler` factory.
  - Register the handler with the server.
  - Run the server indefinitely until interrupted.

Usage:
    python -m examples.run_server --host <host> --port <port> --env-type <env> --robot-type <robot> [--headless]

Examples:
    # Run a server with a UR10 robot in a joint position environment
    python -m examples.run_server --robot-type ur10 --env-type joint_position

    # Run a server in headless mode with a Franka robot
    python -m examples.run_server --robot-type franka --env-type end_effector --headless
"""

import argparse
import asyncio
import logging

from roarm_mcp.mcp.server import MCPServer
from roarm_mcp.mcp.env_handler import create_handler

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


async def run_server(
    host: str,
    port: int,
    env_type: str,
    robot_type: str,
    headless: bool
) -> None:
    """Sets up and runs the MCP server.

    Args:
        host (str): The hostname or IP address to bind the server to.
        port (int): The port number to listen on.
        env_type (str): The type of robot environment to create
                        ("joint_position" or "end_effector").
        robot_type (str): The type of robot to use ("ur10" or "franka").
        headless (bool): Whether to run the simulation in headless mode.
    """
    # 1. Create the MCPServer instance
    server = MCPServer(host=host, port=port)
    
    # 2. Create the appropriate environment handler based on arguments
    logger.info(f"Creating '{env_type}' environment for '{robot_type}' robot...")
    handler = create_handler(env_type=env_type, robot_type=robot_type, headless=headless)
    
    # 3. Register the handler with the server
    server.register_env_handler(handler)
    
    # 4. Start the server
    await server.start()
    logger.info(f"MCP server started and listening on ws://{host}:{port}")
    
    try:
        # Keep the server running until a KeyboardInterrupt is received
        while True:
            await asyncio.sleep(3600) # Sleep for a long time to keep the task alive
    except (KeyboardInterrupt, asyncio.CancelledError):
        logger.info("Shutdown signal received.")
    finally:
        # 5. Stop the server gracefully
        logger.info("Stopping MCP server...")
        await server.stop()
        logger.info("MCP server stopped.")


def main() -> None:
    """Parses command-line arguments and starts the server."""
    parser = argparse.ArgumentParser(
        description="Run the RoArm MCP server for a simulated robot arm environment.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "--host",
        type=str,
        default="localhost",
        help="Hostname or IP address for the server to bind to."
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8765,
        help="Port number for the server to listen on."
    )
    parser.add_argument(
        "--env-type",
        type=str,
        default="joint_position",
        choices=["joint_position", "end_effector"],
        help="The type of control environment to create."
    )
    parser.add_argument(
        "--robot-type",
        type=str,
        default="ur10",
        choices=["ur10", "franka"],
        help="The type of robot arm to simulate."
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run the Isaac Sim environment in headless mode (no GUI)."
    )
    
    args = parser.parse_args()
    
    logger.info("Starting server with configuration:")
    logger.info(f"  Host: {args.host}")
    logger.info(f"  Port: {args.port}")
    logger.info(f"  Env Type: {args.env_type}")
    logger.info(f"  Robot Type: {args.robot_type}")
    logger.info(f"  Headless: {args.headless}")

    try:
        asyncio.run(run_server(args.host, args.port, args.env_type, args.robot_type, args.headless))
    except KeyboardInterrupt:
        logger.info("Server terminated by user.")


if __name__ == "__main__":
    main()