#!/usr/bin/env python3
"""
Script to run the RoArm MCP server.

This script starts the MCP server and connects it to a robot arm environment.
"""

import argparse
import asyncio
import logging

from roarm_mcp.mcp.server import MCPServer
from roarm_mcp.mcp.env_handler import create_handler

# Initialize logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


async def run_server(
    host: str,
    port: int,
    env_type: str,
    robot_type: str,
    headless: bool
) -> None:
    """Run the MCP server.
    
    Args:
        host: The host to bind to.
        port: The port to bind to.
        env_type: The type of environment to create.
        robot_type: The type of robot to use.
        headless: Whether to run in headless mode.
    """
    # Create server
    server = MCPServer(host=host, port=port)
    
    # Create environment handler
    handler = create_handler(env_type=env_type, robot_type=robot_type, headless=headless)
    
    # Register handler with server
    server.register_env_handler(handler)
    
    # Start server
    await server.start()
    logger.info(f"MCP server running on {host}:{port}")
    
    try:
        # Keep the server running
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received")
    finally:
        # Stop server
        await server.stop()
        logger.info("MCP server stopped")


def main() -> None:
    """Run the main function."""
    parser = argparse.ArgumentParser(description="Run the RoArm MCP server")
    parser.add_argument("--host", type=str, default="localhost", help="Host to bind to")
    parser.add_argument("--port", type=int, default=8765, help="Port to bind to")
    parser.add_argument("--env-type", type=str, default="joint_position", choices=["joint_position", "end_effector"],
                        help="Type of environment to create")
    parser.add_argument("--robot-type", type=str, default="ur10", choices=["ur10", "franka"],
                        help="Type of robot to use")
    parser.add_argument("--headless", action="store_true", help="Run in headless mode")
    
    args = parser.parse_args()
    
    asyncio.run(run_server(args.host, args.port, args.env_type, args.robot_type, args.headless))


if __name__ == "__main__":
    main()