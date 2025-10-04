#!/usr/bin/env python3
"""Script to run the RoArm MCP server with direct Isaac Sim integration.

This script is the primary entry point for launching the full simulation
environment. It starts both the MCP server and the Isaac Sim application,
managing the simulation life cycle via the `IsaacSimMCPHandler`.

!!! IMPORTANT !!!
This script MUST be run using the python executable provided with Isaac Sim,
as it requires the Isaac Sim environment and libraries to be loaded.

Usage (Linux):
    /path/to/isaac_sim/python.sh examples/run_isaac_sim_server.py --robot-type <robot>

Usage (Windows):
    C:\\path\\to\\isaac_sim\\python.bat examples\\run_isaac_sim_server.py --robot-type <robot>

Example:
    # From the Isaac Sim root directory:
    ./python.sh /path/to/roarm_mcp/examples/run_isaac_sim_server.py --robot-type ur10 --headless
"""

import argparse
import asyncio
import logging
import sys

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Attempt to import necessary modules. This will fail if not run with Isaac Sim's python.
try:
    from roarm_mcp.mcp.server import MCPServer
    from roarm_mcp.mcp.isaac_sim_handler import IsaacSimMCPHandler
except ImportError as e:
    logger.error(f"Failed to import required modules: {e}")
    logger.error("This script must be run with Isaac Sim's Python environment.")
    logger.error("Example: /path/to/isaac_sim/python.sh examples/run_isaac_sim_server.py")
    sys.exit(1)


async def run_isaac_sim_server(
    host: str,
    port: int,
    robot_type: str,
    headless: bool
) -> None:
    """Initializes and runs the MCP server with a direct Isaac Sim handler.

    Args:
        host (str): The hostname or IP address to bind the server to.
        port (int): The port number for the server to listen on.
        robot_type (str): The type of robot to load in the simulation ("ur10" or "franka").
        headless (bool): If True, runs Isaac Sim without a graphical interface.
    """
    logger.info("🤖 Starting Isaac Sim MCP Server...")
    
    # 1. Create the MCP Server instance
    server = MCPServer(host=host, port=port)
    
    # 2. Create the Isaac Sim handler, which will manage the sim application
    handler = IsaacSimMCPHandler(robot_type=robot_type, headless=headless)
    
    # 3. Register the handler with the server
    server.register_env_handler(handler)
    
    # 4. Start the server
    await server.start()
    logger.info(f"🚀 Isaac Sim MCP Server is running at ws://{host}:{port}")
    logger.info(f"📡 Robot Type: {robot_type.upper()}")
    logger.info(f"👀 Headless Mode: {'ON' if headless else 'OFF'}")
    
    try:
        # Keep the server running until interrupted
        while server.running:
            await asyncio.sleep(1)
    except (KeyboardInterrupt, asyncio.CancelledError):
        logger.info("🛑 Shutdown signal received.")
    finally:
        # 5. Clean up resources gracefully
        logger.info("Cleaning up resources...")
        if handler.is_initialized():
            await handler.close()
        await server.stop()
        logger.info("✅ Isaac Sim MCP Server has been shut down.")


def main():
    """Parses command-line arguments and starts the Isaac Sim MCP server."""
    parser = argparse.ArgumentParser(
        description="Run the RoArm MCP server with direct Isaac Sim integration.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("--host", type=str, default="localhost", help="Host to bind the server to.")
    parser.add_argument("--port", type=int, default=8765, help="Port to bind the server to.")
    parser.add_argument(
        "--robot-type",
        type=str,
        choices=["ur10", "franka"],
        default="ur10",
        help="Type of robot to load in the simulation."
    )
    parser.add_argument("--headless", action="store_true", help="Run Isaac Sim in headless mode.")
    args = parser.parse_args()

    logger.info("Starting server with configuration:")
    logger.info(f"  Host: {args.host}")
    logger.info(f"  Port: {args.port}")
    logger.info(f"  Robot Type: {args.robot_type}")
    logger.info(f"  Headless: {args.headless}")

    try:
        asyncio.run(run_isaac_sim_server(
            host=args.host,
            port=args.port,
            robot_type=args.robot_type,
            headless=args.headless
        ))
    except Exception as e:
        logger.error(f"❌ A critical error occurred during server execution: {e}", exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    main()