#!/usr/bin/env python3
"""
Script to run the RoArm MCP server with Isaac Sim integration.

이 스크립트는 Isaac Sim 5.0 PhysX Tensors를 사용하는 MCP 서버를 시작합니다.
"""

import argparse
import asyncio
import logging
import sys
import os

# Initialize logging first
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Add the project root to Python path
project_root = '/home/roarm_m3/dev_roarm/roarm_mcp'
if project_root not in sys.path:
    sys.path.insert(0, project_root)

# Also add parent directory for relative imports
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

try:
    from mcp.server import MCPServer
    from mcp.isaac_sim_handler import IsaacSimMCPHandler
except ImportError as e:
    logger.error(f"Failed to import MCP modules: {e}")
    logger.info(f"Current Python path: {sys.path}")
    # Try alternative import
    sys.path.append('/home/roarm_m3/dev_roarm')
    from roarm_mcp.mcp.server import MCPServer
    from roarm_mcp.mcp.isaac_sim_handler import IsaacSimMCPHandler


async def run_isaac_sim_server(
    host: str,
    port: int,
    robot_type: str,
    headless: bool
) -> None:
    """Run the MCP server with Isaac Sim integration.
    
    Args:
        host: The host to bind to.
        port: The port to bind to.
        robot_type: The type of robot to use ("ur10" or "franka").
        headless: Whether to run Isaac Sim in headless mode.
    """
    logger.info("🤖 Isaac Sim MCP 서버 시작 중...")
    
    # Create server
    server = MCPServer(host=host, port=port)
    
    # Create Isaac Sim handler
    handler = IsaacSimMCPHandler(robot_type=robot_type, headless=headless)
    
    # Register handler with server
    server.register_env_handler(handler)
    
    # Start server
    await server.start()
    logger.info(f"🚀 Isaac Sim MCP 서버 실행 중: {host}:{port}")
    logger.info(f"📡 로봇 타입: {robot_type.upper()}")
    logger.info(f"👀 헤드리스 모드: {'ON' if headless else 'OFF'}")
    
    try:
        # Keep the server running
        while server.running:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        logger.info("🛑 서버 종료 신호 수신...")
    finally:
        # Cleanup
        if handler.is_initialized:
            await handler.close()
        await server.stop()
        logger.info("✅ Isaac Sim MCP 서버 종료 완료")


def main():
    """Main function to parse arguments and run the server."""
    parser = argparse.ArgumentParser(
        description="Run RoArm MCP server with Isaac Sim integration"
    )
    parser.add_argument(
        "--host",
        type=str,
        default="localhost",
        help="Host to bind the server to (default: localhost)"
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8765,
        help="Port to bind the server to (default: 8765)"
    )
    parser.add_argument(
        "--robot-type",
        type=str,
        choices=["ur10", "franka"],
        default="ur10",
        help="Type of robot to use (default: ur10)"
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run Isaac Sim in headless mode (default: False)"
    )
    parser.add_argument(
        "--isaac-sim-path",
        type=str,
        default="/home/roarm_m3/isaac_sim",
        help="Path to Isaac Sim installation (default: /home/roarm_m3/isaac_sim)"
    )

    args = parser.parse_args()

    # Check Isaac Sim installation
    if not os.path.exists(args.isaac_sim_path):
        logger.error(f"❌ Isaac Sim이 경로에 없습니다: {args.isaac_sim_path}")
        logger.error("Isaac Sim 설치 후 올바른 경로를 지정하세요.")
        sys.exit(1)

    # Add Isaac Sim Python path
    isaac_python_path = os.path.join(args.isaac_sim_path, "python.sh")
    if not os.path.exists(isaac_python_path):
        logger.error(f"❌ Isaac Sim Python 실행파일이 없습니다: {isaac_python_path}")
        logger.error("Isaac Sim python.sh를 사용하여 이 스크립트를 실행하세요:")
        logger.error(f"cd {args.isaac_sim_path}")
        logger.error(f"./python.sh /home/roarm_m3/dev_roarm/roarm_mcp/examples/run_isaac_sim_server.py")
        sys.exit(1)

    logger.info("✅ Isaac Sim 환경 확인 완료")

    # Run the server
    try:
        asyncio.run(run_isaac_sim_server(
            host=args.host,
            port=args.port,
            robot_type=args.robot_type,
            headless=args.headless
        ))
    except Exception as e:
        logger.error(f"❌ 서버 실행 중 오류 발생: {e}")
        import traceback
        logger.error(traceback.format_exc())
        sys.exit(1)


if __name__ == "__main__":
    main()