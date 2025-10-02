#!/usr/bin/env python3
"""
Sample client to test Isaac Sim MCP integration.

이 클라이언트는 Isaac Sim MCP 서버와 통신하여 로봇을 제어합니다.
"""

import asyncio
import json
import logging
import numpy as np
import argparse
import websockets
from typing import Dict, Any

# Initialize logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class IsaacSimMCPClient:
    """Isaac Sim MCP 클라이언트"""
    
    def __init__(self, server_url: str):
        """Initialize the client.
        
        Args:
            server_url: The WebSocket server URL.
        """
        self.server_url = server_url
        self.websocket = None
        
    async def connect(self) -> None:
        """Connect to the MCP server."""
        logger.info(f"🔌 서버 연결 중: {self.server_url}")
        self.websocket = await websockets.connect(self.server_url)
        logger.info("✅ 서버 연결 완료")
        
    async def disconnect(self) -> None:
        """Disconnect from the server."""
        if self.websocket:
            await self.websocket.close()
            logger.info("🔌 서버 연결 종료")
            
    async def send_message(self, message: Dict[str, Any]) -> Dict[str, Any]:
        """Send a message to the server and get response.
        
        Args:
            message: The message to send.
            
        Returns:
            The server response.
        """
        if not self.websocket:
            raise RuntimeError("Not connected to server")
            
        # Send message
        await self.websocket.send(json.dumps(message))
        
        # Receive response
        response = await self.websocket.recv()
        return json.loads(response)
        
    async def reset_environment(self) -> np.ndarray:
        """Reset the Isaac Sim environment.
        
        Returns:
            Initial observation.
        """
        logger.info("🔄 환경 리셋 중...")
        
        message = {"type": "reset"}
        response = await self.send_message(message)
        
        if response.get("type") == "observation":
            observation = np.array(response["data"])
            logger.info(f"✅ 초기 관찰값: {observation}")
            return observation
        else:
            raise RuntimeError(f"Unexpected response: {response}")
            
    async def step_environment(self, action: np.ndarray) -> tuple:
        """Take a step in the environment.
        
        Args:
            action: The action to take.
            
        Returns:
            (observation, reward, terminated, truncated, info)
        """
        message = {
            "type": "step",
            "action": action.tolist()
        }
        
        response = await self.send_message(message)
        
        observation = np.array(response.get("observation", []))
        reward = response.get("reward", 0.0)
        terminated = response.get("terminated", False)
        truncated = response.get("truncated", False)
        info = response.get("info", {})
        
        return observation, reward, terminated, truncated, info
        
    async def get_action_space(self) -> Dict[str, Any]:
        """Get the action space information."""
        message = {"type": "action_space"}
        response = await self.send_message(message)
        return response.get("space", {})
        
    async def get_observation_space(self) -> Dict[str, Any]:
        """Get the observation space information."""
        message = {"type": "observation_space"}
        response = await self.send_message(message)
        return response.get("space", {})


async def run_test_episode(
    client: IsaacSimMCPClient,
    steps: int = 50,
    robot_type: str = "ur10"
) -> None:
    """Run a test episode with the Isaac Sim environment.
    
    Args:
        client: The MCP client.
        steps: Number of steps to run.
        robot_type: Type of robot being controlled.
    """
    logger.info(f"🤖 {robot_type.upper()} 로봇 제어 테스트 시작")
    logger.info(f"📏 실행 스텝: {steps}")
    
    # Get space information
    try:
        action_space = await client.get_action_space()
        obs_space = await client.get_observation_space()
        
        logger.info(f"📊 액션 공간: {action_space}")
        logger.info(f"📊 관찰 공간: {obs_space}")
        
        # Extract DOF count from space info
        dof_count = len(action_space.get("high", []))
        if dof_count == 0:
            if robot_type == "ur10":
                dof_count = 6
            elif robot_type == "franka":
                dof_count = 9
            else:
                dof_count = 6  # default
                
        logger.info(f"🔧 로봇 DOF 수: {dof_count}")
        
    except Exception as e:
        logger.warning(f"⚠️ 공간 정보 조회 실패: {e}")
        # Use default values
        if robot_type == "ur10":
            dof_count = 6
        elif robot_type == "franka":
            dof_count = 9
        else:
            dof_count = 6
            
    # Reset environment
    try:
        initial_obs = await client.reset_environment()
        logger.info(f"✅ 환경 리셋 완료. 초기 관찰값 shape: {initial_obs.shape}")
    except Exception as e:
        logger.error(f"❌ 환경 리셋 실패: {e}")
        return
        
    # Run episode
    total_reward = 0.0
    
    for step in range(steps):
        try:
            # Generate random action (small movements)
            action = np.random.uniform(-0.1, 0.1, size=dof_count)
            
            # Take step
            obs, reward, terminated, truncated, info = await client.step_environment(action)
            total_reward += reward
            
            # Log progress
            if step % 10 == 0:
                logger.info(
                    f"🎯 스텝 {step:3d}: "
                    f"보상={reward:6.3f}, "
                    f"누적보상={total_reward:8.3f}, "
                    f"관찰값 shape={obs.shape}"
                )
                
            # Check termination
            if terminated or truncated:
                logger.info(f"🏁 에피소드 종료: terminated={terminated}, truncated={truncated}")
                break
                
        except Exception as e:
            logger.error(f"❌ 스텝 {step} 실행 중 오류: {e}")
            break
            
    logger.info(f"🏆 에피소드 완료! 총 보상: {total_reward:.3f}")


async def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        description="Isaac Sim MCP client test"
    )
    parser.add_argument(
        "--server-url",
        type=str,
        default="ws://localhost:8765",
        help="MCP server WebSocket URL (default: ws://localhost:8765)"
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=3,
        help="Number of episodes to run (default: 3)"
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=50,
        help="Steps per episode (default: 50)"
    )
    parser.add_argument(
        "--robot-type",
        type=str,
        choices=["ur10", "franka"],
        default="ur10",
        help="Robot type (default: ur10)"
    )

    args = parser.parse_args()

    # Create client
    client = IsaacSimMCPClient(args.server_url)
    
    try:
        # Connect to server
        await client.connect()
        
        # Run test episodes
        for episode in range(args.episodes):
            logger.info(f"\n🎬 에피소드 {episode + 1}/{args.episodes} 시작")
            await run_test_episode(client, args.steps, args.robot_type)
            
            # Wait between episodes
            if episode < args.episodes - 1:
                logger.info("⏰ 다음 에피소드까지 대기...")
                await asyncio.sleep(2)
                
    except KeyboardInterrupt:
        logger.info("🛑 사용자 중단")
    except Exception as e:
        logger.error(f"❌ 클라이언트 오류: {e}")
        import traceback
        logger.error(traceback.format_exc())
    finally:
        await client.disconnect()
        logger.info("👋 클라이언트 종료")


if __name__ == "__main__":
    asyncio.run(main())