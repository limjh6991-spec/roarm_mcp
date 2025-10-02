#!/usr/bin/env python3
"""
Simple test client for Isaac Sim MCP Server
간단한 Isaac Sim MCP 서버 테스트 클라이언트
"""

import asyncio
import websockets
import json
import logging

# Initialize logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class SimpleTestClient:
    def __init__(self, server_url):
        self.server_url = server_url
        
    async def test_connection(self):
        """연결 테스트"""
        try:
            async with websockets.connect(self.server_url) as websocket:
                logger.info(f"✅ 서버 연결 성공: {self.server_url}")
                
                # 테스트 메시지 전송
                test_messages = [
                    {"action": "initialize", "robot_type": "ur10"},
                    {"action": "get_status"},
                    {"action": "reset_environment"},
                    {"action": "get_robot_state"}
                ]
                
                for i, message in enumerate(test_messages, 1):
                    logger.info(f"📤 메시지 {i}/4 전송: {message['action']}")
                    await websocket.send(json.dumps(message))
                    
                    # 응답 대기
                    response = await websocket.recv()
                    data = json.loads(response)
                    logger.info(f"📨 응답 {i}/4: {data.get('status', 'unknown')}")
                    
                    await asyncio.sleep(1)  # 1초 대기
                    
                logger.info("🎉 모든 테스트 메시지 성공!")
                
        except Exception as e:
            logger.error(f"❌ 연결 오류: {e}")
            
async def main():
    """메인 함수"""
    client = SimpleTestClient("ws://localhost:8765")
    await client.test_connection()
    
if __name__ == "__main__":
    asyncio.run(main())