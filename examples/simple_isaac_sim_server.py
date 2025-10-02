#!/usr/bin/env python3
"""
Simplified Isaac Sim MCP Server for testing.
단순화된 Isaac Sim MCP 서버 테스트용
"""

import asyncio
import websockets
import json
import logging
import sys
import os

# Initialize logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Add project path
project_root = '/home/roarm_m3/dev_roarm/roarm_mcp'
if project_root not in sys.path:
    sys.path.insert(0, project_root)

class SimpleIsaacSimMCPServer:
    def __init__(self):
        self.clients = set()
        self.isaac_sim_app = None
        
    async def register_client(self, websocket):
        """클라이언트 등록"""
        self.clients.add(websocket)
        logger.info(f"✅ 클라이언트 연결: {websocket.remote_address}")
        
    async def unregister_client(self, websocket):
        """클라이언트 해제"""
        self.clients.discard(websocket)
        logger.info(f"❌ 클라이언트 연결 해제: {websocket.remote_address}")
        
    async def handle_message(self, websocket, message):
        """메시지 처리"""
        try:
            data = json.loads(message)
            logger.info(f"📨 수신 메시지: {data.get('action', 'unknown')}")
            
            # 간단한 응답 생성
            response = {
                "status": "success",
                "message": f"Received action: {data.get('action', 'unknown')}",
                "data": {
                    "robot_count": 2,
                    "simulation_running": True
                }
            }
            
            await websocket.send(json.dumps(response))
            logger.info(f"📤 응답 전송 완료")
            
        except json.JSONDecodeError:
            error_response = {"status": "error", "message": "Invalid JSON"}
            await websocket.send(json.dumps(error_response))
            
    async def handle_client(self, websocket, path):
        """클라이언트 핸들러"""
        await self.register_client(websocket)
        try:
            async for message in websocket:
                await self.handle_message(websocket, message)
        except websockets.exceptions.ConnectionClosed:
            logger.info("클라이언트 연결이 정상적으로 종료되었습니다")
        except Exception as e:
            logger.error(f"클라이언트 처리 오류: {e}")
        finally:
            await self.unregister_client(websocket)
            
    async def start_server(self, host="localhost", port=8765):
        """서버 시작"""
        logger.info(f"🚀 간단한 MCP 서버 시작: {host}:{port}")
        
        async with websockets.serve(self.handle_client, host, port):
            logger.info(f"📡 WebSocket 서버 대기 중...")
            await asyncio.Future()  # run forever

async def main():
    """메인 함수"""
    server = SimpleIsaacSimMCPServer()
    try:
        await server.start_server()
    except KeyboardInterrupt:
        logger.info("⚡ 서버 종료")

if __name__ == "__main__":
    asyncio.run(main())