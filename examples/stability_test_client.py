#!/usr/bin/env python3
"""
Advanced test client for stability and error handling
안정성 및 에러 처리 테스트용 고급 클라이언트
"""

import asyncio
import websockets
import json
import logging
import time
import random

# Initialize logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class StabilityTestClient:
    def __init__(self, server_url):
        self.server_url = server_url
        
    async def test_connection_disconnect(self):
        """연결 해제 시나리오 테스트"""
        logger.info("🔌 연결/해제 테스트 시작")
        
        for i in range(3):
            try:
                async with websockets.connect(self.server_url) as websocket:
                    logger.info(f"✅ 연결 {i+1}/3 성공")
                    
                    # 짧은 메시지 전송
                    message = {"action": "ping", "test": f"connection_{i+1}"}
                    await websocket.send(json.dumps(message))
                    response = await websocket.recv()
                    
                    logger.info(f"📨 응답 수신: {json.loads(response)['status']}")
                    await asyncio.sleep(1)
                    
                logger.info(f"🔌 연결 {i+1} 정상 종료")
                    
            except Exception as e:
                logger.error(f"❌ 연결 {i+1} 오류: {e}")
                
    async def test_invalid_messages(self):
        """잘못된 메시지 처리 테스트"""
        logger.info("⚠️ 잘못된 메시지 테스트 시작")
        
        invalid_messages = [
            "invalid json",  # 잘못된 JSON
            '{"incomplete": true',  # 불완전한 JSON
            '{"empty": ""}',  # 빈 액션
            '{"action": "unknown_command"}',  # 알 수 없는 명령
            '{"action": null}',  # null 액션
        ]
        
        try:
            async with websockets.connect(self.server_url) as websocket:
                logger.info("✅ 서버 연결 성공")
                
                for i, invalid_msg in enumerate(invalid_messages, 1):
                    logger.info(f"📤 잘못된 메시지 {i}/5 전송: {invalid_msg[:30]}...")
                    
                    try:
                        await websocket.send(invalid_msg)
                        response = await websocket.recv()
                        data = json.loads(response)
                        logger.info(f"📨 서버 응답: {data.get('status', 'unknown')}")
                    except Exception as e:
                        logger.warning(f"⚠️ 메시지 {i} 처리 오류: {e}")
                        
                    await asyncio.sleep(0.5)
                    
        except Exception as e:
            logger.error(f"❌ 잘못된 메시지 테스트 오류: {e}")
            
    async def test_rapid_messages(self):
        """빠른 연속 메시지 테스트"""
        logger.info("🚀 빠른 연속 메시지 테스트 시작")
        
        try:
            async with websockets.connect(self.server_url) as websocket:
                logger.info("✅ 서버 연결 성공")
                
                # 20개 메시지 빠르게 전송
                for i in range(20):
                    message = {
                        "action": "rapid_test",
                        "sequence": i + 1,
                        "timestamp": time.time()
                    }
                    
                    await websocket.send(json.dumps(message))
                    
                    # 10개마다 응답 확인
                    if (i + 1) % 10 == 0:
                        logger.info(f"📤 {i+1}/20 메시지 전송 완료")
                        
                        # 응답 수집
                        for _ in range(10):
                            try:
                                response = await asyncio.wait_for(websocket.recv(), timeout=1.0)
                                data = json.loads(response)
                                if data.get('status') != 'success':
                                    logger.warning(f"⚠️ 비정상 응답: {data}")
                            except asyncio.TimeoutError:
                                logger.warning(f"⚠️ 응답 시간 초과")
                                break
                                
                        logger.info(f"✅ {i+1}번째까지 처리 완료")
                        
        except Exception as e:
            logger.error(f"❌ 빠른 메시지 테스트 오류: {e}")
            
    async def test_long_connection(self):
        """장시간 연결 테스트"""
        logger.info("⏱️ 장시간 연결 테스트 시작 (30초)")
        
        try:
            async with websockets.connect(self.server_url) as websocket:
                logger.info("✅ 장시간 연결 시작")
                
                start_time = time.time()
                message_count = 0
                
                while time.time() - start_time < 30:  # 30초 동안
                    message = {
                        "action": "heartbeat",
                        "timestamp": time.time(),
                        "uptime": time.time() - start_time
                    }
                    
                    await websocket.send(json.dumps(message))
                    response = await websocket.recv()
                    
                    message_count += 1
                    
                    if message_count % 5 == 0:
                        logger.info(f"💗 하트비트 {message_count}개 전송 (경과: {time.time() - start_time:.1f}초)")
                        
                    await asyncio.sleep(2)  # 2초마다
                    
                logger.info(f"✅ 장시간 연결 테스트 완료: {message_count}개 메시지, {time.time() - start_time:.1f}초")
                
        except Exception as e:
            logger.error(f"❌ 장시간 연결 테스트 오류: {e}")

    async def run_all_tests(self):
        """모든 테스트 실행"""
        logger.info("🧪 === 종합 안정성 테스트 시작 ===")
        
        tests = [
            ("연결/해제 테스트", self.test_connection_disconnect),
            ("잘못된 메시지 테스트", self.test_invalid_messages),
            ("빠른 연속 메시지 테스트", self.test_rapid_messages),
            ("장시간 연결 테스트", self.test_long_connection),
        ]
        
        for test_name, test_func in tests:
            logger.info(f"\n🔬 {test_name} 실행 중...")
            try:
                await test_func()
                logger.info(f"✅ {test_name} 완료")
            except Exception as e:
                logger.error(f"❌ {test_name} 실패: {e}")
            
            await asyncio.sleep(2)  # 테스트 간 대기
            
        logger.info("\n🎉 === 모든 안정성 테스트 완료 ===")

async def main():
    """메인 함수"""
    client = StabilityTestClient("ws://localhost:8765")
    await client.run_all_tests()

if __name__ == "__main__":
    asyncio.run(main())