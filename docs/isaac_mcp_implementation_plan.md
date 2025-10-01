# Isaac Sim MCP 서버 구현 계획

## 1. 목표

- FastMCP 기반 Isaac Sim MCP 서버 구현
- Isaac Sim과의 효율적 통신을 위한 확장 모듈 개발
- 기존 로봇팔 강화학습 코드의 Isaac Sim 환경으로 마이그레이션

## 2. 구현 단계

### 2.1 기본 구조 설정

1. **프로젝트 구조 재구성**:
   - `isaac_sim_mcp/`: Isaac Sim MCP 관련 코드 디렉토리
   - `isaac_sim_mcp/server.py`: FastMCP 기반 서버 구현
   - `isaac_sim_mcp/extensions/`: Isaac Sim 확장 모듈 코드

2. **의존성 설정**:
   - `requirements.txt` 업데이트: FastMCP, 필요한 패키지 추가
   - `setup.py` 업데이트: Isaac Sim 확장을 위한 설정

### 2.2 MCP 서버 구현

1. **FastMCP 기반 서버 개발**:
   ```python
   from mcp.server.fastmcp import FastMCP, Context, Image
   import socket
   import json
   import asyncio
   import logging
   from dataclasses import dataclass
   
   # Isaac Sim 연결 클래스 정의
   @dataclass
   class IsaacConnection:
       host: str
       port: int
       sock: socket.socket = None
       
       # 연결 메서드 구현
       def connect(self) -> bool:
           # 구현
   
       # 연결 해제 메서드 구현
       def disconnect(self):
           # 구현
   
       # 응답 수신 메서드 구현
       def receive_full_response(self, sock, buffer_size=16384):
           # 구현
   
   # MCP 서버 라이프사이클 관리
   @asynccontextmanager
   async def lifespan(app: FastMCP) -> AsyncIterator[Dict[str, Any]]:
       # 초기화 로직
       # 서버 시작시 Isaac Sim 연결 설정
       isaac = IsaacConnection(host="localhost", port=8766)
       
       try:
           yield {"isaac": isaac}
       finally:
           # 서버 종료시 연결 해제
           if isaac:
               isaac.disconnect()
   
   # FastMCP 인스턴스 생성
   app = FastMCP(lifespan=lifespan)
   
   # 핸들러 정의
   @app.tool("get_scene_info")
   async def get_scene_info(context: Context):
       # Isaac Sim 씬 정보 조회 구현
   
   @app.tool("create_robot")
   async def create_robot(context: Context, robot_type: str, position: list):
       # 로봇 생성 구현
   ```

2. **통신 모듈 구현**:
   - 소켓 기반 통신 구현
   - 비동기 메시지 처리
   - 에러 핸들링

### 2.3 Isaac Sim 확장 모듈

1. **확장 모듈 구조**:
   ```python
   import omni.ext
   import omni.ui as ui
   import socket
   import json
   import threading
   
   class IsaacSimMCPExtension(omni.ext.IExt):
       def on_startup(self, ext_id):
           # 확장 시작시 호출
           # 소켓 서버 시작
           self._start_socket_server()
   
       def on_shutdown(self):
           # 확장 종료시 호출
           # 소켓 서버 종료
           self._stop_socket_server()
   
       def _start_socket_server(self):
           # 소켓 서버 시작
           self._server_thread = threading.Thread(target=self._run_server)
           self._server_thread.daemon = True
           self._server_thread.start()
   
       def _run_server(self):
           # 소켓 서버 실행
           # 명령 수신 및 처리 로직
   ```

2. **로봇 모델 로딩 및 제어 기능**:
   - 로봇 모델 로드 기능
   - 관절 제어 기능
   - 센서 데이터 수집 기능

### 2.4 강화학습 환경 통합

1. **Isaac Sim 기반 강화학습 환경**:
   ```python
   import gym
   import numpy as np
   from omni.isaac.core import SimulationContext
   
   class IsaacRoArmEnv(gym.Env):
       def __init__(self, robot_type="franka"):
           # 환경 초기화
           # 액션 및 관찰 공간 정의
           self.action_space = gym.spaces.Box(low=-1, high=1, shape=(7,))
           self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(23,))
           
           # 시뮬레이션 컨텍스트 설정
           self.sim = SimulationContext()
           
           # 로봇 로드
           self._load_robot(robot_type)
   
       def reset(self):
           # 환경 리셋
           # 상태 초기화
           return self._get_observation()
   
       def step(self, action):
           # 액션 적용
           # 상태 관찰
           # 보상 계산
           # 종료 조건 확인
           return self._get_observation(), reward, done, info
   
       def _get_observation(self):
           # 관찰 데이터 수집
   
       def _load_robot(self, robot_type):
           # 로봇 모델 로드
   ```

2. **기존 강화학습 알고리즘과 통합**:
   - 강화학습 에이전트 연결
   - 보상 함수 마이그레이션
   - 액션 및 관찰 공간 매핑

## 3. 테스트 계획

1. **단위 테스트**:
   - 소켓 통신 테스트
   - 명령어 처리 테스트
   - 로봇 제어 기능 테스트

2. **통합 테스트**:
   - MCP 서버와 Isaac Sim 연동 테스트
   - 강화학습 환경 테스트
   - 전체 시스템 테스트

## 4. 일정

| 단계 | 작업 | 예상 소요 기간 |
|-----|-----|-------------|
| 1 | 기본 구조 설정 | 1일 |
| 2 | MCP 서버 구현 | 3일 |
| 3 | Isaac Sim 확장 모듈 개발 | 3일 |
| 4 | 강화학습 환경 통합 | 2일 |
| 5 | 테스트 및 버그 수정 | 2일 |

## 5. 필요 자원

1. **소프트웨어**:
   - NVIDIA Isaac Sim
   - Python 3.9+
   - FastMCP
   - mcp[cli]
   - uv/uvx

2. **하드웨어**:
   - NVIDIA RTX GPU
   - 충분한 메모리 (32GB+ 권장)

## 6. 위험 요소 및 대응 방안

1. **Isaac Sim 성능 이슈**:
   - 위험: 3D 시뮬레이션으로 인한 성능 저하
   - 대응: 시뮬레이션 최적화, 하드웨어 업그레이드

2. **호환성 문제**:
   - 위험: Isaac Sim 버전별 API 변경 가능성
   - 대응: 버전 종속성 명확히 문서화, 적응형 코드 작성

3. **학습 곡선**:
   - 위험: Isaac Sim API 학습에 시간 소요
   - 대응: 사전 학습 및 예제 코드 분석 시간 확보