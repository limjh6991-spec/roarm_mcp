# Isaac Sim 확장 모듈 구현 계획

## 1. 개요

Isaac Sim 확장 모듈은 MCP 서버와 Isaac Sim 사이의 인터페이스 역할을 수행하며, 로봇팔 모델 로딩, 시뮬레이션 제어, 센서 데이터 수집 등의 기능을 제공합니다. 이 문서는 해당 확장 모듈의 구현 방법과 계획을 설명합니다.

## 2. 확장 모듈 구조

### 2.1 디렉토리 구조

```
isaac.sim.mcp_extension/
├── config/
│   └── extension.toml       # 확장 설정 파일
├── isaac_sim_mcp_extension/
│   ├── __init__.py          # 패키지 초기화
│   ├── extension.py         # 확장 모듈 메인 클래스
│   ├── robot_control.py     # 로봇 제어 관련 기능
│   ├── simulation.py        # 시뮬레이션 관련 기능
│   └── utils/
│       ├── __init__.py
│       ├── usd.py           # USD 관련 유틸리티
│       └── viz.py           # 시각화 관련 유틸리티
└── examples/
    ├── franka_arm.py        # Franka 로봇팔 예제
    └── custom_robot.py      # 사용자 정의 로봇 예제
```

### 2.2 주요 클래스 및 모듈

1. **확장 모듈 메인 클래스**:
   ```python
   import omni.ext
   import omni.ui as ui
   import socket
   import json
   import threading
   import asyncio
   import carb
   
   class IsaacSimMCPExtension(omni.ext.IExt):
       """Isaac Sim MCP 확장 모듈의 메인 클래스"""
       
       def on_startup(self, ext_id):
           """확장 모듈 시작 시 호출되는 메서드"""
           print("Isaac Sim MCP 확장 모듈을 시작합니다...")
           self._socket_server = None
           self._server_thread = None
           self._running = False
           self._world = None
           
           # 소켓 서버 설정
           self._host = "localhost"
           self._port = 8766
           
           # UI 설정 (필요시)
           self._build_ui()
           
           # 서버 시작
           self._start_socket_server()
       
       def on_shutdown(self):
           """확장 모듈 종료 시 호출되는 메서드"""
           print("Isaac Sim MCP 확장 모듈을 종료합니다...")
           self._running = False
           if self._socket_server:
               self._socket_server.close()
           
           if self._server_thread and self._server_thread.is_alive():
               self._server_thread.join(timeout=5.0)
       
       def _build_ui(self):
           """UI 구성 메서드 (옵션)"""
           self._window = ui.Window("Isaac Sim MCP", width=300, height=200)
           with self._window.frame:
               with ui.VStack(spacing=5):
                   ui.Label("Isaac Sim MCP 서버 상태")
                   self._status_label = ui.Label("연결 대기 중...")
       
       def _start_socket_server(self):
           """소켓 서버 시작"""
           self._running = True
           self._server_thread = threading.Thread(target=self._run_server)
           self._server_thread.daemon = True
           self._server_thread.start()
       
       def _run_server(self):
           """소켓 서버 실행 로직"""
           try:
               self._socket_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
               self._socket_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
               self._socket_server.bind((self._host, self._port))
               self._socket_server.listen(5)
               print(f"Isaac Sim MCP 서버가 {self._host}:{self._port}에서 시작되었습니다.")
               
               while self._running:
                   client_sock, addr = self._socket_server.accept()
                   print(f"클라이언트 연결: {addr}")
                   client_handler = threading.Thread(
                       target=self._handle_client_connection,
                       args=(client_sock,)
                   )
                   client_handler.daemon = True
                   client_handler.start()
           
           except Exception as e:
               print(f"소켓 서버 오류: {str(e)}")
           finally:
               if self._socket_server:
                   self._socket_server.close()
       
       def _handle_client_connection(self, client_sock):
           """클라이언트 연결 처리"""
           try:
               while self._running:
                   # 데이터 수신
                   data = self._receive_data(client_sock)
                   if not data:
                       break
                   
                   # 명령 처리
                   response = self._process_command(data)
                   
                   # 응답 전송
                   self._send_response(client_sock, response)
           
           except Exception as e:
               print(f"클라이언트 처리 오류: {str(e)}")
           finally:
               client_sock.close()
       
       def _receive_data(self, sock):
           """데이터 수신 로직"""
           # 구현
       
       def _send_response(self, sock, response):
           """응답 전송 로직"""
           # 구현
       
       def _process_command(self, data):
           """명령 처리 로직"""
           try:
               command = json.loads(data.decode('utf-8'))
               cmd_type = command.get("type", "")
               
               if cmd_type == "get_scene_info":
                   return self._handle_get_scene_info()
               elif cmd_type == "create_robot":
                   return self._handle_create_robot(command)
               elif cmd_type == "execute_script":
                   return self._handle_execute_script(command)
               else:
                   return {"status": "error", "message": f"알 수 없는 명령: {cmd_type}"}
           
           except Exception as e:
               return {"status": "error", "message": f"명령 처리 오류: {str(e)}"}
       
       # 명령 핸들러 메서드들...
   ```

2. **로봇 제어 모듈**:
   ```python
   from omni.isaac.core.articulations import Articulation
   import omni.kit.commands
   from omni.isaac.core.utils.prims import create_prim
   from omni.isaac.core.utils.stage import add_reference_to_stage
   import numpy as np
   
   class RobotControl:
       """로봇 제어를 위한 클래스"""
       
       def __init__(self):
           """초기화"""
           self.robots = {}  # 로봇 인스턴스 저장
           
       def create_robot(self, robot_type, position, name=None):
           """로봇 생성"""
           if robot_type == "franka":
               return self._create_franka(position, name)
           elif robot_type == "ur10":
               return self._create_ur10(position, name)
           else:
               raise ValueError(f"지원되지 않는 로봇 유형: {robot_type}")
           
       def _create_franka(self, position, name=None):
           """Franka 로봇 생성"""
           # 구현
           
       def _create_ur10(self, position, name=None):
           """UR10 로봇 생성"""
           # 구현
           
       def set_joint_positions(self, robot_name, joint_positions):
           """관절 위치 설정"""
           # 구현
           
       def get_joint_positions(self, robot_name):
           """관절 위치 조회"""
           # 구현
           
       def move_to_cartesian_pose(self, robot_name, position, orientation):
           """카르테시안 위치로 이동"""
           # 구현
   ```

3. **시뮬레이션 모듈**:
   ```python
   from omni.isaac.core import SimulationContext
   from omni.isaac.core import World
   
   class SimulationManager:
       """시뮬레이션 관리를 위한 클래스"""
       
       def __init__(self):
           """초기화"""
           self.world = None
           self.sim_context = None
           
       def initialize_simulation(self):
           """시뮬레이션 초기화"""
           if self.world is None:
               self.world = World()
               self.sim_context = SimulationContext()
           
       def step_simulation(self, steps=1):
           """시뮬레이션 단계 실행"""
           if self.sim_context:
               for _ in range(steps):
                   self.sim_context.step()
           
       def stop_simulation(self):
           """시뮬레이션 정지"""
           if self.sim_context:
               self.sim_context.stop()
           
       def get_scene_info(self):
           """씬 정보 조회"""
           # 구현
   ```

## 3. 구현 계획

### 3.1 단계별 구현 계획

1. **기본 확장 모듈 구조 설정** (1일)
   - 확장 디렉토리 구조 생성
   - 기본 클래스 구현

2. **소켓 통신 구현** (1일)
   - 소켓 서버 설정
   - 비동기 메시지 처리
   - 에러 핸들링

3. **로봇 제어 기능 구현** (2일)
   - 로봇 모델 로드 기능
   - 관절 위치 제어 기능
   - 카르테시안 위치 제어 기능

4. **시뮬레이션 관리 기능 구현** (1일)
   - 시뮬레이션 초기화
   - 시뮬레이션 단계 실행
   - 시뮬레이션 상태 관리

5. **센서 데이터 수집 기능 구현** (1일)
   - 카메라 데이터 수집
   - 관절 센서 데이터 수집
   - 충돌 감지 기능

6. **UI 및 시각화 기능 구현** (1일)
   - 상태 모니터링 UI
   - 디버깅 정보 표시

7. **테스트 및 문서화** (1일)
   - 기능 테스트
   - 예제 코드 작성
   - 문서 작성

### 3.2 구현 우선순위

1. 소켓 통신 기본 기능 (필수)
2. 로봇 모델 로드 기능 (필수)
3. 기본 제어 기능 (필수)
4. 시뮬레이션 관리 기능 (필수)
5. 센서 데이터 수집 기능 (중요)
6. UI 및 시각화 기능 (옵션)

## 4. 주요 기능 상세 설계

### 4.1 로봇 모델 로드 및 제어

1. **로봇 모델 리소스**:
   - Franka Emika Panda 로봇 USD 파일
   - UR10 로봇 USD 파일
   - 사용자 정의 로봇 USD 파일 지원

2. **관절 제어 방법**:
   - 위치 제어 (Position Control)
   - 속도 제어 (Velocity Control)
   - 토크 제어 (Torque Control)

3. **역기구학 계산**:
   - 카르테시안 위치에서 관절 위치로 변환
   - 다양한 IK 알고리즘 지원

### 4.2 센서 데이터 수집

1. **지원 센서 유형**:
   - RGB 카메라
   - 깊이 카메라
   - 관절 위치/속도 센서
   - 힘/토크 센서

2. **데이터 형식**:
   - 이미지: Base64 인코딩 또는 파일 저장
   - 수치 데이터: JSON 형식

### 4.3 명령어 인터페이스

1. **기본 명령어**:
   - `get_scene_info`: 씬 정보 조회
   - `create_robot`: 로봇 생성
   - `set_joint_positions`: 관절 위치 설정
   - `get_joint_positions`: 관절 위치 조회
   - `step_simulation`: 시뮬레이션 단계 실행
   - `execute_script`: 스크립트 실행

2. **명령어 형식**:
   ```json
   {
     "type": "command_type",
     "parameters": {
       "param1": "value1",
       "param2": "value2"
     }
   }
   ```

3. **응답 형식**:
   ```json
   {
     "status": "success|error",
     "message": "상태 메시지",
     "data": {
       "key1": "value1",
       "key2": "value2"
     }
   }
   ```

## 5. 테스트 계획

1. **단위 테스트**:
   - 소켓 통신 테스트
   - 명령어 처리 테스트
   - 로봇 제어 기능 테스트

2. **통합 테스트**:
   - MCP 서버와의 통신 테스트
   - 전체 시스템 테스트

3. **성능 테스트**:
   - 시뮬레이션 성능 측정
   - 통신 지연 측정

## 6. 개발 환경 및 도구

1. **필수 소프트웨어**:
   - NVIDIA Isaac Sim
   - Python 3.9+
   - VS Code 또는 기타 IDE

2. **개발 프로세스**:
   - 코드 버전 관리: Git
   - 문서화: Markdown
   - 코드 스타일: PEP 8

## 7. 위험 요소 및 해결 방안

1. **성능 이슈**:
   - 위험: 복잡한 시뮬레이션으로 인한 성능 저하
   - 해결 방안: 시뮬레이션 최적화, 단순화된 모델 사용

2. **버전 호환성**:
   - 위험: Isaac Sim 버전 업데이트로 인한 API 변경
   - 해결 방안: 버전 종속성 명확히 문서화, 호환성 레이어 구현

3. **디버깅 어려움**:
   - 위험: 실시간 시뮬레이션 환경에서의 디버깅 어려움
   - 해결 방안: 로깅 시스템 강화, 디버깅 UI 구현