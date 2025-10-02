# RoArm MCP: Robot Arm Model Context Protocol

RoArm MCP는 NVIDIA Isaac Sim을 사용하여 로봇 팔 강화 학습을 위한 Model Context Protocol (MCP) 서버 및 클라이언트 구현입니다.

## 소개

이 프로젝트는 로봇 팔 제어를 위한 강화 학습 환경을 제공합니다. MCP(Model Context Protocol)를 사용하여 클라이언트와 서버 간의 통신을 구현하고, NVIDIA Isaac Sim을 통해 물리 기반 시뮬레이션을 제공합니다.

주요 기능:
- **Isaac Sim 5.0 PhysX Tensors API 완전 지원** 🆕
- Isaac Sim을 사용한 물리 기반 로봇 팔 시뮬레이션
- MCP 프로토콜을 통한 서버-클라이언트 통신
- UR10 및 Franka Emika Panda 로봇 팔 지원
- 클라우드 호스팅 에셋 로딩 (HTTPS 기반)
- 관절 위치 제어 및 엔드 이펙터 위치 제어를 위한 환경
- gymnasium과 호환되는 강화 학습 인터페이스

## 🚀 Isaac Sim 5.0 PhysX Tensors 솔루션 

### 주요 특징
- ✅ **중첩된 ArticulationRoot 정리**: 한 서브트리에 Root 1개 원칙 적용
- ✅ **올바른 World.reset() 순서**: PhysX 상태 초기화 후 ArticulationView 생성
- ✅ **PhysX Tensors SimulationView**: `omni.physics.tensors.create_simulation_view()` 사용
- ✅ **Torch Tensor Indices**: Isaac Sim 5.0 API 완전 호환성
- ✅ **클라우드 Asset 지원**: HTTPS 기반 USD 로딩

### 해결된 문제점
1. **"Nested articulation roots are not allowed"** → ArticulationRoot API로 중첩 루트 정리
2. **"'NoneType' object has no attribute 'to'"** → 올바른 torch tensor indices 사용
3. **"Asset loading failures"** → 클라우드 호스팅 에셋 경로로 수정

### 빠른 실행
```bash
cd /home/roarm_m3/isaac_sim
./python.sh /home/roarm_m3/dev_roarm/roarm_mcp/isaac_sim_integration/solutions/isaac_sim_physx_tensors_solution.py
```

자세한 내용은 [`isaac_sim_integration/`](./isaac_sim_integration/) 디렉토리를 참조하세요.

## 설치

### 사전 요구 사항

- Python 3.11+ (Isaac Sim 5.0 호환)
- **NVIDIA Isaac Sim 5.0** ([설치 가이드](https://developer.nvidia.com/isaac-sim))
- CUDA 지원 그래픽카드 (RTX 시리즈 권장)
- torch (Isaac Sim에 포함됨)

### 설치 과정

1. 이 저장소를 복제합니다:
```bash
git clone https://github.com/omni-mcp/isaac-sim-mcp.git
cd isaac-sim-mcp
```

2. Python 가상 환경을 생성하고 활성화합니다:
```bash
python -m venv venv
source venv/bin/activate  # Linux/macOS
# venv\Scripts\activate  # Windows
```

3. 필요한 패키지를 설치합니다:
```bash
pip install -e .
```

## 사용 방법

### MCP 서버 실행

```bash
python -m roarm_mcp.examples.run_server --env-type joint_position --robot-type ur10
```

옵션:
- `--host`: 바인딩할 호스트 (기본값: localhost)
- `--port`: 바인딩할 포트 (기본값: 8765)
- `--env-type`: 환경 유형 (joint_position 또는 end_effector)
- `--robot-type`: 로봇 유형 (ur10 또는 franka)
- `--headless`: 헤드리스 모드로 실행

### 샘플 클라이언트 실행

```bash
python -m roarm_mcp.examples.sample_client
```

옵션:
- `--host`: 연결할 호스트 (기본값: localhost)
- `--port`: 연결할 포트 (기본값: 8765)
- `--episodes`: 실행할 에피소드 수 (기본값: 5)
- `--steps`: 에피소드당 스텝 수 (기본값: 100)
- `--no-render`: 렌더링 비활성화

## 🚀 Isaac Sim 5.0 PhysX Tensors 솔루션

**최신 Isaac Sim 5.0 완전 호환 솔루션**이 완성되었습니다!

### 주요 해결 사항:
1. ✅ **중첩된 ArticulationRoot 정리**: "한 서브트리에 Root 1개" 원칙
2. ✅ **올바른 World.reset() 순서**: PhysX 상태 초기화 후 뷰 생성
3. ✅ **PhysX Tensors SimulationView**: `omni.physics.tensors.create_simulation_view()`
4. ✅ **Torch Tensor Indices**: Isaac Sim 5.0 API 호환성
5. ✅ **클라우드 Asset 지원**: HTTPS 기반 USD 로딩

### 실행 방법:
```bash
cd /home/roarm_m3/isaac_sim
./python.sh /path/to/roarm_mcp/isaac_sim_integration/solutions/isaac_sim_physx_tensors_solution.py
```

### 테스트 결과:
- **로드된 로봇**: UR10 (6 DOF), Franka (9 DOF)
- **ArticulationRoot 정리**: 성공 ✅
- **SimulationView 생성**: 성공 ✅
- **ArticulationView 생성**: 성공 ✅
- **제어 API 호환성**: Isaac Sim 5.0 완전 지원 ✅

📁 **솔루션 위치**: `isaac_sim_integration/` 디렉토리에서 확인 가능

## 프로젝트 구조

```
roarm_mcp/
├── __init__.py             # 패키지 초기화
├── isaac_sim_integration/  # 🆕 Isaac Sim 5.0 완성 솔루션
│   ├── solutions/          # PhysX Tensors 솔루션
│   ├── outputs/           # 생성된 USD 파일들
│   └── README.md          # 솔루션 문서
├── mcp/                    # MCP 프로토콜 및 서버 구현
│   ├── __init__.py
│   ├── protocol.py         # MCP 프로토콜 정의
│   ├── server.py           # MCP 서버 구현
│   ├── client.py           # MCP 클라이언트 구현
│   └── env_handler.py      # 환경 핸들러
├── isaac_sim/              # Isaac Sim 통합
│   ├── __init__.py
│   └── simulator.py        # Isaac Sim 시뮬레이션 래퍼
├── robot/                  # 로봇 제어
│   ├── __init__.py
│   ├── controller.py       # 로봇 제어 인터페이스
│   └── arms.py             # 특정 로봇 구현
├── envs/                   # 강화 학습 환경
│   ├── __init__.py
│   └── robot_env.py        # 로봇 환경 구현
├── tests/                  # 정리된 테스트
├── tests_archive/          # 아카이브된 이전 테스트들
└── examples/               # 예제 코드
    ├── __init__.py
    ├── run_server.py       # 서버 실행 스크립트
    └── sample_client.py    # 샘플 클라이언트
```

## MCP 프로토콜

MCP(Model Context Protocol)는 강화 학습 환경과 에이전트 간의 통신을 위한 프로토콜입니다. 이 구현에서는 다음과 같은 메시지 유형을 지원합니다:

### 제어 메시지

- `RESET`: 환경 초기화
- `STEP`: 환경에서 액션 실행
- `RENDER`: 환경 렌더링
- `CLOSE`: 환경 종료

### 정보 메시지

- `ACTION_SPACE`: 액션 공간 요청
- `OBSERVATION_SPACE`: 관찰 공간 요청

### 응답 메시지

- `OBSERVATION`: 관찰 값
- `REWARD`: 보상 값
- `TERMINATED`: 종료 플래그
- `TRUNCATED`: 중단 플래그
- `INFO`: 추가 정보

### 오류 메시지

- `ERROR`: 오류 메시지

## API 문서

### `roarm_mcp.mcp.server.MCPServer`

MCP 서버 구현입니다.

```python
server = MCPServer(host="localhost", port=8765)
await server.start()
```

### `roarm_mcp.mcp.client.MCPClient`

MCP 클라이언트 구현입니다.

```python
client = MCPClient(host="localhost", port=8765)
observation = client.reset()
observation, reward, terminated, truncated, info = client.step(action)
```

### `roarm_mcp.isaac_sim.simulator.IsaacSimRobotEnv`

Isaac Sim 로봇 환경 래퍼입니다.

```python
env = IsaacSimRobotEnv(robot_usd_path="/path/to/robot.usd", robot_name="robot")
```

### `roarm_mcp.robot.arms.UR10Robot` / `roarm_mcp.robot.arms.FrankaRobot`

특정 로봇 구현입니다.

```python
robot = UR10Robot(robot_env=env)
robot.move_to_home()
```

### `roarm_mcp.envs.robot_env.JointPositionEnv` / `roarm_mcp.envs.robot_env.EndEffectorPositionEnv`

강화 학습 환경 구현입니다.

```python
env = JointPositionEnv(robot_type="ur10", headless=False)
observation = env.reset()
observation, reward, terminated, truncated, info = env.step(action)
```

## 라이센스

MIT 라이센스. 자세한 내용은 LICENSE 파일을 참조하세요.

## 참고 문헌 및 링크

- [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim)
- [Model Context Protocol](https://github.com/google-deepmind/mcp)
- [Gymnasium](https://gymnasium.farama.org/)
- [Stable-Baselines3](https://github.com/DLR-RM/stable-baselines3)