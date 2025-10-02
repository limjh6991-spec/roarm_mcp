# Isaac Sim MCP 연동 테스트

## 개요
Isaac Sim 5.0과 MCP(Model Context Protocol) 연동을 위한 테스트 슈트입니다.

## 테스트 파일 구조

### 기본 환경 테스트
- `test_isaac_sim_basic.py`: Isaac Sim 기본 동작 확인
- `test_isaac_sim_v5.py`: Isaac Sim 5.0 특화 기능 테스트

### MCP 연동 테스트
- `test_isaac_sim_mcp_integration.py`: **최종 완성된 Isaac Sim 5.0 MCP 연동 테스트**

### 로봇 모델 로딩 테스트
- `test_robot_models_final.py`: **UR10, Franka 로봇 모델 로딩 성공 테스트**
- `robot_models_test_v5_success.usd`: 로봇 모델이 로드된 USD 장면 파일
- `robot_loading_test_results.md`: 상세한 테스트 결과 문서

## 주요 성과 (2025년 10월 2일)

### ✅ 완료된 기능
1. **Isaac Sim 5.0 초기화** - SimulationApp 정상 작동
2. **USD Stage 생성** - 타입 안전성 확보
3. **USD 조명 시스템** - DistantLight 정상 추가
4. **물리 엔진 연동** - PhysX GPU 가속 활성화
5. **시뮬레이션 실행** - 10단계 스텝 테스트 성공
6. **USD 파일 저장** - 씬 데이터 영구 보존
7. **🤖 UR10 로봇 모델 로딩** - Universal Robots UR10 성공적 로드
8. **🤖 Franka 로봇 모델 로딩** - Franka Panda 로봇 성공적 로드
9. **로봇 위치 설정** - Transform 안전 설정 (타입 호환성 확보)
10. **멀티 로봇 환경** - 동시 로딩 및 배치 성공

### 🔧 해결한 기술적 문제들
1. **타입 미스매치**: `ctx.new_stage()` bool 반환값 처리
2. **USD API 호환성**: `UsdLux.DistantLight` + `Sdf.Path` 명시적 사용
3. **물리 시뮬레이션**: `World.step()` 메소드 활용
4. **World 생성자**: Isaac Sim 5.0 파라미터 변경사항 적용

## 실행 방법

### 기본 테스트
```bash
cd ~/isaac_sim
./python.sh /home/roarm_m3/dev_roarm/roarm_mcp/tests/test_isaac_sim_basic.py
```

### Isaac Sim 5.0 기능 테스트
```bash
cd ~/isaac_sim
./python.sh /home/roarm_m3/dev_roarm/roarm_mcp/tests/test_isaac_sim_v5.py
```

### MCP 연동 테스트 (최종)
```bash
cd ~/isaac_sim
./python.sh /home/roarm_m3/dev_roarm/roarm_mcp/tests/test_isaac_sim_mcp_integration.py
```

### 로봇 모델 로딩 테스트 🤖
```bash
cd ~/isaac_sim
./python.sh /home/roarm_m3/dev_roarm/roarm_mcp/tests/test_robot_models_final.py
```

## 테스트 결과

### ✅ Isaac Sim 5.0 MCP 연동 완료
- 모든 deprecated API 호환성 문제 해결
- USD 파이프라인 완전 구축
- PhysX GPU 가속 물리 시뮬레이션 동작

### ✅ 로봇 모델 로딩 성공  
- **UR10**: `/Isaac/Robots/UniversalRobots/ur10/ur10.usd` 로딩 성공
- **Franka**: `/Isaac/Robots/Franka/franka.usd` 로딩 성공
- **멀티 로봇**: 동일 환경에서 두 로봇 동시 배치
- **USD Transform**: 타입 안전성 확보한 위치 설정

## 생성된 파일들
- `isaac_test_scene.usd`: 테스트 과정에서 생성된 USD 씬 파일

## 다음 단계
1. **로봇 모델 로딩**: UR10, Franka 등 로봇 모델 Isaac Sim 연동
2. **MCP 실시간 통신**: 서버-클라이언트 간 실시간 데이터 교환
3. **제어 인터페이스**: 로봇 조작 및 센서 데이터 수집

## 시스템 요구사항
- Ubuntu 24.04.3 LTS
- Isaac Sim 5.0
- NVIDIA GPU (RTX 5090 권장)
- Python 3.11+