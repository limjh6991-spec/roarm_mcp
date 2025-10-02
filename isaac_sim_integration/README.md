# Isaac Sim Integration

이 디렉토리는 **RoArm MCP**와 **Isaac Sim 5.0** 통합 솔루션을 포함합니다.

## 📁 구조

```
isaac_sim_integration/
├── solutions/          # 완성된 솔루션 코드
├── outputs/           # 생성된 USD 파일들
└── robot_loading_test_results.md  # 테스트 결과 문서
```

## 🎯 핵심 솔루션

### `solutions/isaac_sim_physx_tensors_solution.py`

Isaac Sim 5.0 PhysX Tensors API를 사용한 **완전한 로봇 제어 솔루션**입니다.

#### 주요 특징:
- ✅ **중첩된 ArticulationRoot 정리**: 한 서브트리에 Root 1개 원칙
- ✅ **올바른 World.reset() 순서**: PhysX 상태 초기화 후 뷰 생성
- ✅ **PhysX Tensors SimulationView**: `omni.physics.tensors.create_simulation_view()`
- ✅ **Torch Tensor Indices**: Isaac Sim 5.0 API 호환성
- ✅ **클라우드 Asset 지원**: HTTPS 기반 USD 로딩

#### 해결된 문제점:
1. **"Nested articulation roots are not allowed"** → ArticulationRoot 정리
2. **"'NoneType' object has no attribute 'to'"** → 올바른 torch tensor indices
3. **"Asset loading failures"** → 클라우드 호스팅 에셋 경로 수정

## 🚀 실행 방법

```bash
cd /home/roarm_m3/isaac_sim
./python.sh /home/roarm_m3/dev_roarm/roarm_mcp/isaac_sim_integration/solutions/isaac_sim_physx_tensors_solution.py
```

## 📊 테스트 결과

- **로드된 로봇**: UR10 (6 DOF), Franka (9 DOF)
- **ArticulationRoot 정리**: 성공
- **SimulationView 생성**: 성공
- **ArticulationView 생성**: 성공
- **제어 API 호환성**: Isaac Sim 5.0 완전 지원

## 📝 다음 단계

1. **RoArm MCP 서버 통합**: MCP 프로토콜과 Isaac Sim 연동
2. **로봇 센서 데이터**: 카메라, LiDAR 등 센서 통합
3. **고급 제어**: 궤적 추적, 역기구학 등

---

**작성일**: 2025년 10월 2일  
**Isaac Sim 버전**: 5.0.0  
**상태**: 완료 ✅