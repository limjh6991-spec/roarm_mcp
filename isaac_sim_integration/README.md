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

## 🎯 Phase 3.1 RGB-D 카메라 센서 시스템 (완료)

### 개발 완료 항목 ✅
1. **Enhanced RGB Camera Sensor**: 49.0 FPS, 해상도/조리개 정합
2. **Enhanced Depth Camera Sensor**: 16-bit 고정밀도, Isaac Sim 5.0 Depth Annotator
3. **Integrated RGB-D System**: 병렬/순차 동기화, ThreadPoolExecutor
4. **압축 및 인코딩**: JPEG/PNG 다중 포맷, 14.3x 압축비
5. **성능 벤치마크**: 17/17 단위 테스트 통과, 166% 달성도

### 센서 파일 위치
```
roarm_mcp/sensors/
├── enhanced_rgb_camera_sensor.py      # RGB 센서 (49.0 FPS)
├── enhanced_depth_camera_sensor.py    # Depth 센서 (16-bit)
├── integrated_rgbd_camera_sensor.py   # 통합 RGB-D 시스템
└── rgb_camera_sensor.py              # 기본 RGB 센서
```

### 출력 디렉토리
- **RGB 출력**: `/tmp/enhanced_rgb_test/`
- **Depth 출력**: `/tmp/enhanced_depth_test/`  
- **통합 출력**: `/tmp/integrated_rgbd_test/`
- **성능 보고서**: `/tmp/rgbd_performance_report.json`

## 📝 다음 단계 (Phase 3.2)

1. **ROS2 노드 개발**: camera_info.json 활용한 ROS2 통합
2. **실시간 스트리밍**: WebRTC/gRPC 기반 원격 전송
3. **AI 파이프라인 연계**: RGB-D 데이터 기반 객체 인식/추적
4. **성능 최적화**: GPU 가속, 캐시 파이프라인 적용

---

**작성일**: 2025년 10월 3일  
**Isaac Sim 버전**: 5.0.0  
**Phase 3.1 상태**: 완료 ✅ (166% 달성)  
**Phase 3.2 상태**: 준비 완료 🚀