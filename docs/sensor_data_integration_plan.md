# 🎥 Isaac Sim 5.0 센서 데이터 통합 계획서
> **RoArm MCP 프로젝트 3단계: 센서 데이터 통합**  
> 작성일: 2025-10-02  
> 현재 단계: MCP 서버 구축 완료 → 센서 데이터 통합 시작

---

## 📋 **현재 상황 요약**

### ✅ **완료된 작업**
1. **Isaac Sim 5.0 PhysX Tensors API 통합** ✅
   - 완성된 솔루션: `isaac_sim_integration/solutions/isaac_sim_physx_tensors_solution.py`
   - UR10, Franka 로봇 모델 로딩 및 제어 성공

2. **MCP 서버-클라이언트 구조 구현** ✅
   - Isaac Sim MCP 핸들러: `mcp/isaac_sim_handler.py`
   - 서버 스크립트: `examples/run_isaac_sim_server.py`
   - 클라이언트: `examples/isaac_sim_client.py`
   - 안정성 테스트 통과

3. **WebSocket 기반 실시간 통신** ✅
   - 포트 8765에서 안정적인 MCP 서버 운영
   - 다양한 에러 시나리오 처리 확인
   - 장시간 연결 안정성 검증

### 🎯 **다음 목표: 센서 데이터 통합**

---

## 🔧 **센서 데이터 통합 아키텍처**

### **1. Isaac Sim 5.0 센서 API 활용**
```
Isaac Sim 5.0 센서 시스템:
├── 카메라 센서 (isaacsim.sensors.camera)
│   ├── RGB 카메라
│   ├── 깊이 카메라 (Depth)
│   ├── 세그멘테이션 카메라
│   └── 스테레오 카메라
├── LiDAR 센서 (isaacsim.sensors.rtx)
│   ├── RTX LiDAR (실시간 레이트레이싱)
│   ├── 회전식 LiDAR (PhysX)
│   └── 솔리드스테이트 LiDAR
└── 물리 센서 (isaacsim.sensors.physics)
    ├── IMU 센서
    ├── 접촉 센서
    └── 힘/토크 센서
```

### **2. MCP 센서 데이터 플로우**
```
센서 데이터 → Isaac Sim Sensors API → MCP Handler → WebSocket → 클라이언트
     ↓              ↓                    ↓             ↓           ↓
 RGB/Depth     센서 뷰 생성        데이터 직렬화    JSON 전송    시각화/분석
 Point Cloud   -> get_data()      -> numpy/base64  -> 실시간    -> 제어 피드백
 IMU Data      -> 동기화          -> 압축 최적화   -> 안정성    -> 의사결정
```

---

## 📅 **단계별 구현 계획**

### **Phase 3.1: 카메라 센서 통합 (1주차)**

#### **3.1.1 카메라 센서 구현**
- [ ] **RGB 카메라 통합**
  - Isaac Sim Camera API 활용
  - 실시간 이미지 캡처 및 전송
  - 해상도 및 프레임레이트 최적화

- [ ] **Depth 카메라 구현**
  - 깊이 정보 수집 및 처리
  - Point Cloud 생성
  - 거리 측정 및 장애물 감지

#### **3.1.2 데이터 전송 최적화**
- [ ] **이미지 압축 및 직렬화**
  - JPEG/PNG 압축 적용
  - Base64 인코딩으로 WebSocket 전송
  - 프레임 드롭 방지 메커니즘

- [ ] **실시간 스트리밍**
  - 비동기 데이터 전송
  - 버퍼링 및 큐 관리
  - 클라이언트별 품질 조절

### **Phase 3.2: LiDAR 센서 통합 (2주차)**

#### **3.2.1 RTX LiDAR 구현**
- [ ] **실시간 레이트레이싱 LiDAR**
  - Isaac Sim RTX LiDAR 설정
  - Point Cloud 데이터 수집
  - 3D 환경 매핑

- [ ] **Point Cloud 처리**
  - 3D 포인트 데이터 필터링
  - 거리 및 각도 정보 추출
  - 실시간 처리 성능 최적화

#### **3.2.2 다중 센서 융합**
- [ ] **카메라-LiDAR 융합**
  - 센서 데이터 시간 동기화
  - 좌표계 변환 및 칼리브레이션
  - 융합 데이터 품질 향상

### **Phase 3.3: 고급 센서 기능 (3주차)**

#### **3.3.1 IMU 및 물리 센서**
- [ ] **IMU 센서 통합**
  - 가속도계, 자이로스코프 데이터
  - 로봇 자세 및 움직임 추적
  - 칼만 필터링 적용

- [ ] **접촉 및 힘 센서**
  - 로봇-환경 상호작용 감지
  - 힘/토크 피드백 수집
  - 안전 제어 메커니즘

#### **3.3.2 센서 데이터 분석**
- [ ] **실시간 데이터 처리**
  - 노이즈 필터링 및 스무딩
  - 이상치 탐지 및 제거
  - 통계적 분석 및 트렌드 추적

---

## 🛠️ **기술적 구현 세부사항**

### **1. Isaac Sim 센서 API 활용**
```python
# 카메라 센서 설정 예시
from isaacsim.sensors.camera import Camera
from omni.isaac.core.utils.prims import create_prim

# RGB 카메라 생성
camera = Camera(
    prim_path="/World/Camera",
    position=np.array([2.0, 2.0, 2.0]),
    target=np.array([0.0, 0.0, 0.0]),
    resolution=(1280, 720),
    frequency=30  # 30 FPS
)

# 이미지 데이터 수집
rgb_data = camera.get_rgba()[:, :, :3]  # RGB only
```

### **2. MCP 센서 데이터 핸들러**
```python
class SensorDataHandler:
    def __init__(self):
        self.cameras = {}
        self.lidars = {}
        self.imus = {}
    
    async def collect_sensor_data(self):
        """모든 센서 데이터 수집"""
        data = {
            "timestamp": time.time(),
            "cameras": await self.collect_camera_data(),
            "lidars": await self.collect_lidar_data(),
            "imus": await self.collect_imu_data()
        }
        return data
    
    async def stream_to_clients(self, data):
        """실시간 데이터 스트리밍"""
        compressed_data = self.compress_data(data)
        await self.broadcast_to_clients(compressed_data)
```

### **3. 데이터 압축 및 전송**
```python
import cv2
import base64
import json

def compress_image(image_array):
    """이미지 압축 및 인코딩"""
    # JPEG 압축
    _, buffer = cv2.imencode('.jpg', image_array, 
                           [cv2.IMWRITE_JPEG_QUALITY, 85])
    # Base64 인코딩
    encoded_image = base64.b64encode(buffer).decode('utf-8')
    return encoded_image

def compress_point_cloud(points):
    """Point Cloud 압축"""
    # 필요한 포인트만 필터링
    filtered_points = filter_by_distance(points, max_distance=10.0)
    # 압축 알고리즘 적용
    compressed = compress_array(filtered_points)
    return compressed
```

---

## 📊 **성능 최적화 전략**

### **1. 데이터 전송 최적화**
- **이미지 품질 조절**: 클라이언트별 해상도 및 압축 설정
- **프레임 레이트 조절**: 네트워크 상황에 따른 동적 FPS 조절
- **선택적 전송**: 관심 영역(ROI) 기반 데이터 전송
- **압축 알고리즘**: 실시간성을 고려한 압축 방식 선택

### **2. 메모리 및 CPU 최적화**
- **버퍼 풀링**: 메모리 재사용을 통한 GC 부담 감소
- **멀티스레딩**: 센서 데이터 수집과 전송 분리
- **배치 처리**: 여러 센서 데이터 동시 처리
- **캐싱 전략**: 자주 사용되는 데이터 캐싱

### **3. 네트워크 최적화**
- **압축 전송**: gzip, zlib 등 실시간 압축
- **델타 압축**: 이전 프레임과의 차이만 전송
- **QoS 제어**: 중요도에 따른 우선순위 전송
- **연결 풀링**: 재사용 가능한 연결 관리

---

## 🧪 **테스트 및 검증 계획**

### **1. 단위 테스트**
- [ ] **센서별 데이터 수집 테스트**
- [ ] **데이터 압축/해제 정확성 테스트**
- [ ] **WebSocket 전송 안정성 테스트**
- [ ] **실시간 성능 벤치마크**

### **2. 통합 테스트**
- [ ] **다중 센서 동시 작동 테스트**
- [ ] **센서 데이터 시간 동기화 테스트**
- [ ] **대용량 데이터 전송 스트레스 테스트**
- [ ] **네트워크 지연 및 패킷 손실 테스트**

### **3. 사용자 시나리오 테스트**
- [ ] **로봇 원격 조작 시나리오**
- [ ] **실시간 환경 모니터링**
- [ ] **장애물 회피 및 경로 계획**
- [ ] **다중 클라이언트 동시 접속**

---

## 📈 **예상 결과물**

### **구현 결과물**
1. **센서 통합 MCP 핸들러**: `mcp/sensor_handler.py`
2. **카메라 센서 모듈**: `sensors/camera_sensor.py`
3. **LiDAR 센서 모듈**: `sensors/lidar_sensor.py`
4. **센서 데이터 클라이언트**: `examples/sensor_data_client.py`
5. **실시간 모니터링 대시보드**: `dashboard/sensor_monitor.html`

### **성능 목표**
- **카메라 스트리밍**: 30 FPS @ 1280x720 해상도
- **LiDAR 데이터**: 10 Hz 업데이트, 100K+ 포인트 처리
- **전송 지연**: < 100ms 엔드-투-엔드 지연
- **동시 연결**: 10개 클라이언트 안정적 지원
- **메모리 사용**: < 2GB 상주 메모리

---

## 🚀 **실행 계획 요약**

### **우선순위 1: 카메라 센서 (1주)**
1. RGB 카메라 기본 구현
2. 이미지 압축 및 WebSocket 전송
3. 실시간 스트리밍 최적화
4. 기본 클라이언트 뷰어 구현

### **우선순위 2: LiDAR 센서 (1주)**
1. RTX LiDAR 통합
2. Point Cloud 데이터 처리
3. 3D 시각화 클라이언트
4. 센서 융합 기초 구현

### **우선순위 3: 고급 기능 (1주)**
1. IMU 센서 추가
2. 다중 센서 동기화
3. 실시간 분석 대시보드
4. 종합 테스트 및 최적화

---

> **🎯 목표**: Isaac Sim 5.0의 강력한 센서 기능을 MCP 프로토콜로 실시간 스트리밍하여, 원격에서도 로봇의 "눈과 귀"를 완전히 활용할 수 있는 시스템 구축

**다음 단계**: 카메라 센서 통합부터 시작하여 단계적으로 센서 데이터 통합 완성!