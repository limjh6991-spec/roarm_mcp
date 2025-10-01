# 로봇팔 시뮬레이션 연동 방안

## 1. 개요

이 문서는 NVIDIA Isaac Sim을 사용하여 로봇팔 시뮬레이션을 구현하고 강화학습 환경과 연동하는 방법에 대한 계획을 설명합니다. Isaac Sim의 물리 기반 시뮬레이션과 로봇 모델을 활용하여 실제와 가까운 로봇팔 제어 환경을 구축하는 방안을 제시합니다.

## 2. 로봇팔 모델 선택

### 2.1 지원 로봇팔 모델

Isaac Sim에서 기본적으로 제공하거나 참조 구현에서 확인된 로봇팔 모델은 다음과 같습니다:

1. **Franka Emika Panda**
   - 7 자유도 협동 로봇팔
   - USD 파일 경로: `Isaac/Robots/Franka/franka_alt_fingers.usd`

2. **Universal Robots UR10**
   - 6 자유도 산업용 로봇팔
   - USD 파일 경로: `Isaac/Robots/UniversalRobots/ur10.usd`

3. **Custom Robot Arm**
   - 사용자 정의 로봇팔 모델 (별도 USD 파일 제작 필요)

### 2.2 모델 로딩 방법

Isaac Sim에서 로봇팔 모델을 로드하는 기본 방법:

```python
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.core.prims import XFormPrim
import numpy as np

# 에셋 경로 가져오기
assets_root_path = get_assets_root_path()

# Franka 로봇팔 로드
asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
franka_robot = add_reference_to_stage(asset_path, "/World/Franka")

# 로봇 위치 설정
robot_prim = XFormPrim(prim_path="/World/Franka")
robot_prim.set_world_pose(position=np.array([0.0, 0.0, 0.0]))
```

## 3. 로봇팔 제어 방법

### 3.1 관절 위치 제어

Isaac Sim에서 Articulation을 통해 로봇팔의 관절을 제어하는 방법:

```python
from omni.isaac.core.articulations import Articulation
import numpy as np

# 로봇팔 초기화
franka_robot = Articulation(prim_path="/World/Franka", name="Franka")
franka_robot.initialize(simulation_context.physics_sim_view)

# 컨트롤러 가져오기
controller = franka_robot.get_articulation_controller()

# 관절 위치 설정 (예: 7 자유도 Franka)
positions = np.array([0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0])
controller.apply_action(positions, ActionType.POSITION)
```

### 3.2 카르테시안 위치 제어

역기구학(IK)을 사용하여 로봇팔의 카르테시안 위치 제어:

```python
from omni.isaac.robot_tools.IK import IK

# IK 솔버 생성
ik_solver = IK(robot_path="/World/Franka", end_effector_path="/World/Franka/panda_link8")

# 목표 카르테시안 위치 및 회전 설정
target_position = np.array([0.5, 0.0, 0.5])  # x, y, z (미터)
target_orientation = np.array([1.0, 0.0, 0.0, 0.0])  # 쿼터니언 (w, x, y, z)

# IK 해 계산
joint_positions = ik_solver.solve(target_position, target_orientation)

# 계산된 관절 각도 적용
controller.apply_action(joint_positions, ActionType.POSITION)
```

### 3.3 그리퍼 제어

그리퍼를 포함한 로봇팔 모델의 경우, 그리퍼 제어 방법:

```python
# 그리퍼 관절 인덱스 (로봇 모델에 따라 다름)
gripper_indices = [9, 10]  # Franka의 경우 예시

# 그리퍼 열기
open_positions = np.array([0.04, 0.04])  # 개방 위치 (미터)
controller.apply_action(open_positions, ActionType.POSITION, gripper_indices)

# 그리퍼 닫기
close_positions = np.array([0.0, 0.0])  # 닫힘 위치 (미터)
controller.apply_action(close_positions, ActionType.POSITION, gripper_indices)
```

## 4. 센서 데이터 수집

### 4.1 관절 상태 데이터

로봇팔의 관절 상태 정보 수집:

```python
# 관절 위치 읽기
joint_positions = franka_robot.get_joint_positions()

# 관절 속도 읽기
joint_velocities = franka_robot.get_joint_velocities()

# 관절 토크/힘 읽기
joint_efforts = franka_robot.get_joint_efforts()
```

### 4.2 카메라 데이터

로봇팔 작업 영역의 시각 정보 수집:

```python
from omni.isaac.sensor import Camera
import numpy as np

# 카메라 생성
camera = Camera(
    prim_path="/World/Camera",
    position=np.array([1.0, 0.0, 0.5]),
    orientation=np.array([0.0, 0.0, 0.0, 1.0]),
    width=640,
    height=480
)

# 카메라 초기화
camera.initialize()

# 이미지 데이터 수집
rgb_data = camera.get_rgba()
depth_data = camera.get_depth()
```

### 4.3 충돌 감지

로봇팔과 환경 사이의 충돌 감지:

```python
from omni.isaac.core.utils.physics import PhysicsUtils

# 충돌 감지
contacts = PhysicsUtils.get_all_contacts(simulation_context.physics_sim_view)

# 로봇팔 관련 충돌 필터링
robot_contacts = [c for c in contacts if "/World/Franka" in c.body0_name or "/World/Franka" in c.body1_name]
```

## 5. 강화학습 환경 통합

### 5.1 관찰 공간 설계

로봇팔 강화학습을 위한 관찰 공간 구성:

```python
import gym
import numpy as np

# 관찰 공간 정의
observation_space = gym.spaces.Dict({
    # 관절 상태: 위치, 속도
    "joint_positions": gym.spaces.Box(low=-np.pi, high=np.pi, shape=(7,)),
    "joint_velocities": gym.spaces.Box(low=-10.0, high=10.0, shape=(7,)),
    
    # 엔드 이펙터 상태: 위치, 회전
    "ee_position": gym.spaces.Box(low=-2.0, high=2.0, shape=(3,)),
    "ee_orientation": gym.spaces.Box(low=-1.0, high=1.0, shape=(4,)),
    
    # 작업 관련 상태: 목표 위치까지 거리
    "target_distance": gym.spaces.Box(low=0.0, high=np.inf, shape=(1,)),
    
    # 시각 정보 (옵션)
    "camera": gym.spaces.Box(low=0, high=255, shape=(480, 640, 3), dtype=np.uint8)
})
```

### 5.2 액션 공간 설계

로봇팔 제어를 위한 액션 공간:

```python
# 관절 위치 제어
action_space_joint = gym.spaces.Box(low=-1.0, high=1.0, shape=(7,))

# 카르테시안 위치 제어
action_space_cartesian = gym.spaces.Box(low=-1.0, high=1.0, shape=(6,))  # 위치(3) + 회전(3)
```

### 5.3 보상 함수 설계

로봇팔 제어 작업에 적합한 보상 함수 예시:

```python
def compute_reward(self, achieved_goal, desired_goal, info):
    # 목표 위치와 현재 위치 사이의 거리
    distance = np.linalg.norm(achieved_goal - desired_goal)
    
    # 거리 기반 음의 보상
    distance_reward = -distance
    
    # 부드러운 움직임 보상
    smoothness_reward = -np.sum(np.square(self.joint_velocities))
    
    # 에너지 효율성 보상
    energy_reward = -np.sum(np.abs(self.joint_efforts))
    
    # 작업 완료 보상
    task_reward = 10.0 if distance < 0.05 else 0.0
    
    # 총 보상 계산
    total_reward = distance_reward + 0.01 * smoothness_reward + 0.005 * energy_reward + task_reward
    
    return total_reward
```

## 6. 구현 계획

### 6.1 로봇팔 모델 로드 및 초기화

1. Isaac Sim에서 로봇팔 USD 모델 로드
2. 관절 제어를 위한 Articulation 초기화
3. 그리퍼 제어 설정 (필요시)
4. 카메라 및 센서 설정

### 6.2 로봇팔 제어 클래스 구현

1. 관절 위치 제어 메서드
2. 카르테시안 위치 제어 메서드 (IK 사용)
3. 그리퍼 제어 메서드
4. 센서 데이터 수집 메서드

### 6.3 강화학습 환경 구현

1. Gym 환경 클래스 정의
2. 관찰/액션 공간 설정
3. 보상 함수 구현
4. 리셋 및 스텝 메서드 구현

### 6.4 MCP 서버 통합

1. MCP 명령 핸들러 구현
2. 로봇팔 제어 명령 처리
3. 센서 데이터 제공 API
4. 강화학습 환경 제어 API

## 7. 테스트 계획

### 7.1 기본 제어 테스트

1. 관절 위치 제어 테스트
2. 카르테시안 위치 제어 테스트
3. 그리퍼 제어 테스트

### 7.2 작업 시뮬레이션 테스트

1. 물체 집기(Pick) 작업 테스트
2. 물체 놓기(Place) 작업 테스트
3. 경로 추종 테스트

### 7.3 강화학습 훈련 테스트

1. 단순 관절 제어 학습 테스트
2. 목표 위치 도달 학습 테스트
3. 물체 조작 학습 테스트

## 8. 결론

Isaac Sim을 활용한 로봇팔 시뮬레이션은 다음과 같은 이점을 제공합니다:

1. 사실적인 물리 기반 시뮬레이션
2. 다양한 로봇팔 모델 지원
3. 센서 시뮬레이션 및 데이터 수집
4. 강화학습 환경으로의 쉬운 통합

이러한 이점을 활용하여 실제 로봇으로의 전이(Sim-to-Real Transfer)가 용이한 강화학습 환경을 구축할 수 있습니다.