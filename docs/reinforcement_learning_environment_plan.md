# Isaac Sim 기반 강화학습 환경 설정 계획

## 1. 개요

이 문서는 NVIDIA Isaac Sim을 기반으로 한 로봇팔 강화학습 환경 구축 방법과 기존 코드와의 통합 계획을 설명합니다. Isaac Sim의 물리 기반 시뮬레이션과 실제 같은 렌더링을 활용하여 실제 로봇으로의 전이가 용이한 강화학습 환경을 구현하는 방안을 제시합니다.

## 2. 기존 강화학습 환경 분석

### 2.1 현재 구현된 Gym 환경

현재 `roarm_mcp` 프로젝트에서는 Gymnasium 기반의 강화학습 환경이 구현되어 있습니다. 주요 특징은 다음과 같습니다:

- PyBullet 기반 물리 시뮬레이션
- 단순화된 로봇팔 모델 사용
- 관절 위치 제어 중심의 액션 공간
- 픽셀 및 상태 기반의 관찰 공간
- 목표 위치 도달 보상 함수

### 2.2 필요한 변경 사항

Isaac Sim 기반 환경으로 전환하기 위해 필요한 주요 변경 사항:

1. PyBullet에서 Isaac Sim으로 시뮬레이션 엔진 변경
2. 로봇팔 모델을 Isaac Sim USD 형식으로 변경
3. Isaac Sim API를 통한 관절 제어 및 상태 조회 메서드 재구현
4. 관찰 공간 및 보상 함수 조정

## 3. Isaac Sim 기반 강화학습 환경 설계

### 3.1 환경 클래스 구조

```python
import gym
import numpy as np
from omni.isaac.core import SimulationContext
from omni.isaac.core.articulations import Articulation
from omni.isaac.robot_tools.IK import IK

class IsaacRoArmEnv(gym.Env):
    """Isaac Sim 기반 로봇팔 강화학습 환경"""
    
    def __init__(self, robot_type="franka", task_type="reach", obs_type="state"):
        """환경 초기화
        
        Args:
            robot_type: 로봇 모델 유형 (franka, ur10 등)
            task_type: 작업 유형 (reach, pick, place 등)
            obs_type: 관찰 유형 (state, pixel, state_pixel)
        """
        # 시뮬레이션 컨텍스트
        self.sim_context = SimulationContext()
        
        # 작업 유형 및 관찰 유형
        self.task_type = task_type
        self.obs_type = obs_type
        
        # 로봇팔 로드 및 초기화
        self._load_robot(robot_type)
        
        # 액션 및 관찰 공간 정의
        self._define_spaces()
        
        # 목표 위치 초기화
        self.target_position = np.zeros(3)
    
    def reset(self):
        """환경 리셋"""
        # 시뮬레이션 리셋
        self.sim_context.reset()
        
        # 로봇 관절 초기 위치 설정
        self._reset_robot_pose()
        
        # 새 목표 위치 설정
        self._set_new_target()
        
        # 초기 관찰값 반환
        return self._get_observation()
    
    def step(self, action):
        """환경 스텝 실행"""
        # 액션 적용 (관절 제어)
        self._apply_action(action)
        
        # 시뮬레이션 단계 진행
        for _ in range(self.sim_steps_per_env_step):
            self.sim_context.step()
        
        # 관찰, 보상, 완료 여부, 정보 반환
        observation = self._get_observation()
        reward = self._compute_reward()
        done = self._is_done()
        info = self._get_info()
        
        return observation, reward, done, info
    
    def render(self, mode="human"):
        """환경 렌더링 (Isaac Sim은 자동 렌더링됨)"""
        pass
    
    def close(self):
        """환경 종료"""
        self.sim_context.stop()
    
    def _load_robot(self, robot_type):
        """로봇 모델 로드 및 초기화"""
        # 구현
    
    def _define_spaces(self):
        """액션 및 관찰 공간 정의"""
        # 구현
    
    def _reset_robot_pose(self):
        """로봇 초기 자세 설정"""
        # 구현
    
    def _set_new_target(self):
        """새 목표 위치 설정"""
        # 구현
    
    def _get_observation(self):
        """현재 관찰값 수집"""
        # 구현
    
    def _apply_action(self, action):
        """로봇에 액션 적용"""
        # 구현
    
    def _compute_reward(self):
        """보상 계산"""
        # 구현
    
    def _is_done(self):
        """종료 조건 확인"""
        # 구현
    
    def _get_info(self):
        """추가 정보 수집"""
        # 구현
```

### 3.2 관찰 공간 설계

Isaac Sim 기반 환경에서 사용할 관찰 공간 설계:

```python
def _define_spaces(self):
    """액션 및 관찰 공간 정의"""
    # 액션 공간 (7 자유도 Franka 로봇의 경우)
    self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(7,), dtype=np.float32)
    
    # 관찰 공간 (상태 기반)
    if self.obs_type == "state" or self.obs_type == "state_pixel":
        # 상태 기반 관찰 공간 정의
        self.state_space = gym.spaces.Dict({
            # 관절 상태: 위치, 속도
            "joint_positions": gym.spaces.Box(low=-np.pi, high=np.pi, shape=(7,), dtype=np.float32),
            "joint_velocities": gym.spaces.Box(low=-10.0, high=10.0, shape=(7,), dtype=np.float32),
            
            # 엔드 이펙터 상태: 위치, 회전
            "ee_position": gym.spaces.Box(low=-2.0, high=2.0, shape=(3,), dtype=np.float32),
            "ee_orientation": gym.spaces.Box(low=-1.0, high=1.0, shape=(4,), dtype=np.float32),
            
            # 목표 관련: 목표 위치, 목표까지 거리
            "target_position": gym.spaces.Box(low=-2.0, high=2.0, shape=(3,), dtype=np.float32),
            "target_distance": gym.spaces.Box(low=0.0, high=np.inf, shape=(1,), dtype=np.float32),
            
            # 그리퍼 상태 (그리퍼가 있는 경우)
            "gripper_width": gym.spaces.Box(low=0.0, high=0.08, shape=(1,), dtype=np.float32)
        })
    
    # 픽셀 기반 관찰 공간 (카메라 이미지)
    if self.obs_type == "pixel" or self.obs_type == "state_pixel":
        # 카메라 이미지 (RGB)
        self.pixel_space = gym.spaces.Box(
            low=0, high=255, shape=(480, 640, 3), dtype=np.uint8
        )
    
    # 최종 관찰 공간
    if self.obs_type == "state":
        self.observation_space = self.state_space
    elif self.obs_type == "pixel":
        self.observation_space = self.pixel_space
    elif self.obs_type == "state_pixel":
        # 상태와 픽셀을 결합한 관찰 공간
        self.observation_space = gym.spaces.Dict({
            "state": self.state_space,
            "pixels": self.pixel_space
        })
```

### 3.3 액션 적용 메서드

Isaac Sim 환경에서 관절 제어 액션을 적용하는 방법:

```python
def _apply_action(self, action):
    """로봇에 액션 적용"""
    # 액션 범위 조정 ([-1, 1] → 실제 관절 제한)
    scaled_action = self._scale_action(action)
    
    # 컨트롤러 가져오기
    controller = self.robot.get_articulation_controller()
    
    # 관절 위치 명령 적용
    controller.apply_action(scaled_action)

def _scale_action(self, action):
    """액션 스케일링 ([-1, 1] → 실제 관절 제한)"""
    # 각 관절의 최소/최대 제한
    joint_limits = self.robot.get_joint_limits()
    joint_mins, joint_maxs = joint_limits[:, 0], joint_limits[:, 1]
    
    # 스케일링 ([-1, 1] → [min, max])
    scaled_action = 0.5 * (action + 1.0) * (joint_maxs - joint_mins) + joint_mins
    
    return scaled_action
```

### 3.4 보상 함수 설계

로봇팔 제어 작업에 적합한 보상 함수:

```python
def _compute_reward(self):
    """보상 계산"""
    # 엔드 이펙터 위치 및 목표 위치
    ee_position = self._get_ee_position()
    target_position = self.target_position
    
    # 목표까지 거리 계산
    distance = np.linalg.norm(ee_position - target_position)
    
    # 이전 스텝과 현재 스텝의 거리 변화
    distance_diff = self.prev_distance - distance
    self.prev_distance = distance
    
    # 거리 기반 음의 보상
    distance_reward = -distance
    
    # 거리 감소 보상 (진행 방향 보상)
    progress_reward = 10.0 * distance_diff
    
    # 부드러운 움직임 보상
    joint_velocities = self.robot.get_joint_velocities()
    smoothness_reward = -0.01 * np.sum(np.square(joint_velocities))
    
    # 에너지 효율성 보상
    joint_efforts = self.robot.get_joint_efforts()
    energy_reward = -0.005 * np.sum(np.abs(joint_efforts))
    
    # 작업 완료 보상
    task_reward = 10.0 if distance < 0.05 else 0.0
    
    # 총 보상 계산
    total_reward = distance_reward + progress_reward + smoothness_reward + energy_reward + task_reward
    
    return total_reward
```

## 4. 기존 코드와의 통합 계획

### 4.1 통합 접근 방식

기존 Gymnasium 기반 환경 코드와 Isaac Sim 환경을 통합하는 방법:

1. **인터페이스 호환성 유지**:
   - 동일한 Gym 인터페이스 유지 (reset, step, render 등)
   - 유사한 관찰/액션 공간 구조 유지

2. **환경 선택 메커니즘**:
   - 환경 생성 시 시뮬레이션 엔진을 선택할 수 있는 팩토리 패턴 적용
   - `sim_type` 매개변수로 "bullet" 또는 "isaac" 선택 가능

3. **점진적 마이그레이션**:
   - 기존 PyBullet 환경 유지하면서 Isaac Sim 환경 추가
   - 동일한 작업에 대해 두 환경에서 비교 테스트 가능

### 4.2 환경 팩토리 구현

환경 생성을 위한 팩토리 패턴 구현:

```python
def create_roarm_env(
    sim_type="bullet",
    robot_type="franka",
    task_type="reach",
    obs_type="state",
    render=False,
    **kwargs
):
    """로봇팔 강화학습 환경 생성 팩토리 함수
    
    Args:
        sim_type: 시뮬레이션 유형 ("bullet" 또는 "isaac")
        robot_type: 로봇 유형 ("franka", "ur10" 등)
        task_type: 작업 유형 ("reach", "pick", "place" 등)
        obs_type: 관찰 유형 ("state", "pixel", "state_pixel")
        render: 렌더링 활성화 여부
        **kwargs: 추가 매개변수
    
    Returns:
        gym.Env: 생성된 환경 인스턴스
    """
    if sim_type == "bullet":
        # 기존 PyBullet 기반 환경
        from robot_arm.simulation.gym_env import RoArmEnv
        return RoArmEnv(
            robot_type=robot_type,
            task_type=task_type,
            obs_type=obs_type,
            render=render,
            **kwargs
        )
    
    elif sim_type == "isaac":
        # Isaac Sim 기반 환경
        from robot_arm.simulation.isaac_env import IsaacRoArmEnv
        return IsaacRoArmEnv(
            robot_type=robot_type,
            task_type=task_type,
            obs_type=obs_type,
            **kwargs
        )
    
    else:
        raise ValueError(f"Unknown simulation type: {sim_type}")
```

### 4.3 강화학습 알고리즘 통합

기존 강화학습 알고리즘 코드와 새로운 환경 통합:

```python
import gymnasium as gym
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import CheckpointCallback

# 환경 생성 (Isaac Sim 또는 PyBullet)
env = create_roarm_env(
    sim_type="isaac",  # "bullet" 또는 "isaac"
    robot_type="franka",
    task_type="reach",
    obs_type="state"
)

# Stable Baselines3 에이전트 생성
model = SAC(
    "MultiInputPolicy" if isinstance(env.observation_space, gym.spaces.Dict) else "MlpPolicy",
    env,
    verbose=1,
    tensorboard_log="./logs/"
)

# 체크포인트 저장 콜백
checkpoint_callback = CheckpointCallback(
    save_freq=10000,
    save_path="./models/",
    name_prefix="roarm_isaac"
)

# 모델 학습
model.learn(
    total_timesteps=1000000,
    callback=checkpoint_callback
)

# 모델 저장
model.save("roarm_isaac_sac")
```

## 5. 작업 유형 및 확장

### 5.1 지원할 작업 유형

Isaac Sim 환경에서 구현할 강화학습 작업 유형:

1. **도달 작업 (Reach)**:
   - 목표 위치에 엔드 이펙터 도달
   - 다양한 목표 위치 생성

2. **집기 작업 (Pick)**:
   - 물체 감지 및 위치 파악
   - 물체 집기 (그리퍼 제어 포함)

3. **놓기 작업 (Place)**:
   - 목표 위치에 물체 놓기
   - 정확한 위치 및 방향 제어

4. **순서 작업 (Sequential)**:
   - 여러 물체를 순서대로 정리
   - 복잡한 작업 계획 및 실행

### 5.2 과제 난이도 단계

다양한 난이도의 작업 지원:

1. **초급 과제**:
   - 고정된 목표 위치
   - 장애물 없음
   - 충분한 시간 제한

2. **중급 과제**:
   - 움직이는 목표
   - 정적 장애물 회피
   - 엄격한 시간 제한

3. **고급 과제**:
   - 움직이는 장애물
   - 불확실성 및 노이즈
   - 복합 작업 연속 실행

## 6. 구현 계획

### 6.1 단계별 구현 계획

1. **기본 환경 구조 구현** (2일)
   - 환경 클래스 구현
   - 인터페이스 정의
   - 로봇 로드 및 초기화

2. **관찰 및 액션 공간 구현** (1일)
   - 상태 기반 관찰 공간
   - 관절 위치 제어 액션 공간

3. **도달 작업 구현** (2일)
   - 랜덤 목표 생성
   - 거리 기반 보상 함수
   - 성공 조건 정의

4. **픽셀 기반 관찰 구현** (1일)
   - 카메라 설정 및 렌더링
   - 이미지 처리 및 관찰 통합

5. **그리퍼 제어 및 물체 조작 구현** (2일)
   - 물체 생성 및 물리 시뮬레이션
   - 그리퍼 제어 및 파지 감지

6. **기존 강화학습 알고리즘 통합** (1일)
   - 환경 팩토리 구현
   - 모델 훈련 스크립트 조정

7. **테스트 및 성능 최적화** (2일)
   - 환경 동작 테스트
   - 학습 성능 평가 및 최적화

### 6.2 구현 우선순위

1. 기본 도달 작업 환경 (필수)
2. 상태 기반 관찰 (필수)
3. 기존 강화학습 알고리즘 통합 (필수)
4. 픽셀 기반 관찰 (중요)
5. 물체 조작 작업 (중요)
6. 복합 작업 (옵션)

## 7. 테스트 계획

### 7.1 환경 테스트

1. **기능 테스트**:
   - 환경 생성 및 초기화 테스트
   - 리셋 및 스텝 함수 테스트
   - 관찰 및 액션 공간 테스트

2. **성능 테스트**:
   - 시뮬레이션 속도 측정
   - 메모리 사용량 모니터링
   - 렌더링 성능 평가

### 7.2 학습 테스트

1. **학습 성능 테스트**:
   - 여러 강화학습 알고리즘 비교
   - 학습 곡선 분석
   - 수렴 속도 및 안정성 평가

2. **전이 테스트**:
   - Isaac Sim과 PyBullet 환경 간 정책 전이
   - 도메인 랜덤화 효과 평가
   - 실제 로봇 전이 가능성 평가

## 8. 기대 효과 및 장점

1. **사실적 시뮬레이션**:
   - 물리 기반 고품질 시뮬레이션
   - 실제와 유사한 물리적 상호작용

2. **시각적 요소 활용**:
   - 고품질 렌더링 기반 시각 정보
   - 컴퓨터 비전 알고리즘 통합 용이

3. **확장성**:
   - 다양한 로봇 모델 지원
   - 복잡한 작업 구현 가능

4. **Sim-to-Real 전이**:
   - 시뮬레이션과 실제 간 차이 감소
   - 실제 로봇 적용 가능성 향상