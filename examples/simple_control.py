"""
간단한 로봇팔 제어 예제
"""

import sys
import os
import yaml
import argparse
import time
from pathlib import Path

# 프로젝트 루트 디렉토리 추가
project_root = Path(__file__).parent.parent
sys.path.append(str(project_root))

# 로봇팔 인터페이스 임포트
from robot_arm.arm_interface import ArmInterface
from robot_arm.simulation.gym_env import RoArmEnv
from robot_arm.simulation.visualization import RobotVisualizer
from robot_arm.hardware.serial_interface import SerialArmInterface

def parse_args():
    """명령행 인자 파싱"""
    parser = argparse.ArgumentParser(description="간단한 로봇팔 제어 예제")
    parser.add_argument("--sim", action="store_true", help="시뮬레이션 모드 사용")
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0", help="하드웨어 연결 시리얼 포트")
    parser.add_argument("--visualize", action="store_true", help="시각화 활성화")
    return parser.parse_args()

def main():
    """메인 함수"""
    args = parse_args()
    
    # 설정 로드
    config_path = project_root / "config" / "robot_config.yaml"
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    # 로봇팔 인터페이스 생성
    if args.sim:
        print("시뮬레이션 모드에서 로봇팔 초기화 중...")
        # 시뮬레이션 환경 생성
        env = RoArmEnv(render_mode="human" if args.visualize else None)
        obs, _ = env.reset()
        
        # 몇 가지 기본 동작 수행
        for _ in range(100):
            action = [0.01, 0.01, 0.01, 0.0, 0.0, 0.0]  # 단순 움직임 예시
            obs, reward, terminated, truncated, info = env.step(action)
            print(f"보상: {reward}, 목표까지 거리: {info['distance_to_target']}")
            time.sleep(0.01)
            if terminated or truncated:
                break
        
        if args.visualize:
            # 시뮬레이션은 자체 렌더링을 사용
            pass
            
    else:
        print(f"하드웨어 모드에서 로봇팔 초기화 중... 포트: {args.port}")
        arm = SerialArmInterface(port=args.port)
        
        # 로봇팔 연결
        if arm.connect():
            print("로봇팔 연결 성공!")
            
            # 현재 관절 위치 조회
            joint_pos = arm.get_joint_positions()
            print(f"현재 관절 위치: {joint_pos}")
            
            # 현재 카르테시안 위치 조회
            cart_pos = arm.get_cartesian_position()
            print(f"현재 위치: {cart_pos}")
            
            # 간단한 관절 이동
            print("관절 이동 중...")
            new_joint_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 중립 위치
            arm.move_joints(new_joint_pos, speed=0.5)
            time.sleep(2.0)  # 이동 완료 대기
            
            # 그리퍼 제어
            print("그리퍼 열기...")
            arm.gripper_control(0.08, speed=0.5)  # 최대로 열기
            time.sleep(1.0)
            
            print("그리퍼 닫기...")
            arm.gripper_control(0.0, speed=0.5)  # 완전히 닫기
            time.sleep(1.0)
            
            # 카르테시안 이동
            print("카르테시안 이동 중...")
            new_cart_pos = [0.2, 0.0, 0.3, 0.0, 3.14, 0.0]  # 예시 위치
            arm.move_cartesian(new_cart_pos, speed=0.5)
            time.sleep(2.0)  # 이동 완료 대기
            
            # 로봇 상태 확인
            status = arm.get_status()
            print(f"로봇 상태: {status}")
            
            # 연결 해제
            arm.disconnect()
            print("로봇팔 연결 해제")
            
            # 시각화 (요청된 경우)
            if args.visualize:
                print("로봇팔 시각화 중...")
                visualizer = RobotVisualizer()
                visualizer.plot_robot(joint_pos, new_cart_pos[:3])  # xyz만 사용
                
        else:
            print("로봇팔 연결 실패!")

if __name__ == "__main__":
    main()