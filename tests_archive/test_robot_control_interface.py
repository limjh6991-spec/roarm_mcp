#!/usr/bin/env python3
"""
🤖 RoArm MCP - Isaac Sim 로봇 제어 인터페이스 구현
UR10, Franka 로봇의 Joint 제어 및 DOF 조작 기능 구현
"""

import sys
import os
import numpy as np
import time
print(f"🤖 RoArm MCP - Isaac Sim 로봇 제어 인터페이스 테스트")
print(f"Python 버전: {sys.version}")
print(f"작업 디렉토리: {os.getcwd()}")

def test_robot_control_interface():
    """Isaac Sim에서 UR10, Franka 로봇 제어 인터페이스 구현 및 테스트"""
    
    # Isaac Sim 5.0 SimulationApp 초기화
    from isaacsim import SimulationApp
    simulation_app = SimulationApp({
        "headless": True,
        "width": 1280,
        "height": 720
    })
    
    try:
        print("✅ Isaac Sim 5.0 SimulationApp 초기화 성공")
        
        # Isaac Sim 5.0 호환 모듈 import
        import omni.usd
        from pxr import UsdGeom, UsdLux, Sdf, Gf
        from isaacsim.core.utils.stage import get_current_stage, add_reference_to_stage
        from isaacsim.core.api.world import World
        from isaacsim.core.api.objects import GroundPlane
        from isaacsim.core.prims import SingleArticulation
        
        # USD Stage 생성
        stage = get_current_stage()
        print("✅ USD Stage 생성 성공")
        
        # Isaac Sim World 초기화
        world = World()
        print("✅ Isaac Sim World 초기화 성공")
        
        # 기본 환경 설정
        setup_environment(stage, world)
        
        # 로봇 모델 로딩
        ur10_robot, franka_robot = load_robot_models(stage, world)
        
        # 로봇 제어 인터페이스 생성
        ur10_controller, franka_controller = create_robot_controllers(stage, world, ur10_robot, franka_robot)
        
        # 제어 기능 테스트
        test_robot_control(world, ur10_controller, franka_controller)
        
        # USD 파일 저장
        output_path = "/home/roarm_m3/dev_roarm/roarm_mcp/tests/robot_control_test_success.usd"
        stage.Export(output_path)
        print(f"✅ USD 파일 저장 완료: {output_path}")
        
        return True
        
    except Exception as e:
        print(f"❌ 로봇 제어 인터페이스 테스트 오류: {e}")
        import traceback
        traceback.print_exc()
        return False
        
    finally:
        # 안전한 종료
        simulation_app.close()

def setup_environment(stage, world):
    """기본 시뮬레이션 환경 설정"""
    print("\n🌟 시뮬레이션 환경 설정...")
    
    # 조명 설정
    light_path = Sdf.Path("/World/defaultLight")
    light_prim = UsdLux.DistantLight.Define(stage, light_path)
    distant_light = UsdLux.DistantLight(stage.GetPrimAtPath(light_path))
    distant_light.CreateIntensityAttr(3000.0)
    print("  - 기본 조명 설정 완료")
    
    # 바닥 추가
    ground_plane = GroundPlane(prim_path="/World/groundPlane")
    world.scene.add(ground_plane)
    print("  - 바닥 평면 추가 완료")
    
    # World 초기화
    world.reset()
    print("  - World 초기화 완료")

def load_robot_models(stage, world):
    """UR10, Franka 로봇 모델 로딩"""
    print("\n🤖 로봇 모델 로딩...")
    
    ur10_robot = None
    franka_robot = None
    
    # UR10 로봇 로딩
    try:
        ur10_asset_path = "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
        ur10_prim_path = "/World/UR10"
        add_reference_to_stage(usd_path=ur10_asset_path, prim_path=ur10_prim_path)
        
        ur10_prim = stage.GetPrimAtPath(ur10_prim_path)
        if ur10_prim.IsValid():
            # 위치 설정
            ur10_xform = UsdGeom.Xformable(ur10_prim)
            if not ur10_xform.GetOrderedXformOps():
                translate_op = ur10_xform.AddTranslateOp()
                translate_op.Set(Gf.Vec3d(-1.0, 0.0, 0.0))  # 왼쪽에 배치
            print("  - UR10 로봇 로딩 및 위치 설정 완료")
            ur10_robot = ur10_prim_path
    except Exception as e:
        print(f"  - UR10 로딩 실패: {e}")
    
    # Franka 로봇 로딩
    try:
        franka_asset_path = "/Isaac/Robots/Franka/franka.usd"
        franka_prim_path = "/World/Franka"
        add_reference_to_stage(usd_path=franka_asset_path, prim_path=franka_prim_path)
        
        franka_prim = stage.GetPrimAtPath(franka_prim_path)
        if franka_prim.IsValid():
            # 위치 설정
            franka_xform = UsdGeom.Xformable(franka_prim)
            if not franka_xform.GetOrderedXformOps():
                translate_op = franka_xform.AddTranslateOp()
                translate_op.Set(Gf.Vec3d(1.0, 0.0, 0.0))  # 오른쪽에 배치
            print("  - Franka 로봇 로딩 및 위치 설정 완료")
            franka_robot = franka_prim_path
    except Exception as e:
        print(f"  - Franka 로딩 실패: {e}")
    
    return ur10_robot, franka_robot

def create_robot_controllers(stage, world, ur10_robot, franka_robot):
    """로봇 제어 인터페이스 생성"""
    print("\n🎮 로봇 제어 인터페이스 생성...")
    
    ur10_controller = None
    franka_controller = None
    
    # UR10 제어 인터페이스 생성
    if ur10_robot:
        try:
            ur10_articulation = SingleArticulation(prim_path=ur10_robot, name="ur10_articulation")
            world.scene.add(ur10_articulation)
            
            # World 리셋으로 Articulation 초기화
            world.reset()
            
            # UR10 제어기 생성
            ur10_controller = RobotController("UR10", ur10_articulation)
            print(f"  - UR10 제어기 생성 완료: {ur10_controller.get_info()}")
            
        except Exception as e:
            print(f"  - UR10 제어기 생성 실패: {e}")
    
    # Franka 제어 인터페이스 생성  
    if franka_robot:
        try:
            franka_articulation = SingleArticulation(prim_path=franka_robot, name="franka_articulation")
            world.scene.add(franka_articulation)
            
            # World 리셋으로 Articulation 초기화
            world.reset()
            
            # Franka 제어기 생성
            franka_controller = RobotController("Franka", franka_articulation)
            print(f"  - Franka 제어기 생성 완료: {franka_controller.get_info()}")
            
        except Exception as e:
            print(f"  - Franka 제어기 생성 실패: {e}")
    
    return ur10_controller, franka_controller

class RobotController:
    """로봇 제어 인터페이스 클래스"""
    
    def __init__(self, name, articulation):
        self.name = name
        self.articulation = articulation
        self._initialize_controller()
    
    def _initialize_controller(self):
        """제어기 초기화"""
        try:
            self.num_dof = self.articulation.num_dof
            self.joint_names = self.articulation.get_joint_names()
            
            # 초기 관절 상태 확인
            self.current_positions = self.articulation.get_joint_positions()
            self.current_velocities = self.articulation.get_joint_velocities()
            
            print(f"    {self.name} 제어기 초기화 완료")
            
        except Exception as e:
            print(f"    {self.name} 제어기 초기화 실패: {e}")
            self.num_dof = 0
            self.joint_names = []
            self.current_positions = None
            self.current_velocities = None
    
    def get_info(self):
        """로봇 정보 반환"""
        return f"DOF: {self.num_dof}, Joints: {len(self.joint_names) if self.joint_names else 0}"
    
    def get_joint_positions(self):
        """현재 관절 위치 반환"""
        try:
            return self.articulation.get_joint_positions()
        except:
            return np.zeros(self.num_dof) if self.num_dof > 0 else None
    
    def get_joint_velocities(self):
        """현재 관절 속도 반환"""
        try:
            return self.articulation.get_joint_velocities()
        except:
            return np.zeros(self.num_dof) if self.num_dof > 0 else None
    
    def set_joint_positions(self, positions):
        """관절 위치 설정"""
        try:
            if self.num_dof > 0 and len(positions) == self.num_dof:
                self.articulation.set_joint_positions(positions)
                return True
            else:
                print(f"    {self.name}: 잘못된 관절 위치 크기 {len(positions)}, 예상: {self.num_dof}")
                return False
        except Exception as e:
            print(f"    {self.name} 관절 위치 설정 실패: {e}")
            return False
    
    def set_joint_velocities(self, velocities):
        """관절 속도 설정"""
        try:
            if self.num_dof > 0 and len(velocities) == self.num_dof:
                self.articulation.set_joint_velocities(velocities)
                return True
            else:
                print(f"    {self.name}: 잘못된 관절 속도 크기 {len(velocities)}, 예상: {self.num_dof}")
                return False
        except Exception as e:
            print(f"    {self.name} 관절 속도 설정 실패: {e}")
            return False

def test_robot_control(world, ur10_controller, franka_controller):
    """로봇 제어 기능 테스트"""
    print("\n🔧 로봇 제어 기능 테스트...")
    
    # 기본 정보 출력
    if ur10_controller:
        print(f"  - UR10: {ur10_controller.get_info()}")
        positions = ur10_controller.get_joint_positions()
        if positions is not None:
            print(f"    현재 관절 위치: {positions[:3] if len(positions) > 3 else positions}")
    
    if franka_controller:
        print(f"  - Franka: {franka_controller.get_info()}")
        positions = franka_controller.get_joint_positions()
        if positions is not None:
            print(f"    현재 관절 위치: {positions[:3] if len(positions) > 3 else positions}")
    
    # 관절 제어 테스트
    print("\n🎮 관절 제어 테스트 시작...")
    
    for step in range(5):
        print(f"  단계 {step+1}/5:")
        
        # UR10 제어 테스트
        if ur10_controller and ur10_controller.num_dof > 0:
            # 작은 각도 변화로 안전하게 테스트
            test_positions = np.zeros(ur10_controller.num_dof)
            test_positions[0] = 0.1 * np.sin(step * 0.5)  # 첫 번째 관절만 약간 회전
            
            success = ur10_controller.set_joint_positions(test_positions)
            if success:
                print(f"    UR10 관절 제어 성공: Joint[0] = {test_positions[0]:.3f}")
            else:
                print(f"    UR10 관절 제어 실패")
        
        # Franka 제어 테스트
        if franka_controller and franka_controller.num_dof > 0:
            # 작은 각도 변화로 안전하게 테스트
            test_positions = np.zeros(franka_controller.num_dof)
            test_positions[0] = 0.1 * np.cos(step * 0.5)  # 첫 번째 관절만 약간 회전
            
            success = franka_controller.set_joint_positions(test_positions)
            if success:
                print(f"    Franka 관절 제어 성공: Joint[0] = {test_positions[0]:.3f}")
            else:
                print(f"    Franka 관절 제어 실패")
        
        # 시뮬레이션 스텝 진행
        world.step(render=False)
        time.sleep(0.1)  # 잠시 대기
    
    print("\n✅ 로봇 제어 기능 테스트 완료!")

if __name__ == "__main__":
    print("=" * 60)
    print("=== Isaac Sim 로봇 제어 인터페이스 테스트 ===")
    success = test_robot_control_interface()
    if success:
        print("\n🎉 로봇 제어 인터페이스 구현 성공!")
    else:
        print("\n❌ 로봇 제어 인터페이스 구현 실패")
        sys.exit(1)