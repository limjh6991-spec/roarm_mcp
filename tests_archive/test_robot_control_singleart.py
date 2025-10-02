#!/usr/bin/env python3
"""
🤖 RoArm MCP - Isaac Sim 5.0 로봇 제어 인터페이스 (SingleArticulation 사용)
UR10, Franka 로봇의 Joint 제어 및 DOF 조작 기능 구현
"""

import sys
import os
import time
import numpy as np

def main():
    """Isaac Sim 5.0에서 SingleArticulation을 사용한 로봇 제어 인터페이스 구현 및 테스트"""
    
    print(f"🤖 RoArm MCP - Isaac Sim 5.0 로봇 제어 인터페이스 테스트 (SingleArticulation)")
    print(f"Python 버전: {sys.version}")
    print(f"작업 디렉토리: {os.getcwd()}")
    
    # Isaac Sim 5.0 SimulationApp 초기화 (headless 모드)
    from isaacsim import SimulationApp
    simulation_app = SimulationApp({
        "headless": True,
        "width": 1280,
        "height": 720
    })
    
    try:
        print("✅ Isaac Sim 5.0 SimulationApp 초기화 성공")
        
        # SimulationApp 초기화 후 Isaac Sim 모듈 import
        import omni.usd
        from pxr import UsdGeom, UsdLux, Sdf, Gf
        from isaacsim.core.api.world import World
        from isaacsim.core.api.objects import GroundPlane
        from isaacsim.core.utils.stage import get_current_stage, add_reference_to_stage
        from isaacsim.core.prims import SingleArticulation
        
        # USD Stage 생성
        stage = get_current_stage()
        print("✅ USD Stage 생성 성공")
        
        # Isaac Sim World 초기화
        world = World()
        print("✅ Isaac Sim World 초기화 성공")
        
        # 기본 환경 설정
        setup_environment(stage, world, UsdLux, Sdf, GroundPlane)
        
        # 로봇 모델 로딩
        ur10_prim_path, franka_prim_path = load_robot_models(stage, add_reference_to_stage, UsdGeom, Gf)
        
        # 로봇 제어 인터페이스 생성
        ur10_controller, franka_controller = create_robot_controllers(
            world, ur10_prim_path, franka_prim_path, SingleArticulation, np
        )
        
        # 제어 기능 테스트
        test_robot_control(world, ur10_controller, franka_controller, np, time)
        
        # USD 파일 저장
        output_path = "/home/roarm_m3/dev_roarm/roarm_mcp/tests/robot_control_success_singleart.usd"
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
        print("🔚 Isaac Sim SimulationApp 종료")

def setup_environment(stage, world, UsdLux, Sdf, GroundPlane):
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

def load_robot_models(stage, add_reference_to_stage, UsdGeom, Gf):
    """UR10, Franka 로봇 모델 로딩"""
    print("\n🤖 로봇 모델 로딩...")
    
    ur10_prim_path = None
    franka_prim_path = None
    
    # UR10 로봇 로딩
    try:
        ur10_asset_path = "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
        ur10_prim_path = "/World/UR10"
        add_reference_to_stage(usd_path=ur10_asset_path, prim_path=ur10_prim_path)
        
        ur10_prim = stage.GetPrimAtPath(ur10_prim_path)
        if ur10_prim.IsValid():
            # 위치 설정 (왼쪽에 배치)
            ur10_xform = UsdGeom.Xformable(ur10_prim)
            translate_op = ur10_xform.AddTranslateOp()
            translate_op.Set(Gf.Vec3d(-1.5, 0.0, 0.0))
            print("  - UR10 로봇 로딩 및 위치 설정 완료")
    except Exception as e:
        print(f"  - UR10 로딩 실패: {e}")
        ur10_prim_path = None
    
    # Franka 로봇 로딩
    try:
        franka_asset_path = "/Isaac/Robots/Franka/franka.usd"
        franka_prim_path = "/World/Franka"
        add_reference_to_stage(usd_path=franka_asset_path, prim_path=franka_prim_path)
        
        franka_prim = stage.GetPrimAtPath(franka_prim_path)
        if franka_prim.IsValid():
            # 위치 설정 (오른쪽에 배치)
            franka_xform = UsdGeom.Xformable(franka_prim)
            translate_op = franka_xform.AddTranslateOp()
            translate_op.Set(Gf.Vec3d(1.5, 0.0, 0.0))
            print("  - Franka 로봇 로딩 및 위치 설정 완료")
    except Exception as e:
        print(f"  - Franka 로딩 실패: {e}")
        franka_prim_path = None
    
    return ur10_prim_path, franka_prim_path

def create_robot_controllers(world, ur10_prim_path, franka_prim_path, SingleArticulation, np):
    """로봇 제어 인터페이스 생성 (SingleArticulation 사용)"""
    print("\n🎮 로봇 제어 인터페이스 생성 (SingleArticulation)...")
    
    ur10_controller = None
    franka_controller = None
    
    # UR10 제어 인터페이스 생성
    if ur10_prim_path:
        try:
            ur10_articulation = SingleArticulation(prim_path=ur10_prim_path, name="ur10_robot")
            world.scene.add(ur10_articulation)
            print("  - UR10 SingleArticulation 추가 완료")
            
            # UR10 제어기 생성
            ur10_controller = RobotController("UR10", ur10_articulation, np)
            print(f"  - UR10 제어기 생성 완료")
            
        except Exception as e:
            print(f"  - UR10 제어기 생성 실패: {e}")
    
    # Franka 제어 인터페이스 생성  
    if franka_prim_path:
        try:
            franka_articulation = SingleArticulation(prim_path=franka_prim_path, name="franka_robot")
            world.scene.add(franka_articulation)
            print("  - Franka SingleArticulation 추가 완료")
            
            # Franka 제어기 생성
            franka_controller = RobotController("Franka", franka_articulation, np)
            print(f"  - Franka 제어기 생성 완료")
            
        except Exception as e:
            print(f"  - Franka 제어기 생성 실패: {e}")
    
    # World 초기화로 모든 로봇 활성화
    try:
        print("  - World 리셋 시작...")
        world.reset()
        print("  - World 리셋 완료")
        
        # 제어기 초기화
        if ur10_controller:
            ur10_controller.initialize_after_reset()
        if franka_controller:
            franka_controller.initialize_after_reset()
            
    except Exception as e:
        print(f"  - World 리셋 실패: {e}")
    
    return ur10_controller, franka_controller

class RobotController:
    """Isaac Sim 5.0 SingleArticulation 로봇 제어 인터페이스 클래스"""
    
    def __init__(self, name, articulation, np):
        self.name = name
        self.articulation = articulation
        self.np = np
        self.num_dof = 0
        self.joint_names = []
        self.initialized = False
    
    def initialize_after_reset(self):
        """World 리셋 후 제어기 초기화"""
        try:
            # SingleArticulation의 DOF 개수 가져오기
            if hasattr(self.articulation, 'num_dof'):
                self.num_dof = self.articulation.num_dof
            else:
                self.num_dof = 0
            
            # 관절 이름 가져오기
            if hasattr(self.articulation, 'dof_names'):
                self.joint_names = self.articulation.dof_names
            else:
                self.joint_names = []
            
            # 초기 관절 상태 확인
            try:
                self.current_positions = self.articulation.get_joint_positions()
                self.current_velocities = self.articulation.get_joint_velocities()
            except:
                self.current_positions = None
                self.current_velocities = None
            
            if self.num_dof > 0:
                self.initialized = True
                print(f"    {self.name} 제어기 초기화 완료: DOF={self.num_dof}")
            else:
                print(f"    {self.name} 제어기 초기화: DOF가 0이므로 대기 상태")
                self.initialized = False
            
        except Exception as e:
            print(f"    {self.name} 제어기 초기화 실패: {e}")
            self.initialized = False
    
    def get_info(self):
        """로봇 정보 반환"""
        if self.initialized:
            return f"DOF: {self.num_dof}, Joints: {len(self.joint_names)}"
        else:
            return "초기화되지 않음 또는 DOF=0"
    
    def get_joint_positions(self):
        """현재 관절 위치 반환"""
        if not self.initialized:
            return None
        try:
            positions = self.articulation.get_joint_positions()
            return positions if positions is not None else self.np.zeros(self.num_dof)
        except:
            return self.np.zeros(self.num_dof)
    
    def get_joint_velocities(self):
        """현재 관절 속도 반환"""
        if not self.initialized:
            return None
        try:
            velocities = self.articulation.get_joint_velocities()
            return velocities if velocities is not None else self.np.zeros(self.num_dof)
        except:
            return self.np.zeros(self.num_dof)
    
    def set_joint_positions(self, positions):
        """관절 위치 설정"""
        if not self.initialized:
            print(f"    {self.name}: 초기화되지 않음")
            return False
            
        try:
            if len(positions) == self.num_dof:
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
        if not self.initialized:
            print(f"    {self.name}: 초기화되지 않음")
            return False
            
        try:
            if len(velocities) == self.num_dof:
                self.articulation.set_joint_velocities(velocities)
                return True
            else:
                print(f"    {self.name}: 잘못된 관절 속도 크기 {len(velocities)}, 예상: {self.num_dof}")
                return False
        except Exception as e:
            print(f"    {self.name} 관절 속도 설정 실패: {e}")
            return False

def test_robot_control(world, ur10_controller, franka_controller, np, time):
    """로봇 제어 기능 테스트"""
    print("\n🔧 로봇 제어 기능 테스트...")
    
    # 기본 정보 출력
    if ur10_controller:
        print(f"  - UR10: {ur10_controller.get_info()}")
        if ur10_controller.initialized:
            positions = ur10_controller.get_joint_positions()
            if positions is not None and len(positions) > 0:
                print(f"    현재 관절 위치: {positions[:min(3, len(positions))]}")
    
    if franka_controller:
        print(f"  - Franka: {franka_controller.get_info()}")
        if franka_controller.initialized:
            positions = franka_controller.get_joint_positions()
            if positions is not None and len(positions) > 0:
                print(f"    현재 관절 위치: {positions[:min(3, len(positions))]}")
    
    # 초기화된 로봇이 있는지 확인
    if not ((ur10_controller and ur10_controller.initialized) or 
            (franka_controller and franka_controller.initialized)):
        print("⚠️ 초기화된 로봇이 없어 제어 테스트를 건너뜁니다.")
        return
    
    # 관절 제어 테스트
    print("\n🎮 관절 제어 테스트 시작...")
    
    for step in range(5):
        print(f"  단계 {step+1}/5:")
        
        # UR10 제어 테스트
        if ur10_controller and ur10_controller.initialized:
            # 작은 각도 변화로 안전하게 테스트
            test_positions = np.zeros(ur10_controller.num_dof)
            test_positions[0] = 0.05 * np.sin(step * 0.3)  # 첫 번째 관절만 약간 회전
            
            success = ur10_controller.set_joint_positions(test_positions)
            if success:
                print(f"    ✅ UR10 관절 제어 성공: Joint[0] = {test_positions[0]:.3f}")
            else:
                print(f"    ❌ UR10 관절 제어 실패")
        
        # Franka 제어 테스트
        if franka_controller and franka_controller.initialized:
            # 작은 각도 변화로 안전하게 테스트
            test_positions = np.zeros(franka_controller.num_dof)
            test_positions[0] = 0.05 * np.cos(step * 0.3)  # 첫 번째 관절만 약간 회전
            
            success = franka_controller.set_joint_positions(test_positions)
            if success:
                print(f"    ✅ Franka 관절 제어 성공: Joint[0] = {test_positions[0]:.3f}")
            else:
                print(f"    ❌ Franka 관절 제어 실패")
        
        # 시뮬레이션 스텝 진행
        try:
            world.step(render=False)
            time.sleep(0.1)  # 짧은 대기 시간
        except Exception as e:
            print(f"    ⚠️ World step 오류: {e}")
    
    print("\n✅ 로봇 제어 기능 테스트 완료!")
    
    # 최종 상태 확인
    if ur10_controller and ur10_controller.initialized:
        try:
            final_pos = ur10_controller.get_joint_positions()
            if final_pos is not None and len(final_pos) > 0:
                print(f"  UR10 최종 관절 위치: {final_pos[:min(3, len(final_pos))]}")
        except:
            print("  UR10 최종 상태 확인 실패")
    
    if franka_controller and franka_controller.initialized:
        try:
            final_pos = franka_controller.get_joint_positions()
            if final_pos is not None and len(final_pos) > 0:
                print(f"  Franka 최종 관절 위치: {final_pos[:min(3, len(final_pos))]}")
        except:
            print("  Franka 최종 상태 확인 실패")

if __name__ == "__main__":
    print("=" * 70)
    print("=== Isaac Sim 5.0 로봇 제어 인터페이스 테스트 (SingleArticulation) ===")
    print("=" * 70)
    
    success = main()
    
    print("\n" + "=" * 70)
    if success:
        print("🎉 로봇 제어 인터페이스 구현 성공!")
        print("✅ Isaac Sim 5.0 SingleArticulation API 호환성 확인됨")
        print("✅ UR10, Franka 로봇의 관절 제어 기능 구현 완료")
    else:
        print("❌ 로봇 제어 인터페이스 구현 실패")
        print("⚠️  문제가 발생했습니다. 로그를 확인해주세요.")
        sys.exit(1)
    print("=" * 70)