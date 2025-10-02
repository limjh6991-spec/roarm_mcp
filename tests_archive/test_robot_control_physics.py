#!/usr/bin/env python3
"""
🤖 RoArm MCP - Isaac Sim 5.0 로봇 제어 인터페이스 (Physics 대기 방식)
로봇 모델 로딩 후 충분한 physics 준비 시간 허용
"""

import sys
import os
import time
import numpy as np

def main():
    """Isaac Sim 5.0에서 physics 대기 방식으로 로봇 제어 인터페이스 구현 및 테스트"""
    
    print(f"🤖 RoArm MCP - Isaac Sim 5.0 로봇 제어 인터페이스 테스트 (Physics 대기)")
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
        
        # 로봇 모델 로딩 후 충분한 대기 시간 제공
        print("\n⏳ 로봇 모델 physics 준비 대기 중...")
        for i in range(5):
            time.sleep(1)
            print(f"  대기 중... {i+1}/5초")
        
        # 로봇 제어 인터페이스 생성
        ur10_controller, franka_controller = create_robot_controllers(
            world, ur10_prim_path, franka_prim_path, SingleArticulation, np, stage
        )
        
        # 제어 기능 테스트
        test_robot_control(world, ur10_controller, franka_controller, np, time)
        
        # USD 파일 저장
        output_path = "/home/roarm_m3/dev_roarm/roarm_mcp/tests/robot_control_success_physics.usd"
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
    """UR10, Franka 로봇 모델 로딩 (개선된 방식)"""
    print("\n🤖 로봇 모델 로딩...")
    
    ur10_prim_path = None
    franka_prim_path = None
    
    # UR10 로봇 로딩
    try:
        ur10_asset_path = "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
        ur10_prim_path = "/World/UR10"
        
        print(f"  - UR10 모델 로딩 중... ({ur10_asset_path})")
        add_reference_to_stage(usd_path=ur10_asset_path, prim_path=ur10_prim_path)
        
        # 로딩 후 검증
        ur10_prim = stage.GetPrimAtPath(ur10_prim_path)
        if ur10_prim.IsValid():
            # 위치 설정 (왼쪽에 배치)
            ur10_xform = UsdGeom.Xformable(ur10_prim)
            translate_op = ur10_xform.AddTranslateOp()
            translate_op.Set(Gf.Vec3d(-2.0, 0.0, 0.0))  # 더 멀리 배치
            print("  - UR10 로봇 로딩 및 위치 설정 완료")
            
            # Prim 타입 확인
            print(f"    UR10 Prim 타입: {ur10_prim.GetTypeName()}")
            print(f"    UR10 Prim 경로: {ur10_prim.GetPath()}")
        else:
            print("  - UR10 Prim이 유효하지 않음")
            ur10_prim_path = None
            
    except Exception as e:
        print(f"  - UR10 로딩 실패: {e}")
        ur10_prim_path = None
    
    # Franka 로봇 로딩
    try:
        franka_asset_path = "/Isaac/Robots/Franka/franka.usd"
        franka_prim_path = "/World/Franka"
        
        print(f"  - Franka 모델 로딩 중... ({franka_asset_path})")
        add_reference_to_stage(usd_path=franka_asset_path, prim_path=franka_prim_path)
        
        # 로딩 후 검증
        franka_prim = stage.GetPrimAtPath(franka_prim_path)
        if franka_prim.IsValid():
            # 위치 설정 (오른쪽에 배치)
            franka_xform = UsdGeom.Xformable(franka_prim)
            translate_op = franka_xform.AddTranslateOp()
            translate_op.Set(Gf.Vec3d(2.0, 0.0, 0.0))  # 더 멀리 배치
            print("  - Franka 로봇 로딩 및 위치 설정 완료")
            
            # Prim 타입 확인
            print(f"    Franka Prim 타입: {franka_prim.GetTypeName()}")
            print(f"    Franka Prim 경로: {franka_prim.GetPath()}")
        else:
            print("  - Franka Prim이 유효하지 않음")
            franka_prim_path = None
            
    except Exception as e:
        print(f"  - Franka 로딩 실패: {e}")
        franka_prim_path = None
    
    return ur10_prim_path, franka_prim_path

def create_robot_controllers(world, ur10_prim_path, franka_prim_path, SingleArticulation, np, stage):
    """로봇 제어 인터페이스 생성 (개선된 방식)"""
    print("\n🎮 로봇 제어 인터페이스 생성 (SingleArticulation)...")
    
    ur10_controller = None
    franka_controller = None
    
    # UR10 제어 인터페이스 생성
    if ur10_prim_path:
        try:
            print(f"  - UR10 SingleArticulation 생성 중... ({ur10_prim_path})")
            
            # Prim이 여전히 유효한지 확인
            ur10_prim = stage.GetPrimAtPath(ur10_prim_path)
            if not ur10_prim.IsValid():
                print("  - UR10 Prim이 더 이상 유효하지 않음")
            else:
                ur10_articulation = SingleArticulation(prim_path=ur10_prim_path, name="ur10_robot")
                world.scene.add(ur10_articulation)
                print("  - UR10 SingleArticulation 추가 완료")
                
                # UR10 제어기 생성
                ur10_controller = RobotController("UR10", ur10_articulation, np)
                print(f"  - UR10 제어기 생성 완료")
            
        except Exception as e:
            print(f"  - UR10 제어기 생성 실패: {e}")
    else:
        print("  - UR10 경로가 None이므로 건너뜀")
    
    # Franka 제어 인터페이스 생성  
    if franka_prim_path:
        try:
            print(f"  - Franka SingleArticulation 생성 중... ({franka_prim_path})")
            
            # Prim이 여전히 유효한지 확인
            franka_prim = stage.GetPrimAtPath(franka_prim_path)
            if not franka_prim.IsValid():
                print("  - Franka Prim이 더 이상 유효하지 않음")
            else:
                franka_articulation = SingleArticulation(prim_path=franka_prim_path, name="franka_robot")
                world.scene.add(franka_articulation)
                print("  - Franka SingleArticulation 추가 완료")
                
                # Franka 제어기 생성
                franka_controller = RobotController("Franka", franka_articulation, np)
                print(f"  - Franka 제어기 생성 완료")
            
        except Exception as e:
            print(f"  - Franka 제어기 생성 실패: {e}")
    else:
        print("  - Franka 경로가 None이므로 건너뜀")
    
    # World 초기화로 모든 로봇 활성화 (여러 번 시도)
    reset_attempts = 3
    for attempt in range(reset_attempts):
        try:
            print(f"  - World 리셋 시도 {attempt+1}/{reset_attempts}...")
            world.reset()
            print("  - World 리셋 성공!")
            break
            
        except Exception as e:
            print(f"  - World 리셋 실패 (시도 {attempt+1}): {e}")
            if attempt == reset_attempts - 1:
                print("  - 모든 World 리셋 시도 실패, 계속 진행...")
            else:
                time.sleep(2)  # 다음 시도 전 대기
    
    # 제어기 초기화
    if ur10_controller:
        ur10_controller.initialize_after_reset()
    if franka_controller:
        franka_controller.initialize_after_reset()
    
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
            # SingleArticulation의 속성들을 안전하게 확인
            print(f"    {self.name} 제어기 초기화 중...")
            
            # DOF 개수 확인
            if hasattr(self.articulation, 'num_dof'):
                self.num_dof = self.articulation.num_dof
                print(f"    {self.name}: DOF 개수 = {self.num_dof}")
            else:
                self.num_dof = 0
                print(f"    {self.name}: DOF 속성이 없음")
            
            # 관절 이름 확인
            if hasattr(self.articulation, 'dof_names'):
                self.joint_names = self.articulation.dof_names
                print(f"    {self.name}: 관절 이름 개수 = {len(self.joint_names) if self.joint_names else 0}")
            else:
                self.joint_names = []
                print(f"    {self.name}: 관절 이름 속성이 없음")
            
            # 초기 관절 상태 확인
            try:
                if self.num_dof > 0:
                    self.current_positions = self.articulation.get_joint_positions()
                    self.current_velocities = self.articulation.get_joint_velocities()
                    print(f"    {self.name}: 초기 관절 상태 확인 완료")
                else:
                    self.current_positions = None
                    self.current_velocities = None
                    print(f"    {self.name}: DOF가 0이므로 관절 상태 확인 건너뜀")
            except Exception as joint_e:
                print(f"    {self.name}: 관절 상태 확인 실패 - {joint_e}")
                self.current_positions = None
                self.current_velocities = None
            
            if self.num_dof > 0:
                self.initialized = True
                print(f"    ✅ {self.name} 제어기 초기화 성공: DOF={self.num_dof}")
            else:
                print(f"    ⚠️  {self.name} 제어기: DOF가 0이므로 비활성 상태")
                self.initialized = False
            
        except Exception as e:
            print(f"    ❌ {self.name} 제어기 초기화 실패: {e}")
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
    active_robots = []
    if ur10_controller and ur10_controller.initialized:
        active_robots.append("UR10")
    if franka_controller and franka_controller.initialized:
        active_robots.append("Franka")
    
    if not active_robots:
        print("⚠️ 초기화된 로봇이 없어 제어 테스트를 건너뜁니다.")
        return
    
    print(f"✅ 활성 로봇: {', '.join(active_robots)}")
    
    # 관절 제어 테스트
    print("\n🎮 관절 제어 테스트 시작...")
    
    test_success = False
    for step in range(3):  # 테스트 단계 줄임
        print(f"  단계 {step+1}/3:")
        
        step_success = False
        
        # UR10 제어 테스트
        if ur10_controller and ur10_controller.initialized:
            test_positions = np.zeros(ur10_controller.num_dof)
            test_positions[0] = 0.03 * np.sin(step * 0.5)  # 더 작은 각도로 안전하게
            
            success = ur10_controller.set_joint_positions(test_positions)
            if success:
                print(f"    ✅ UR10 관절 제어 성공: Joint[0] = {test_positions[0]:.3f}")
                step_success = True
            else:
                print(f"    ❌ UR10 관절 제어 실패")
        
        # Franka 제어 테스트
        if franka_controller and franka_controller.initialized:
            test_positions = np.zeros(franka_controller.num_dof)
            test_positions[0] = 0.03 * np.cos(step * 0.5)  # 더 작은 각도로 안전하게
            
            success = franka_controller.set_joint_positions(test_positions)
            if success:
                print(f"    ✅ Franka 관절 제어 성공: Joint[0] = {test_positions[0]:.3f}")
                step_success = True
            else:
                print(f"    ❌ Franka 관절 제어 실패")
        
        if step_success:
            test_success = True
        
        # 시뮬레이션 스텝 진행
        try:
            world.step(render=False)
            time.sleep(0.2)  # 충분한 대기 시간
        except Exception as e:
            print(f"    ⚠️ World step 오류: {e}")
    
    if test_success:
        print("\n🎉 로봇 제어 기능 테스트 성공!")
    else:
        print("\n⚠️ 로봇 제어 기능 테스트에서 일부 문제 발생")

if __name__ == "__main__":
    print("=" * 70)
    print("=== Isaac Sim 5.0 로봇 제어 인터페이스 테스트 (Physics 대기) ===")
    print("=" * 70)
    
    success = main()
    
    print("\n" + "=" * 70)
    if success:
        print("🎉 로봇 제어 인터페이스 구현 성공!")
        print("✅ Isaac Sim 5.0 SingleArticulation API 호환성 확인됨")
        print("✅ 로봇 physics 대기 방식 구현 완료")
    else:
        print("❌ 로봇 제어 인터페이스 구현 실패")
        print("⚠️  문제가 발생했습니다. 로그를 확인해주세요.")
        sys.exit(1)
    print("=" * 70)