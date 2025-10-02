#!/usr/bin/env python3
"""
====================================================================
=== Isaac Sim 5.0 ArticulationRootAPI 수동 적용 솔루션 ===
====================================================================

Isaac Sim 설치 환경에서 ArticulationRootAPI를 수동으로 적용하여
Franka와 UR10 로봇의 SingleArticulation을 성공적으로 생성하는 솔루션
"""

import sys
import time
import traceback
import numpy as np
from typing import Optional, Dict, List, Tuple

# Isaac Sim 5.0 헤드리스 모드로 시작
from isaacsim import SimulationApp

CONFIG = {
    "renderer": "RayTracedLighting", 
    "headless": True,
    "width": 1280,
    "height": 720,
}

simulation_app = SimulationApp(CONFIG)

# USD 및 물리 관련 imports
from pxr import Usd, UsdGeom, UsdLux, Gf, Sdf, UsdPhysics
import omni.usd
import omni.kit.app
from omni.isaac.core.utils.stage import add_reference_to_stage, get_current_stage
from omni.isaac.core.utils.prims import get_prim_at_path, create_prim
from omni.isaac.core.world import World
from omni.isaac.core.articulations import Articulation

print("🤖 RoArm MCP - Isaac Sim 5.0 ArticulationRootAPI 수동 적용 솔루션")
print(f"Python 버전: {sys.version}")
import os
print(f"작업 디렉토리: {os.getcwd()}")

class ArticulationRootAPIApplier:
    """ArticulationRootAPI를 수동으로 적용하는 유틸리티"""
    
    @staticmethod
    def find_robot_root_candidates(stage: Usd.Stage, robot_path: str) -> List[str]:
        """
        로봇에서 ArticulationRootAPI 적용 후보 경로들을 찾습니다.
        
        Args:
            stage: USD Stage
            robot_path: 로봇 프림 경로
            
        Returns:
            ArticulationRootAPI 적용 후보 경로 리스트
        """
        print(f"\n🔍 {robot_path}에서 ArticulationRootAPI 적용 후보 탐색...")
        
        candidates = []
        
        # 1. 로봇 루트 경로 자체
        candidates.append(robot_path)
        
        # 2. 관습적인 베이스 링크 패턴들
        base_patterns = [
            "base_link", "base", "robot_base", "root_link",
            "panda_link0",  # Franka 특화
            "ur10_base",    # UR10 특화
            "link0", "link_0"
        ]
        
        for pattern in base_patterns:
            candidate_path = f"{robot_path}/{pattern}"
            candidates.append(candidate_path)
        
        # 3. 실제 존재하는 후보들만 필터링
        valid_candidates = []
        for candidate in candidates:
            prim = stage.GetPrimAtPath(candidate)
            if prim and prim.IsValid():
                valid_candidates.append(candidate)
                print(f"  ✅ 유효한 후보: {candidate}")
        
        print(f"  📊 총 {len(valid_candidates)}개의 유효한 후보 발견")
        return valid_candidates
    
    @staticmethod
    def apply_articulation_root_api(stage: Usd.Stage, prim_path: str) -> bool:
        """
        지정된 프림에 ArticulationRootAPI를 적용합니다.
        
        Args:
            stage: USD Stage
            prim_path: ArticulationRootAPI를 적용할 프림 경로
            
        Returns:
            적용 성공 여부
        """
        try:
            print(f"    🎯 {prim_path}에 ArticulationRootAPI 적용 시도...")
            
            prim = stage.GetPrimAtPath(prim_path)
            if not prim or not prim.IsValid():
                print(f"    ❌ 유효하지 않은 프림: {prim_path}")
                return False
            
            # ArticulationRootAPI 적용
            articulation_root_api = UsdPhysics.ArticulationRootAPI.Apply(prim)
            if not articulation_root_api:
                print(f"    ❌ ArticulationRootAPI 적용 실패: {prim_path}")
                return False
            
            print(f"    ✅ ArticulationRootAPI 적용 성공: {prim_path}")
            return True
            
        except Exception as e:
            print(f"    ❌ ArticulationRootAPI 적용 중 오류: {e}")
            return False
    
    @staticmethod
    def verify_articulation_root_api(stage: Usd.Stage, prim_path: str) -> bool:
        """
        ArticulationRootAPI가 올바르게 적용되었는지 확인합니다.
        """
        try:
            prim = stage.GetPrimAtPath(prim_path)
            if not prim or not prim.IsValid():
                return False
            
            # ArticulationRootAPI 확인
            articulation_root_api = UsdPhysics.ArticulationRootAPI(prim)
            if articulation_root_api:
                print(f"    ✅ ArticulationRootAPI 검증 성공: {prim_path}")
                return True
            else:
                print(f"    ❌ ArticulationRootAPI 검증 실패: {prim_path}")
                return False
                
        except Exception as e:
            print(f"    ❌ ArticulationRootAPI 검증 중 오류: {e}")
            return False

class EnhancedRobotController:
    """ArticulationRootAPI 적용 후 로봇 제어기"""
    
    def __init__(self, robot_name: str, robot_path: str):
        self.robot_name = robot_name
        self.robot_path = robot_path
        self.articulation = None
        self.articulation_root_path = None
        self.initialized = False
        
        print(f"\n🎮 {self.robot_name} 향상된 제어기 생성...")
        
    def apply_articulation_root_and_create(self, stage: Usd.Stage) -> bool:
        """
        ArticulationRootAPI를 적용하고 Articulation 인스턴스를 생성합니다.
        """
        # 1. ArticulationRootAPI 적용 후보 찾기
        candidates = ArticulationRootAPIApplier.find_robot_root_candidates(stage, self.robot_path)
        
        if not candidates:
            print(f"    ❌ {self.robot_name}: ArticulationRootAPI 적용 후보를 찾지 못했습니다")
            return False
        
        # 2. 후보들에 ArticulationRootAPI 적용 시도
        for candidate in candidates:
            print(f"    🔄 {self.robot_name}: {candidate}에 ArticulationRootAPI 적용 시도...")
            
            if ArticulationRootAPIApplier.apply_articulation_root_api(stage, candidate):
                # 적용 후 검증
                if ArticulationRootAPIApplier.verify_articulation_root_api(stage, candidate):
                    self.articulation_root_path = candidate
                    print(f"    ✅ {self.robot_name}: ArticulationRootAPI 적용 및 검증 성공 - {candidate}")
                    break
        
        if not self.articulation_root_path:
            print(f"    ❌ {self.robot_name}: 모든 후보에서 ArticulationRootAPI 적용 실패")
            return False
        
        # 3. Articulation 인스턴스 생성
        try:
            print(f"    🔧 {self.robot_name}: Articulation 인스턴스 생성... (경로: {self.articulation_root_path})")
            self.articulation = Articulation(prim_path=self.articulation_root_path)
            
            if self.articulation:
                print(f"    ✅ {self.robot_name}: Articulation 인스턴스 생성 성공")
                return True
            else:
                print(f"    ❌ {self.robot_name}: Articulation 인스턴스 생성 실패")
                return False
                
        except Exception as e:
            print(f"    ❌ {self.robot_name}: Articulation 생성 중 오류 - {e}")
            return False
    
    def initialize(self) -> bool:
        """로봇 제어기 초기화"""
        if not self.articulation:
            print(f"    ❌ {self.robot_name}: Articulation 인스턴스가 없습니다")
            return False
            
        try:
            print(f"    🔧 {self.robot_name} 제어기 초기화 중... (경로: {self.articulation_root_path})")
            
            # DOF 개수 확인
            dof_count = self.articulation.num_dof
            print(f"      {self.robot_name}: DOF 개수 = {dof_count}")
            
            if dof_count is None or dof_count <= 0:
                print(f"    ❌ {self.robot_name}: 유효하지 않은 DOF 개수")
                return False
            
            # 관절 이름 확인
            joint_names = self.articulation.dof_names
            print(f"      {self.robot_name}: 관절 이름 개수 = {len(joint_names) if joint_names else 0}")
            
            if not joint_names or len(joint_names) == 0:
                print(f"    ❌ {self.robot_name}: 관절 이름이 없습니다")
                return False
            
            print(f"      {self.robot_name}: 관절 이름들 = {joint_names}")
            
            # 관절 상태 확인  
            joint_positions = self.get_joint_positions()
            if joint_positions is not None:
                print(f"      {self.robot_name}: 현재 관절 위치 = {np.round(joint_positions, 3)}")
                self.initialized = True
                print(f"    ✅ {self.robot_name} 제어기 초기화 성공")
                return True
            else:
                print(f"    ❌ {self.robot_name}: 관절 위치를 읽을 수 없습니다")
                return False
                
        except Exception as e:
            print(f"    ❌ {self.robot_name} 제어기 초기화 실패: {e}")
            return False
    
    def get_joint_positions(self) -> Optional[np.ndarray]:
        """현재 관절 위치 가져오기"""
        if not self.articulation:
            return None
        try:
            return self.articulation.get_joint_positions()
        except Exception as e:
            print(f"      {self.robot_name}: 관절 위치 확인 실패 - {e}")
            return None
    
    def set_joint_positions(self, positions: np.ndarray) -> bool:
        """관절 위치 설정"""
        if not self.initialized:
            print(f"      {self.robot_name}: 제어기가 초기화되지 않음")
            return False
        try:
            self.articulation.set_joint_positions(positions)
            return True
        except Exception as e:
            print(f"      {self.robot_name}: 관절 위치 설정 실패 - {e}")
            return False

def setup_physics_scene(stage: Usd.Stage) -> bool:
    """Physics Scene 명시적 생성"""
    try:
        print("🔧 Physics Scene 설정 중...")
        
        physics_scene_path = "/World/physicsScene"
        existing_scene = get_prim_at_path(physics_scene_path)
        
        if existing_scene is not None:
            print("    ✅ 기존 Physics Scene 발견")
            return True
        
        physics_scene = UsdPhysics.Scene.Define(stage, Sdf.Path(physics_scene_path))
        if physics_scene:
            print("    ✅ Physics Scene 생성 완료")
            return True
        else:
            print("    ❌ Physics Scene 생성 실패")
            return False
            
    except Exception as e:
        print(f"    ❌ Physics Scene 설정 실패: {e}")
        return False

def main():
    """메인 테스트 함수"""
    
    robot_controllers: Dict[str, EnhancedRobotController] = {}
    
    try:
        print("✅ Isaac Sim 5.0 SimulationApp 초기화 성공")
        
        # USD Stage 생성
        omni.usd.get_context().new_stage()
        stage = omni.usd.get_context().get_stage()
        if not stage:
            raise RuntimeError("USD Stage 생성 실패")
        print("✅ USD Stage 생성 성공")
        
        # Physics Scene 설정
        if not setup_physics_scene(stage):
            raise RuntimeError("Physics Scene 설정 실패")
        
        # Isaac Sim World 초기화
        world = World()
        print("✅ Isaac Sim World 초기화 성공")
        
        print("\n🌟 시뮬레이션 환경 설정...")
        
        # 기본 조명 설정
        light_prim = create_prim("/World/defaultLight", "DistantLight")
        light = UsdLux.DistantLight(light_prim)
        light.CreateIntensityAttr(1000.0)
        print("  - 기본 조명 설정 완료")
        
        # 바닥 평면 추가
        ground_prim = create_prim("/World/GroundPlane", "Xform")
        add_reference_to_stage("/Isaac/Environments/Simple_Warehouse/warehouse.usd", ground_prim.GetPath())
        print("  - 바닥 평면 추가 완료")
        
        print("\n🤖 로봇 모델 로딩...")
        
        # UR10 로봇 로딩
        print("  - UR10 모델 로딩 중... (/Isaac/Robots/UniversalRobots/ur10/ur10.usd)")
        ur10_prim = create_prim("/World/UR10", "Xform")
        add_reference_to_stage("/Isaac/Robots/UniversalRobots/ur10/ur10.usd", ur10_prim.GetPath())
        
        # UR10 위치 설정 (XformCommonAPI 사용으로 중복 op 방지)
        ur10_xform = UsdGeom.Xformable(ur10_prim)
        UsdGeom.XformCommonAPI(ur10_xform).SetTranslate(Gf.Vec3d(0.0, 0.0, 0.0))
        print("  - UR10 로봇 로딩 및 위치 설정 완료")
        
        # Franka 로봇 로딩
        print("  - Franka 모델 로딩 중... (/Isaac/Robots/Franka/franka.usd)")
        franka_prim = create_prim("/World/Franka", "Xform")
        add_reference_to_stage("/Isaac/Robots/Franka/franka.usd", franka_prim.GetPath())
        
        # Franka 위치 설정 (XformCommonAPI 사용으로 중복 op 방지)
        franka_xform = UsdGeom.Xformable(franka_prim)
        UsdGeom.XformCommonAPI(franka_xform).SetTranslate(Gf.Vec3d(2.0, 0.0, 0.0))
        print("  - Franka 로봇 로딩 및 위치 설정 완료")
        
        # USD 레퍼런스 로딩 대기
        print("\n⏳ USD 레퍼런스 로딩 대기 중...")
        for i in range(10):
            print(f"  대기 중... {i+1}/10초")
            omni.kit.app.get_app().update()
            time.sleep(1.0)
        
        # 로봇 제어기 생성
        print("\n🎮 향상된 로봇 제어 인터페이스 생성...")
        
        ur10_controller = EnhancedRobotController("UR10", "/World/UR10")
        franka_controller = EnhancedRobotController("Franka", "/World/Franka")
        
        robot_controllers["UR10"] = ur10_controller
        robot_controllers["Franka"] = franka_controller
        
        # ArticulationRootAPI 적용 및 Articulation 생성
        print("\n🔧 ArticulationRootAPI 적용 및 Articulation 생성...")
        
        successful_controllers = []
        for robot_name, controller in robot_controllers.items():
            if controller.apply_articulation_root_and_create(stage):
                successful_controllers.append(controller)
            else:
                print(f"    ❌ {robot_name}: ArticulationRootAPI 적용 및 Articulation 생성 실패")
        
        # World 리셋
        print(f"\n🔄 World 리셋 (성공한 로봇: {len(successful_controllers)}개)...")
        
        max_reset_attempts = 3
        for attempt in range(1, max_reset_attempts + 1):
            try:
                print(f"  - World 리셋 시도 {attempt}/{max_reset_attempts}...")
                world.reset()
                print(f"  - World 리셋 성공 (시도 {attempt})")
                break
            except Exception as e:
                print(f"  - World 리셋 실패 (시도 {attempt}): {e}")
                if attempt < max_reset_attempts:
                    print("    2초 후 재시도...")
                    time.sleep(2.0)
                else:
                    print("  - 모든 World 리셋 시도 실패, 계속 진행...")
        
        # 로봇 제어기 초기화
        print("\n🔧 로봇 제어기 초기화...")
        
        initialized_controllers = []
        for controller in successful_controllers:
            if controller.initialize():
                initialized_controllers.append(controller)
        
        # 제어 기능 테스트
        print(f"\n🔧 로봇 제어 기능 테스트 (초기화된 로봇: {len(initialized_controllers)}개)...")
        
        if initialized_controllers:
            for controller in initialized_controllers:
                print(f"  ✅ {controller.robot_name}: 경로={controller.articulation_root_path}, DOF={controller.articulation.num_dof}")
            
            # 간단한 관절 동작 테스트
            print("\n🎯 관절 동작 테스트...")
            
            for controller in initialized_controllers:
                try:
                    current_pos = controller.get_joint_positions()
                    if current_pos is not None:
                        print(f"    {controller.robot_name}: 현재 관절 위치 = {np.round(current_pos, 3)}")
                        
                        # 작은 움직임 테스트
                        test_pos = current_pos.copy()
                        if len(test_pos) > 0:
                            test_pos[0] += 0.1
                        
                            print(f"    {controller.robot_name}: 테스트 위치로 이동 = {np.round(test_pos, 3)}")
                            if controller.set_joint_positions(test_pos):
                                print(f"    ✅ {controller.robot_name}: 관절 위치 설정 성공")
                            else:
                                print(f"    ❌ {controller.robot_name}: 관절 위치 설정 실패")
                    
                except Exception as e:
                    print(f"    ❌ {controller.robot_name}: 동작 테스트 실패 - {e}")
        else:
            print("  ⚠️ 초기화된 로봇이 없어 제어 테스트를 건너뜁니다.")
        
        # USD 파일 저장
        stage_path = "/home/roarm_m3/dev_roarm/roarm_mcp/tests/robot_control_articulation_root_api.usd"
        stage.Export(stage_path)
        print(f"✅ USD 파일 저장 완료: {stage_path}")
        
    except Exception as e:
        print(f"❌ 테스트 실행 중 오류 발생: {e}")
        print("상세 오류 정보:")
        traceback.print_exc()
    
    finally:
        # SimulationApp 종료
        simulation_app.close()
        print("🏁 Isaac Sim 5.0 ArticulationRootAPI 수동 적용 테스트 완료")

if __name__ == "__main__":
    main()