#!/usr/bin/env python3
"""
=======================================================================
=== Isaac Sim 5.0 로봇 제어 인터페이스 테스트 (ArticulationRoot 탐색) ===
=======================================================================

USD ArticulationAPI 레벨에서 올바른 관절 루트를 찾아 처리하는 개선된 버전
- Physics Scene 명시적 생성
- ArticulationRootAPI 기반 관절 루트 자동 탐색  
- 올바른 관절 루트 경로로 SingleArticulation 생성
- 로딩 완료 대기 및 검증 로직 강화
"""

import sys
import time
import traceback
import numpy as np
from typing import Optional, Dict, List, Tuple

# Isaac Sim imports
from isaacsim.simulation_app import SimulationApp

# Isaac Sim 5.0 헤드리스 모드로 시작
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
from isaacsim.core.prims import SingleArticulation

print("🤖 RoArm MCP - Isaac Sim 5.0 로봇 제어 인터페이스 테스트 (ArticulationRoot 탐색)")
print(f"Python 버전: {sys.version}")
import os
print(f"작업 디렉토리: {os.getcwd()}")

class ArticulationRootFinder:
    """USD ArticulationRootAPI 기반 관절 루트 탐색 유틸리티"""
    
    @staticmethod
    def find_articulation_root_path(stage: Usd.Stage, under_path: str = "/World") -> Optional[str]:
        """
        주어진 경로 하위에서 ArticulationRootAPI가 적용된 프림을 찾습니다.
        
        Args:
            stage: USD Stage
            under_path: 탐색할 상위 경로
            
        Returns:
            관절 루트 프림의 경로 (없으면 None)
        """
        under_path = str(under_path)
        print(f"    📡 {under_path} 하위에서 ArticulationRoot 탐색 중...")
        
        for prim in stage.Traverse():
            prim_path = prim.GetPath().pathString
            
            # 지정된 경로 하위가 아니면 스킵
            if not prim_path.startswith(under_path):
                continue
                
            # ArticulationRootAPI가 적용된 프림 확인
            if UsdPhysics.ArticulationRootAPI(prim):
                print(f"    ✅ ArticulationRoot 발견: {prim_path}")
                return prim_path
                
        print(f"    ❌ {under_path} 하위에서 ArticulationRoot를 찾지 못했습니다")
        return None
    
    @staticmethod
    def wait_for_articulation_root(stage: Usd.Stage, under_path: str, timeout: float = 10.0) -> Optional[str]:
        """
        관절 루트가 로딩될 때까지 대기합니다.
        
        Args:
            stage: USD Stage
            under_path: 탐색할 상위 경로  
            timeout: 최대 대기 시간 (초)
            
        Returns:
            관절 루트 프림의 경로 (타임아웃 시 None)
        """
        print(f"    ⏳ {under_path} 관절 루트 로딩 대기 중... (최대 {timeout}초)")
        
        start_time = time.time()
        attempt = 0
        
        while time.time() - start_time < timeout:
            attempt += 1
            
            # 관절 루트 탐색
            root_path = ArticulationRootFinder.find_articulation_root_path(stage, under_path)
            
            if root_path:
                # 프림 존재 확인
                prim = get_prim_at_path(root_path)
                if prim is not None and prim.IsValid():
                    print(f"    ✅ 관절 루트 확인 완료: {root_path} (시도: {attempt})")
                    return root_path
            
            # Kit 프레임 업데이트 (로딩 진행)
            omni.kit.app.get_app().update()
            time.sleep(0.1)
            
            if attempt % 20 == 0:  # 2초마다 진행 상황 출력
                print(f"      대기 중... {attempt//10}/10초")
        
        print(f"    ❌ {under_path} 관절 루트 로딩 타임아웃 ({timeout}초)")
        return None

class EnhancedRobotController:
    """ArticulationRoot 기반 개선된 로봇 제어기"""
    
    def __init__(self, robot_name: str, articulation_root_path: str):
        self.robot_name = robot_name
        self.articulation_root_path = articulation_root_path
        self.articulation = None
        self.initialized = False
        
        print(f"    🎮 {self.robot_name} 제어기 생성 중... ({articulation_root_path})")
        
        try:
            # SingleArticulation 생성 (올바른 관절 루트 경로 사용)
            self.articulation = SingleArticulation(prim_path=articulation_root_path)
            print(f"    ✅ {self.robot_name} SingleArticulation 생성 완료")
            
        except Exception as e:
            print(f"    ❌ {self.robot_name} SingleArticulation 생성 실패: {e}")
            self.articulation = None
    
    def initialize(self) -> bool:
        """로봇 제어기 초기화"""
        if not self.articulation:
            print(f"    ❌ {self.robot_name}: SingleArticulation이 없습니다")
            return False
            
        try:
            print(f"    🔧 {self.robot_name} 제어기 초기화 중...")
            
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
    
    def set_joint_velocities(self, velocities: np.ndarray) -> bool:
        """관절 속도 설정"""
        if not self.initialized:
            print(f"      {self.robot_name}: 제어기가 초기화되지 않음")
            return False
        try:
            self.articulation.set_joint_velocities(velocities)
            return True
        except Exception as e:
            print(f"      {self.robot_name}: 관절 속도 설정 실패 - {e}")
            return False

def setup_physics_scene(stage: Usd.Stage) -> bool:
    """Physics Scene 명시적 생성"""
    try:
        print("🔧 Physics Scene 설정 중...")
        
        # Physics Scene이 이미 있는지 확인
        physics_scene_path = "/World/physicsScene"
        existing_scene = get_prim_at_path(physics_scene_path)
        
        if existing_scene is not None:
            print("    ✅ 기존 Physics Scene 발견")
            return True
        
        # 새로운 Physics Scene 생성
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
        print(f"    UR10 기존 xformOpOrder = {[op.GetOpName() for op in ur10_xform.GetOrderedXformOps()]}")
        UsdGeom.XformCommonAPI(ur10_xform).SetTranslate(Gf.Vec3d(0.0, 0.0, 0.0))
        
        print("  - UR10 로봇 로딩 및 위치 설정 완료")
        print(f"    UR10 Prim 타입: {ur10_prim.GetTypeName()}")
        print(f"    UR10 Prim 경로: {ur10_prim.GetPath()}")
        
        # Franka 로봇 로딩  
        print("  - Franka 모델 로딩 중... (/Isaac/Robots/Franka/franka.usd)")
        franka_prim = create_prim("/World/Franka", "Xform") 
        add_reference_to_stage("/Isaac/Robots/Franka/franka.usd", franka_prim.GetPath())
        
        # Franka 위치 설정 (XformCommonAPI 사용으로 중복 op 방지)
        franka_xform = UsdGeom.Xformable(franka_prim)
        print(f"    Franka 기존 xformOpOrder = {[op.GetOpName() for op in franka_xform.GetOrderedXformOps()]}")
        UsdGeom.XformCommonAPI(franka_xform).SetTranslate(Gf.Vec3d(2.0, 0.0, 0.0))
        
        print("  - Franka 로봇 로딩 및 위치 설정 완료")
        print(f"    Franka Prim 타입: {franka_prim.GetTypeName()}")
        print(f"    Franka Prim 경로: {franka_prim.GetPath()}")
        
        # USD 레퍼런스 로딩 대기
        print("\n⏳ USD 레퍼런스 로딩 대기 중...")
        for i in range(10):
            print(f"  대기 중... {i+1}/10초")
            omni.kit.app.get_app().update()
            time.sleep(1.0)
        
        print("\n🔍 ArticulationRoot 탐색...")
        
        # UR10 관절 루트 찾기
        ur10_root_path = ArticulationRootFinder.wait_for_articulation_root(
            stage, "/World/UR10", timeout=10.0
        )
        
        # Franka 관절 루트 찾기  
        franka_root_path = ArticulationRootFinder.wait_for_articulation_root(
            stage, "/World/Franka", timeout=10.0
        )
        
        print(f"\n📍 발견된 관절 루트:")
        print(f"  - UR10: {ur10_root_path}")
        print(f"  - Franka: {franka_root_path}")
        
        if not ur10_root_path:
            print("⚠️ UR10 ArticulationRoot를 찾지 못했습니다")
        if not franka_root_path:
            print("⚠️ Franka ArticulationRoot를 찾지 못했습니다")
        
        print("\n🎮 로봇 제어 인터페이스 생성 (ArticulationRoot 기반)...")
        
        # UR10 제어기 생성
        if ur10_root_path:
            ur10_controller = EnhancedRobotController("UR10", ur10_root_path)
            robot_controllers["UR10"] = ur10_controller
        
        # Franka 제어기 생성
        if franka_root_path:
            franka_controller = EnhancedRobotController("Franka", franka_root_path)
            robot_controllers["Franka"] = franka_controller
        
        # World 리셋 (ArticulationRoot 기반)
        print("\n🔄 World 리셋 (ArticulationRoot 준비)...")
        
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
        for robot_name, controller in robot_controllers.items():
            if controller.initialize():
                initialized_controllers.append(controller)
        
        # 제어 기능 테스트
        print("\n🔧 로봇 제어 기능 테스트...")
        
        if initialized_controllers:
            for controller in initialized_controllers:
                print(f"  - {controller.robot_name}: 초기화됨, DOF={controller.articulation.num_dof}")
            
            # 간단한 관절 동작 테스트
            print("\n🎯 관절 동작 테스트...")
            
            for controller in initialized_controllers:
                try:
                    # 현재 위치 저장
                    current_pos = controller.get_joint_positions()
                    if current_pos is not None:
                        print(f"    {controller.robot_name}: 현재 관절 위치 = {np.round(current_pos, 3)}")
                        
                        # 작은 움직임 테스트 (첫 번째 관절만 0.1 라디안 이동)
                        test_pos = current_pos.copy()
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
        stage_path = "/home/roarm_m3/dev_roarm/roarm_mcp/tests/robot_control_articulation_root.usd"
        stage.Export(stage_path)
        print(f"✅ USD 파일 저장 완료: {stage_path}")
        
    except Exception as e:
        print(f"❌ 테스트 실행 중 오류 발생: {e}")
        print("상세 오류 정보:")
        traceback.print_exc()
    
    finally:
        # SimulationApp 종료
        simulation_app.close()
        print("🏁 Isaac Sim 5.0 테스트 완료")

if __name__ == "__main__":
    main()