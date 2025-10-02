#!/usr/bin/env python3
"""
====================================================================
=== Isaac Sim 5.0 로봇 USD 구조 분석 및 직접 경로 제어 테스트 ===
====================================================================

ArticulationRoot 자동 탐색 실패시 로봇의 실제 USD 구조를 분석하고
직접 관절 경로를 찾아서 SingleArticulation을 생성하는 접근
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

print("🤖 RoArm MCP - Isaac Sim 5.0 로봇 USD 구조 분석 및 직접 경로 제어 테스트")
print(f"Python 버전: {sys.version}")
import os
print(f"작업 디렉토리: {os.getcwd()}")

class RobotStructureAnalyzer:
    """로봇 USD 구조 분석 및 관절 경로 탐색 유틸리티"""
    
    @staticmethod
    def analyze_robot_structure(stage: Usd.Stage, robot_path: str) -> Dict:
        """
        로봇 프림 구조를 분석하여 관절 정보를 수집합니다.
        
        Args:
            stage: USD Stage
            robot_path: 로봇 프림 경로
            
        Returns:
            로봇 구조 정보 딕셔너리
        """
        print(f"\n🔍 {robot_path} 로봇 구조 분석 시작...")
        
        robot_prim = stage.GetPrimAtPath(robot_path)
        if not robot_prim:
            print(f"  ❌ 로봇 프림을 찾을 수 없습니다: {robot_path}")
            return {}
        
        structure = {
            "robot_path": robot_path,
            "robot_type": robot_prim.GetTypeName(),
            "children": [],
            "joints": [],
            "links": [],
            "potential_roots": []
        }
        
        print(f"  📊 루트 프림: {robot_path} (타입: {robot_prim.GetTypeName()})")
        
        # 하위 프림들을 재귀적으로 분석
        def analyze_prim(prim, depth=0):
            indent = "  " * (depth + 1)
            prim_path = prim.GetPath().pathString
            prim_type = prim.GetTypeName()
            
            print(f"{indent}├─ {prim_path.split('/')[-1]} ({prim_type})")
            
            # 프림 타입별 분류
            if prim_type in ["Joint", "RevoluteJoint", "PrismaticJoint"]:
                structure["joints"].append(prim_path)
                print(f"{indent}   🔗 관절 발견: {prim_path}")
            elif prim_type in ["Xform", "Mesh", "Scope"]:
                if "link" in prim_path.lower() or "base" in prim_path.lower():
                    structure["links"].append(prim_path)
                    print(f"{indent}   🔧 링크 발견: {prim_path}")
            
            # ArticulationRootAPI 확인
            try:
                if UsdPhysics.ArticulationRootAPI(prim):
                    structure["potential_roots"].append(prim_path)
                    print(f"{indent}   🎯 ArticulationRoot API 발견: {prim_path}")
            except:
                pass
            
            # 하위 프림들 재귀 분석
            for child in prim.GetChildren():
                if depth < 4:  # 깊이 제한
                    analyze_prim(child, depth + 1)
                    
        # 분석 시작
        analyze_prim(robot_prim)
        
        print(f"\n📈 분석 결과:")
        print(f"  - 관절 개수: {len(structure['joints'])}")
        print(f"  - 링크 개수: {len(structure['links'])}")
        print(f"  - ArticulationRoot 후보: {len(structure['potential_roots'])}")
        
        if structure['potential_roots']:
            print(f"  ✅ ArticulationRoot 후보들:")
            for root in structure['potential_roots']:
                print(f"    - {root}")
        else:
            print(f"  ⚠️ ArticulationRoot를 찾지 못했습니다")
            
        if structure['joints']:
            print(f"  🔗 발견된 관절들:")
            for joint in structure['joints'][:5]:  # 처음 5개만 표시
                print(f"    - {joint}")
            if len(structure['joints']) > 5:
                print(f"    ... 및 {len(structure['joints'])-5}개 더")
                
        return structure
    
    @staticmethod
    def find_articulation_root_candidates(stage: Usd.Stage, robot_path: str) -> List[str]:
        """
        로봇에서 가능한 articulation root 후보들을 찾습니다.
        """
        candidates = []
        
        # 1. 직접적인 ArticulationRootAPI 확인
        robot_prim = stage.GetPrimAtPath(robot_path)
        if robot_prim and UsdPhysics.ArticulationRootAPI(robot_prim):
            candidates.append(robot_path)
            
        # 2. 하위 프림에서 ArticulationRootAPI 탐색
        for prim in stage.Traverse():
            prim_path = prim.GetPath().pathString
            if prim_path.startswith(robot_path) and UsdPhysics.ArticulationRootAPI(prim):
                candidates.append(prim_path)
        
        # 3. 관습적인 경로들 확인
        common_patterns = [
            f"{robot_path}/base_link",
            f"{robot_path}/robot_base", 
            f"{robot_path}/ur10",  # UR10 specific
            f"{robot_path}/franka",  # Franka specific
            f"{robot_path}/panda",  # Franka Panda specific
        ]
        
        for pattern in common_patterns:
            prim = stage.GetPrimAtPath(pattern)
            if prim and prim.IsValid():
                candidates.append(pattern)
                
        return list(set(candidates))  # 중복 제거

class DirectPathRobotController:
    """직접 경로 기반 로봇 제어기"""
    
    def __init__(self, robot_name: str, candidate_paths: List[str]):
        self.robot_name = robot_name
        self.candidate_paths = candidate_paths
        self.articulation = None
        self.successful_path = None
        self.initialized = False
        
        print(f"\n🎮 {self.robot_name} 제어기 생성 중...")
        print(f"  후보 경로들: {candidate_paths}")
        
        # 후보 경로들을 하나씩 시도
        for path in candidate_paths:
            try:
                print(f"    🔄 {path} 경로로 SingleArticulation 시도...")
                self.articulation = SingleArticulation(prim_path=path)
                self.successful_path = path
                print(f"    ✅ {path}에서 SingleArticulation 생성 성공")
                break
            except Exception as e:
                print(f"    ❌ {path}에서 실패: {str(e)}")
                continue
                
        if not self.articulation:
            print(f"    💥 모든 후보 경로에서 SingleArticulation 생성 실패")
    
    def initialize(self) -> bool:
        """로봇 제어기 초기화"""
        if not self.articulation:
            print(f"    ❌ {self.robot_name}: SingleArticulation이 없습니다")
            return False
            
        try:
            print(f"    🔧 {self.robot_name} 제어기 초기화 중... (경로: {self.successful_path})")
            
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
    
    robot_controllers: Dict[str, DirectPathRobotController] = {}
    
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
        
        # 로봇 구조 분석
        print("\n🔍 로봇 구조 분석...")
        
        ur10_structure = RobotStructureAnalyzer.analyze_robot_structure(stage, "/World/UR10")
        franka_structure = RobotStructureAnalyzer.analyze_robot_structure(stage, "/World/Franka")
        
        # 후보 경로들 수집
        print("\n📍 Articulation Root 후보 경로 수집...")
        
        ur10_candidates = RobotStructureAnalyzer.find_articulation_root_candidates(stage, "/World/UR10")
        franka_candidates = RobotStructureAnalyzer.find_articulation_root_candidates(stage, "/World/Franka")
        
        print(f"  - UR10 후보들: {ur10_candidates}")
        print(f"  - Franka 후보들: {franka_candidates}")
        
        # 로봇 제어기 생성
        print("\n🎮 로봇 제어 인터페이스 생성 (직접 경로 기반)...")
        
        if ur10_candidates:
            ur10_controller = DirectPathRobotController("UR10", ur10_candidates)
            robot_controllers["UR10"] = ur10_controller
        else:
            print("  ⚠️ UR10 후보 경로를 찾지 못했습니다")
            
        if franka_candidates:
            franka_controller = DirectPathRobotController("Franka", franka_candidates)
            robot_controllers["Franka"] = franka_controller
        else:
            print("  ⚠️ Franka 후보 경로를 찾지 못했습니다")
        
        # World 리셋
        print("\n🔄 World 리셋 (직접 경로 기반)...")
        
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
        print(f"\n🔧 로봇 제어 기능 테스트 (초기화된 로봇: {len(initialized_controllers)}개)...")
        
        if initialized_controllers:
            for controller in initialized_controllers:
                print(f"  ✅ {controller.robot_name}: 경로={controller.successful_path}, DOF={controller.articulation.num_dof}")
            
            # 간단한 관절 동작 테스트
            print("\n🎯 관절 동작 테스트...")
            
            for controller in initialized_controllers:
                try:
                    current_pos = controller.get_joint_positions()
                    if current_pos is not None:
                        print(f"    {controller.robot_name}: 현재 관절 위치 = {np.round(current_pos, 3)}")
                        
                        # 작은 움직임 테스트
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
        stage_path = "/home/roarm_m3/dev_roarm/roarm_mcp/tests/robot_control_direct_path.usd"
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