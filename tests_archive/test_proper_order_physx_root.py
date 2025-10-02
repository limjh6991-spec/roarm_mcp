#!/usr/bin/env python3
"""
====================================================================
=== Isaac Sim 5.0 올바른 순서 PhysX 루트 및 초기화 솔루션 ===
====================================================================

사용자 지적사항 모두 반영:
1. 실제 루트 경로 탐색 (/World/UR10/ur10, /World/Franka/franka)
2. World.reset() -> 뷰 생성 -> initialize 순서 준수  
3. 인스턴스 참조 비활성화 및 Rig 프림 분리
4. XformCommonAPI 경고 회피
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

print("🤖 RoArm MCP - Isaac Sim 5.0 올바른 순서 PhysX 루트 및 초기화 솔루션")
print(f"Python 버전: {sys.version}")
import os
print(f"작업 디렉토리: {os.getcwd()}")

def find_articulation_root_path(stage: Usd.Stage, under: str) -> Optional[str]:
    """
    실제 Articulation 루트 경로를 찾는 함수
    
    Args:
        stage: USD Stage
        under: 검색할 상위 경로 (예: "/World/UR10")
        
    Returns:
        실제 ArticulationRoot 경로 또는 None
    """
    print(f"\n🔍 {under} 하위에서 실제 Articulation 루트 탐색...")
    
    # 1) 기존 ArticulationRootAPI 탐색
    for prim in stage.Traverse():
        prim_path = prim.GetPath().pathString
        if prim_path.startswith(under) and UsdPhysics.ArticulationRootAPI(prim):
            print(f"  ✅ 기존 ArticulationRootAPI 발견: {prim_path}")
            return prim_path
    
    # 2) RigidBody/Joint 분포로 추정 (휴리스틱)
    candidates = {}
    for prim in stage.Traverse():
        prim_path = prim.GetPath().pathString
        if not prim_path.startswith(under):
            continue
            
        has_rb = UsdPhysics.RigidBodyAPI(prim)
        has_joint = UsdPhysics.Joint(prim)
        
        if has_rb or has_joint:
            parent = prim.GetParent()
            if parent:
                parent_path = parent.GetPath().pathString
                candidates[parent_path] = candidates.get(parent_path, 0) + 1
    
    # 가장 많이 모이는 상위 프림을 루트로 가정
    if candidates:
        best_root = max(candidates, key=candidates.get)
        print(f"  📊 RigidBody/Joint 분포 분석:")
        for path, count in sorted(candidates.items(), key=lambda x: x[1], reverse=True):
            print(f"    - {path}: {count}개")
        print(f"  ✅ 추정된 루트: {best_root}")
        return best_root
    
    print(f"  ❌ {under} 하위에서 Articulation 루트를 찾지 못했습니다")
    return None

def dump_physics_schema(stage: Usd.Stage, root: str):
    """물리 스키마 덤프 (디버깅용)"""
    print(f"\n🔧 {root} 물리 스키마 덤프:")
    
    for prim in stage.Traverse():
        prim_path = prim.GetPath().pathString
        if not prim_path.startswith(root):
            continue
            
        tags = []
        if UsdPhysics.ArticulationRootAPI(prim):
            tags.append("ARoot")
        if UsdPhysics.RigidBodyAPI(prim):
            tags.append("RB")
        if UsdPhysics.CollisionAPI(prim):
            tags.append("Col")
        if UsdPhysics.Joint(prim):
            tags.append("Joint")
            
        if tags:
            print(f"  - {prim_path} ({prim.GetTypeName()}): {tags}")

def ensure_offset_and_reference(stage: Usd.Stage, rig_path: str, ref_root_path: str, usd_path: str):
    """
    오프셋 프림(Rig)과 참조 프림을 안전하게 생성
    
    Args:
        stage: USD Stage
        rig_path: 오프셋용 Rig 프림 경로 (변환 적용용)
        ref_root_path: 로봇 참조 프림 경로
        usd_path: 참조할 USD 파일 경로
        
    Returns:
        (rig_prim, robot_prim) 튜플
    """
    print(f"🔧 오프셋 및 참조 설정: {rig_path} -> {ref_root_path}")
    
    # 오프셋 프림 (Rig) - 변환 전용
    rig_prim = stage.DefinePrim(Sdf.Path(rig_path), "Xform")
    
    # 로봇 루트 프림 (참조 담는 자리)  
    robot_prim = stage.DefinePrim(Sdf.Path(ref_root_path), "Xform")
    robot_prim.SetInstanceable(False)  # 인스턴스 비활성화
    
    # 참조 추가 (이미 있으면 중복 추가 방지)
    if not robot_prim.HasAuthoredReferences():
        robot_prim.GetReferences().AddReference(usd_path)
        print(f"  ✅ 참조 추가: {usd_path}")
    else:
        print(f"  ⚠️ 기존 참조 유지")
    
    return rig_prim, robot_prim

class ProperOrderRobotController:
    """올바른 순서의 로봇 제어기"""
    
    def __init__(self, robot_name: str, rig_path: str, robot_path: str):
        self.robot_name = robot_name
        self.rig_path = rig_path
        self.robot_path = robot_path
        self.actual_root_path = None
        self.articulation = None
        self.initialized = False
        
        print(f"\n🎮 {self.robot_name} 올바른 순서 제어기 생성...")
        print(f"  Rig: {self.rig_path}")
        print(f"  Robot: {self.robot_path}")
    
    def find_and_apply_articulation_root(self, stage: Usd.Stage) -> bool:
        """실제 루트 경로를 찾고 ArticulationRootAPI 적용"""
        
        # 실제 루트 경로 탐색
        self.actual_root_path = find_articulation_root_path(stage, self.robot_path)
        
        if not self.actual_root_path:
            print(f"    ❌ {self.robot_name}: 실제 루트 경로를 찾지 못했습니다")
            return False
        
        print(f"    ✅ {self.robot_name}: 실제 루트 경로 = {self.actual_root_path}")
        
        # ArticulationRootAPI 적용 (없는 경우에만)
        root_prim = stage.GetPrimAtPath(self.actual_root_path)
        if not root_prim or not root_prim.IsValid():
            print(f"    ❌ {self.robot_name}: 유효하지 않은 루트 프림")
            return False
        
        if not UsdPhysics.ArticulationRootAPI(root_prim):
            articulation_root_api = UsdPhysics.ArticulationRootAPI.Apply(root_prim)
            if articulation_root_api:
                print(f"    ✅ {self.robot_name}: ArticulationRootAPI 적용 성공")
            else:
                print(f"    ❌ {self.robot_name}: ArticulationRootAPI 적용 실패")
                return False
        else:
            print(f"    ℹ️ {self.robot_name}: 기존 ArticulationRootAPI 사용")
        
        return True
    
    def create_articulation_after_reset(self) -> bool:
        """World 리셋 후 Articulation 인스턴스 생성"""
        if not self.actual_root_path:
            print(f"    ❌ {self.robot_name}: 실제 루트 경로가 없습니다")
            return False
        
        try:
            print(f"    🔧 {self.robot_name}: Articulation 인스턴스 생성... (경로: {self.actual_root_path})")
            self.articulation = Articulation(prim_path=self.actual_root_path, name=f"{self.robot_name.lower()}_articulation")
            
            if self.articulation:
                print(f"    ✅ {self.robot_name}: Articulation 인스턴스 생성 성공")
                return True
            else:
                print(f"    ❌ {self.robot_name}: Articulation 인스턴스 생성 실패")
                return False
                
        except Exception as e:
            print(f"    ❌ {self.robot_name}: Articulation 생성 중 오류 - {e}")
            return False
    
    def initialize_articulation(self) -> bool:
        """Articulation 초기화"""
        if not self.articulation:
            print(f"    ❌ {self.robot_name}: Articulation 인스턴스가 없습니다")
            return False
        
        try:
            print(f"    🔧 {self.robot_name}: Articulation 초기화...")
            self.articulation.initialize()
            
            # 초기화 검증
            dof_count = self.articulation.num_dof
            print(f"      {self.robot_name}: DOF 개수 = {dof_count}")
            
            if dof_count is None or dof_count <= 0:
                print(f"    ❌ {self.robot_name}: 유효하지 않은 DOF 개수")
                return False
            
            # 관절 이름 확인
            joint_names = self.articulation.dof_names
            print(f"      {self.robot_name}: 관절 이름들 = {joint_names}")
            
            if not joint_names or len(joint_names) == 0:
                print(f"    ❌ {self.robot_name}: 관절 이름이 없습니다")
                return False
            
            # 관절 상태 확인
            joint_positions = self.articulation.get_joint_positions()
            if joint_positions is not None:
                print(f"      {self.robot_name}: 현재 관절 위치 = {np.round(joint_positions, 3)}")
                self.initialized = True
                print(f"    ✅ {self.robot_name}: Articulation 초기화 성공")
                return True
            else:
                print(f"    ❌ {self.robot_name}: 관절 위치를 읽을 수 없습니다")
                return False
                
        except Exception as e:
            print(f"    ❌ {self.robot_name}: Articulation 초기화 실패 - {e}")
            return False
    
    def get_joint_positions(self) -> Optional[np.ndarray]:
        """현재 관절 위치 가져오기"""
        if not self.initialized or not self.articulation:
            return None
        try:
            return self.articulation.get_joint_positions()
        except Exception as e:
            print(f"      {self.robot_name}: 관절 위치 확인 실패 - {e}")
            return None
    
    def set_joint_positions(self, positions: np.ndarray) -> bool:
        """관절 위치 설정"""
        if not self.initialized or not self.articulation:
            return False
        try:
            self.articulation.set_joint_positions(positions)
            return True
        except Exception as e:
            print(f"      {self.robot_name}: 관절 위치 설정 실패 - {e}")
            return False

def setup_physics_scene(stage: Usd.Stage) -> bool:
    """Physics Scene 설정"""
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
    
    robot_controllers: Dict[str, ProperOrderRobotController] = {}
    
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
        
        # Isaac Sim World 초기화 (리셋 전에)
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
        
        print("\n🤖 로봇 모델 로딩 (올바른 구조)...")
        
        # UR10 로봇 오프셋 및 참조 설정
        ur10_rig, ur10_robot = ensure_offset_and_reference(
            stage, 
            "/World/UR10_Rig", 
            "/World/UR10", 
            "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
        )
        
        # UR10 위치 설정 (Rig 프림에만)
        ur10_rig_xform = UsdGeom.Xformable(ur10_rig)
        UsdGeom.XformCommonAPI(ur10_rig_xform).SetTranslate(Gf.Vec3d(0.0, 0.0, 0.0))
        print("  - UR10 로봇 로딩 및 위치 설정 완료")
        
        # Franka 로봇 오프셋 및 참조 설정
        franka_rig, franka_robot = ensure_offset_and_reference(
            stage,
            "/World/Franka_Rig",
            "/World/Franka",
            "/Isaac/Robots/Franka/franka.usd"
        )
        
        # Franka 위치 설정 (Rig 프림에만)
        franka_rig_xform = UsdGeom.Xformable(franka_rig)
        UsdGeom.XformCommonAPI(franka_rig_xform).SetTranslate(Gf.Vec3d(2.0, 0.0, 0.0))
        print("  - Franka 로봇 로딩 및 위치 설정 완료")
        
        # USD 레퍼런스 로딩 대기
        print("\n⏳ USD 레퍼런스 로딩 대기 중...")
        for i in range(10):
            print(f"  대기 중... {i+1}/10초")
            omni.kit.app.get_app().update()
            time.sleep(1.0)
        
        # 물리 스키마 분석 (디버깅)
        dump_physics_schema(stage, "/World/UR10")
        dump_physics_schema(stage, "/World/Franka")
        
        # 로봇 제어기 생성
        print("\n🎮 올바른 순서 로봇 제어 인터페이스 생성...")
        
        ur10_controller = ProperOrderRobotController("UR10", "/World/UR10_Rig", "/World/UR10")
        franka_controller = ProperOrderRobotController("Franka", "/World/Franka_Rig", "/World/Franka")
        
        robot_controllers["UR10"] = ur10_controller
        robot_controllers["Franka"] = franka_controller
        
        # 1단계: 실제 루트 경로 탐색 및 ArticulationRootAPI 적용
        print("\n🔧 1단계: 실제 루트 경로 탐색 및 ArticulationRootAPI 적용...")
        
        prepared_controllers = []
        for robot_name, controller in robot_controllers.items():
            if controller.find_and_apply_articulation_root(stage):
                prepared_controllers.append(controller)
            else:
                print(f"    ❌ {robot_name}: 루트 경로 탐색 또는 API 적용 실패")
        
        # 2단계: World 리셋 (중요!)
        print(f"\n🔄 2단계: World 리셋 (준비된 로봇: {len(prepared_controllers)}개)...")
        
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
        
        # 리셋 후 프레임 처리
        for _ in range(3):
            omni.kit.app.get_app().update()
            time.sleep(0.02)
        
        # 3단계: 리셋 후 Articulation 인스턴스 생성
        print(f"\n🔧 3단계: 리셋 후 Articulation 인스턴스 생성...")
        
        created_controllers = []
        for controller in prepared_controllers:
            if controller.create_articulation_after_reset():
                created_controllers.append(controller)
        
        # 4단계: 업데이트 및 초기화
        print(f"\n🔧 4단계: 업데이트 및 초기화 (생성된 Articulation: {len(created_controllers)}개)...")
        
        # 한 프레임 업데이트
        omni.kit.app.get_app().update()
        
        initialized_controllers = []
        for controller in created_controllers:
            if controller.initialize_articulation():
                initialized_controllers.append(controller)
        
        # 5단계: 제어 기능 테스트
        print(f"\n🎯 5단계: 로봇 제어 기능 테스트 (초기화된 로봇: {len(initialized_controllers)}개)...")
        
        if initialized_controllers:
            for controller in initialized_controllers:
                print(f"  ✅ {controller.robot_name}: 경로={controller.actual_root_path}, DOF={controller.articulation.num_dof}")
            
            # 간단한 관절 동작 테스트
            print("\n🎯 관절 동작 테스트...")
            
            for controller in initialized_controllers:
                try:
                    current_pos = controller.get_joint_positions()
                    if current_pos is not None and len(current_pos) > 0:
                        print(f"    {controller.robot_name}: 현재 관절 위치 = {np.round(current_pos, 3)}")
                        
                        # 작은 움직임 테스트
                        test_pos = current_pos.copy()
                        test_pos[0] += 0.1
                        
                        print(f"    {controller.robot_name}: 테스트 위치로 이동 = {np.round(test_pos, 3)}")
                        if controller.set_joint_positions(test_pos):
                            print(f"    ✅ {controller.robot_name}: 관절 위치 설정 성공")
                            
                            # 물리 시뮬레이션 스텝 실행
                            for _ in range(10):
                                world.step(render=False)
                            
                            # 변경된 위치 확인
                            new_pos = controller.get_joint_positions()
                            if new_pos is not None:
                                print(f"    {controller.robot_name}: 변경 후 위치 = {np.round(new_pos, 3)}")
                        else:
                            print(f"    ❌ {controller.robot_name}: 관절 위치 설정 실패")
                    
                except Exception as e:
                    print(f"    ❌ {controller.robot_name}: 동작 테스트 실패 - {e}")
        else:
            print("  ⚠️ 초기화된 로봇이 없어 제어 테스트를 건너뜁니다.")
        
        # USD 파일 저장
        stage_path = "/home/roarm_m3/dev_roarm/roarm_mcp/tests/robot_control_proper_order.usd"
        stage.Export(stage_path)
        print(f"✅ USD 파일 저장 완료: {stage_path}")
        
        # 최종 결과 요약
        print(f"\n📊 최종 결과 요약:")
        print(f"  - 생성된 제어기: {len(robot_controllers)}개")
        print(f"  - 루트 경로 탐색 성공: {len(prepared_controllers)}개")
        print(f"  - Articulation 생성 성공: {len(created_controllers)}개")
        print(f"  - 최종 초기화 성공: {len(initialized_controllers)}개")
        
        if initialized_controllers:
            print(f"  ✅ 성공한 로봇들:")
            for controller in initialized_controllers:
                print(f"    - {controller.robot_name}: {controller.actual_root_path} (DOF: {controller.articulation.num_dof})")
        
        return len(initialized_controllers) > 0
        
    except Exception as e:
        print(f"❌ 테스트 실행 중 오류 발생: {e}")
        print("상세 오류 정보:")
        traceback.print_exc()
        return False
    
    finally:
        # SimulationApp 종료
        simulation_app.close()
        print("🏁 Isaac Sim 5.0 올바른 순서 PhysX 루트 및 초기화 테스트 완료")

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)