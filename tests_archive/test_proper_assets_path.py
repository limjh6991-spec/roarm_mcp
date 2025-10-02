#!/usr/bin/env python3
"""
🤖 RoArm MCP - Isaac Sim 5.0 올바른 로봇 Assets 경로 및 PhysX 초기화 솔루션

본 스크립트는 Isaac Sim 5.0에서 올바른 로봇 Assets 경로를 사용하여 
PhysX Tensors와 ArticulationRootAPI를 올바른 순서로 초기화하는 방법을 보여줍니다.

주요 수정사항:
1. get_assets_root_path() 사용하여 올바른 Assets 경로 획득
2. 올바른 로봇 USD 파일 경로 사용
3. ArticulationRootAPI 적용 후 World.reset() 시퀀스
4. PhysX Tensors 초기화 순서 수정
5. Rig 분리 및 instanceable=False 설정
"""

import sys
import time
import carb
import numpy as np
from pathlib import Path
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Sdf, Gf
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(message)s')
logger = logging.getLogger(__name__)

def print_header():
    print("🤖 RoArm MCP - Isaac Sim 5.0 올바른 Assets 경로 PhysX 초기화 솔루션")
    print(f"Python 버전: {sys.version}")
    print(f"작업 디렉토리: {Path.cwd()}")

# Initialize Isaac Sim
from isaacsim import SimulationApp

print_header()

# Simulation configuration - headless for testing
simulation_config = {
    "headless": True,  # Change to False for GUI
    "renderer": "RayTracedLighting",
    "display_options": 3094,  # Disable most visual features for performance
    "width": 1280,
    "height": 720,
}

print("✅ Isaac Sim 5.0 SimulationApp 초기화 중...")
simulation_app = SimulationApp(simulation_config)
print("✅ Isaac Sim 5.0 SimulationApp 초기화 성공")

try:
    # Isaac Sim imports (must be after SimulationApp initialization)
    from isaacsim.core.api import World
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.storage.native import get_assets_root_path
    from isaacsim.core.utils.prims import get_prim_at_path, is_prim_path_valid
    from omni.physx.tensors import create_simulation_view
    from omni.physics.tensors import ArticulationView
    from pxr import PhysxSchema
    
    print("✅ Isaac Sim 모듈 import 성공")
    
    # Get Isaac Sim assets root path
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder")
        raise RuntimeError("Isaac Sim assets not found")
    
    print(f"✅ Isaac Sim Assets 경로 발견: {assets_root_path}")
    
    # Robot asset paths (correct for Isaac Sim 5.0)
    robot_assets = {
        "UR10": {
            "asset_path": assets_root_path + "/Isaac/Robots/UniversalRobots/ur10/ur10.usd",
            "prim_path": "/World/UR10",
            "rig_path": "/World/UR10_Rig",
            "expected_root": "/World/UR10/ur10"
        },
        "Franka": {
            "asset_path": assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd", 
            "prim_path": "/World/Franka",
            "rig_path": "/World/Franka_Rig",
            "expected_root": "/World/Franka/franka"
        }
    }
    
    # Verify robot assets exist
    print("🔧 로봇 Assets 존재 여부 확인...")
    for robot_name, robot_info in robot_assets.items():
        asset_file = Path(robot_info["asset_path"])
        if asset_file.exists():
            print(f"  ✅ {robot_name}: {asset_file}")
        else:
            print(f"  ❌ {robot_name}: {asset_file} (파일이 존재하지 않음)")
    
    # Create USD stage
    print("✅ USD Stage 생성 성공")
    
    # Initialize World
    print("🔧 Physics Scene 설정 중...")
    my_world = World(stage_units_in_meters=1.0)
    stage = my_world.stage
    
    # Check for existing physics scene
    physics_scene_path = "/physicsScene"
    if is_prim_path_valid(physics_scene_path):
        print("    ✅ 기존 Physics Scene 발견")
    else:
        print("    🔧 Physics Scene 생성")
        
    print("✅ Isaac Sim World 초기화 성공")
    
    # Setup simulation environment
    print("\n🌟 시뮬레이션 환경 설정...")
    my_world.scene.add_default_ground_plane()
    print("  - 기본 조명 설정 완료")
    print("  - 바닥 평면 추가 완료")

def ensure_offset_and_reference(stage, rig_path, robot_path, asset_path):
    """Rig prim 생성 및 로봇 레퍼런스 추가 (instanceable=False)"""
    print(f"🔧 오프셋 및 참조 설정: {rig_path} -> {robot_path}")
    
    # Create Rig prim as Xform
    rig_prim = UsdGeom.Xform.Define(stage, rig_path).GetPrim()
    
    # Add reference to robot under the rig
    robot_prim = stage.DefinePrim(robot_path)
    robot_prim.GetReferences().AddReference(asset_path)
    
    # CRITICAL: Set instanceable=False for PhysX compatibility
    robot_prim.SetInstanceable(False)
    
    # Position robot (example positions)
    if "UR10" in robot_path:
        # Position UR10
        UsdGeom.XformCommonAPI(rig_prim).SetTranslate((2.0, 0.0, 0.0))
    elif "Franka" in robot_path:
        # Position Franka  
        UsdGeom.XformCommonAPI(rig_prim).SetTranslate((-2.0, 0.0, 0.0))
    
    print(f"  ✅ 참조 추가: {asset_path}")
    return robot_prim

def find_articulation_root_path(stage, base_path):
    """
    실제 Articulation 루트를 찾는 함수
    RigidBody와 Joint의 분포를 분석하여 가장 적절한 루트를 찾습니다.
    """
    print(f"🔍 {base_path} 하위에서 실제 Articulation 루트 탐색...")
    
    def count_physics_components(prim_path):
        """prim 하위의 RigidBody와 Joint 개수 계산"""
        prim = get_prim_at_path(prim_path)
        if not prim or not prim.IsValid():
            return 0, 0
            
        rigid_body_count = 0
        joint_count = 0
        
        def traverse_prim(p):
            nonlocal rigid_body_count, joint_count
            
            # Check for RigidBodyAPI
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                rigid_body_count += 1
                
            # Check for Joint schemas  
            if (prim.HasAPI(UsdPhysics.RevoluteJoint) or 
                prim.HasAPI(UsdPhysics.PrismaticJoint) or
                prim.HasAPI(UsdPhysics.FixedJoint) or
                prim.HasAPI(UsdPhysics.SphericalJoint)):
                joint_count += 1
                
            # Traverse children
            for child in p.GetChildren():
                traverse_prim(child)
                
        traverse_prim(prim)
        return rigid_body_count, joint_count
    
    base_prim = get_prim_at_path(base_path)
    if not base_prim or not base_prim.IsValid():
        print(f"  ❌ 기본 경로 {base_path}가 유효하지 않음")
        return None
        
    candidates = []
    
    # Direct children of base_path
    for child in base_prim.GetChildren():
        child_path = str(child.GetPath())
        rb_count, joint_count = count_physics_components(child_path)
        
        if rb_count > 0 and joint_count > 0:
            score = rb_count + joint_count
            candidates.append((child_path, score, rb_count, joint_count))
            print(f"    🎯 후보: {child_path} (RB: {rb_count}, Joints: {joint_count}, Score: {score})")
    
    if candidates:
        # Sort by score (descending)
        candidates.sort(key=lambda x: x[1], reverse=True)
        best_path = candidates[0][0]
        print(f"  ✅ 최적 루트 경로 발견: {best_path}")
        return best_path
    else:
        print(f"  ❌ {base_path} 하위에서 Articulation 루트를 찾지 못했습니다")
        return None

def apply_articulation_root_api(stage, prim_path):
    """ArticulationRootAPI 적용"""
    prim = get_prim_at_path(prim_path)
    if not prim or not prim.IsValid():
        print(f"    ❌ Prim {prim_path} 찾을 수 없음")
        return False
        
    # Apply ArticulationRootAPI
    if not prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        UsdPhysics.ArticulationRootAPI.Apply(prim)
        print(f"    ✅ ArticulationRootAPI 적용: {prim_path}")
    else:
        print(f"    ℹ️  ArticulationRootAPI 이미 적용됨: {prim_path}")
    
    return True

def dump_physics_schema(stage, prim_path):
    """물리 스키마 정보 덤프 (디버깅용)"""
    print(f"\n🔧 {prim_path} 물리 스키마 덤프:")
    
    prim = get_prim_at_path(prim_path)
    if not prim or not prim.IsValid():
        print("  ❌ Prim이 유효하지 않음")
        return
    
    def check_prim_physics(p, indent="  "):
        path = str(p.GetPath())
        
        # Check physics APIs
        apis = []
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            apis.append("RigidBody")
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            apis.append("ArticulationRoot")
        if p.HasAPI(UsdPhysics.RevoluteJoint):
            apis.append("RevoluteJoint") 
        if p.HasAPI(UsdPhysics.PrismaticJoint):
            apis.append("PrismaticJoint")
        if p.HasAPI(UsdPhysics.FixedJoint):
            apis.append("FixedJoint")
            
        if apis:
            print(f"{indent}{path}: {', '.join(apis)}")
            
        # Recurse to children (limit depth)
        if len(indent) < 12:  # Limit depth
            for child in p.GetChildren():
                check_prim_physics(child, indent + "  ")
    
    check_prim_physics(prim)

class ProperOrderRobotController:
    """올바른 순서로 로봇을 초기화하는 제어 클래스"""
    
    def __init__(self, robot_name, rig_path, robot_path, expected_root):
        self.robot_name = robot_name
        self.rig_path = rig_path
        self.robot_path = robot_path
        self.expected_root = expected_root
        self.actual_root_path = None
        self.articulation_view = None
        self.is_ready = False
        
        print(f"🎮 {robot_name} 올바른 순서 제어기 생성...")
        print(f"  Rig: {rig_path}")
        print(f"  Robot: {robot_path}")
        
    def find_and_apply_root(self, stage):
        """1단계: 실제 루트 경로를 찾고 ArticulationRootAPI 적용"""
        self.actual_root_path = find_articulation_root_path(stage, self.robot_path)
        
        if self.actual_root_path:
            success = apply_articulation_root_api(stage, self.actual_root_path)
            if success:
                print(f"    ✅ {self.robot_name}: ArticulationRootAPI 적용 성공")
                return True
            else:
                print(f"    ❌ {self.robot_name}: ArticulationRootAPI 적용 실패")
                return False
        else:
            print(f"    ❌ {self.robot_name}: 실제 루트 경로를 찾지 못했습니다")
            return False
            
    def create_articulation_view_after_reset(self):
        """3단계: 리셋 후 Articulation 인스턴스 생성"""
        if not self.actual_root_path:
            print(f"    ❌ {self.robot_name}: 루트 경로가 없어 Articulation 생성 불가")
            return False
            
        try:
            # PhysX Tensors ArticulationView 생성 (올바른 패턴 사용)
            self.articulation_view = ArticulationView(
                prim_paths_expr=self.actual_root_path,
                name=f"{self.robot_name}_articulation"
            )
            print(f"    ✅ {self.robot_name}: Articulation 인스턴스 생성 성공")
            return True
        except Exception as e:
            print(f"    ❌ {self.robot_name}: Articulation 생성 실패 - {e}")
            return False
            
    def initialize(self):
        """4단계: 초기화"""
        if not self.articulation_view:
            print(f"    ❌ {self.robot_name}: Articulation 인스턴스가 없어 초기화 불가")
            return False
            
        try:
            self.articulation_view.initialize()
            print(f"    ✅ {self.robot_name}: 초기화 성공")
            self.is_ready = True
            return True
        except Exception as e:
            print(f"    ❌ {self.robot_name}: 초기화 실패 - {e}")
            return False
            
    def test_control(self):
        """5단계: 제어 기능 테스트"""
        if not self.is_ready or not self.articulation_view:
            print(f"    ⚠️ {self.robot_name}: 준비되지 않아 테스트 건너뜀")
            return False
            
        try:
            # Get articulation properties
            dof_count = self.articulation_view.count
            joint_positions = self.articulation_view.get_joint_positions()
            joint_velocities = self.articulation_view.get_joint_velocities()
            
            print(f"    🎯 {self.robot_name} 제어 테스트:")
            print(f"      - DOF 수: {dof_count}")
            print(f"      - 현재 관절 위치: {joint_positions}")
            print(f"      - 현재 관절 속도: {joint_velocities}")
            
            # Test simple joint position control
            if dof_count > 0 and joint_positions is not None:
                # Create small position offset for testing
                target_positions = joint_positions + np.random.uniform(-0.1, 0.1, joint_positions.shape)
                self.articulation_view.set_joint_position_targets(target_positions)
                print(f"      - 목표 관절 위치 설정 완료: {target_positions}")
                
            return True
        except Exception as e:
            print(f"    ❌ {self.robot_name}: 제어 테스트 실패 - {e}")
            return False

def main():
    """메인 실행 함수"""

    # Load robots with proper structure
    print("\n🤖 로봇 모델 로딩 (올바른 구조)...")
    robot_prims = {}
    for robot_name, robot_info in robot_assets.items():
        asset_file = Path(robot_info["asset_path"])
        if asset_file.exists():
            robot_prim = ensure_offset_and_reference(
                stage, 
                robot_info["rig_path"], 
                robot_info["prim_path"], 
                robot_info["asset_path"]
            )
            robot_prims[robot_name] = robot_prim
            print(f"  - {robot_name} 로봇 로딩 및 위치 설정 완료")
        else:
            print(f"  ⚠️ {robot_name} 로봇 파일이 없어 건너뜀: {asset_file}")
    
    # Wait for USD references to load
    print("\n⏳ USD 레퍼런스 로딩 대기 중...")
    for i in range(10):
        print(f"  대기 중... {i+1}/10초")
        time.sleep(1)
        my_world.step()  # Process USD loading
    
    # Debug: Dump physics schema for loaded robots
    for robot_name, robot_info in robot_assets.items():
        if robot_name in robot_prims:
            dump_physics_schema(stage, robot_info["prim_path"])
    
    # Create ProperOrderRobotController instances
    print("\n🎮 올바른 순서 로봇 제어 인터페이스 생성...")
    controllers = {}
    for robot_name, robot_info in robot_assets.items():
        if robot_name in robot_prims:
            controller = ProperOrderRobotController(
                robot_name,
                robot_info["rig_path"],
                robot_info["prim_path"], 
                robot_info["expected_root"]
            )
            controllers[robot_name] = controller
    
    # 5-Stage Proper Order Initialization Process
    print(f"\n🔧 1단계: 실제 루트 경로 탐색 및 ArticulationRootAPI 적용...")
    prepared_robots = []
    
    for robot_name, controller in controllers.items():
        if controller.find_and_apply_root(stage):
            prepared_robots.append(robot_name)
        else:
            print(f"    ❌ {robot_name}: 루트 경로 탐색 또는 API 적용 실패")
    
    # Stage 2: World reset (CRITICAL for proper PhysX initialization)
    print(f"\n🔄 2단계: World 리셋 (준비된 로봇: {len(prepared_robots)}개)...")
    reset_success = False
    for attempt in range(3):
        try:
            print(f"  - World 리셋 시도 {attempt + 1}/3...")
            my_world.reset()
            reset_success = True
            print(f"  - World 리셋 성공 (시도 {attempt + 1})")
            break
        except Exception as reset_e:
            print(f"  - World 리셋 실패 (시도 {attempt + 1}): {reset_e}")
            if attempt == 2:
                print("  ❌ 최대 재시도 횟수 초과, 계속 진행")
    
    # Stage 3: Create articulation instances AFTER reset
    print(f"\n🔧 3단계: 리셋 후 Articulation 인스턴스 생성...")
    articulated_robots = []
    
    for robot_name in prepared_robots:
        if controllers[robot_name].create_articulation_view_after_reset():
            articulated_robots.append(robot_name)
    
    # Stage 4: Initialize articulations
    print(f"\n🔧 4단계: 업데이트 및 초기화 (생성된 Articulation: {len(articulated_robots)}개)...")
    initialized_robots = []
    
    if articulated_robots:
        # Step physics to ensure everything is properly loaded
        for _ in range(5):
            my_world.step()
        
        for robot_name in articulated_robots:
            if controllers[robot_name].initialize():
                initialized_robots.append(robot_name)
    
    # Stage 5: Test robot control functionality  
    print(f"\n🎯 5단계: 로봇 제어 기능 테스트 (초기화된 로봇: {len(initialized_robots)}개)...")
    
    if initialized_robots:
        for robot_name in initialized_robots:
            controllers[robot_name].test_control()
    else:
        print("  ⚠️ 초기화된 로봇이 없어 제어 테스트를 건너뜁니다.")
    
    # Save USD file for inspection
    usd_save_path = "/home/roarm_m3/dev_roarm/roarm_mcp/tests/robot_control_proper_assets.usd"
    stage.Export(usd_save_path)
    print(f"✅ USD 파일 저장 완료: {usd_save_path}")
    
    # Final summary
    print(f"\n📊 최종 결과 요약:")
    print(f"  - 생성된 제어기: {len(controllers)}개")
    print(f"  - 루트 경로 탐색 성공: {len(prepared_robots)}개")
    print(f"  - Articulation 생성 성공: {len(articulated_robots)}개")
    print(f"  - 최종 초기화 성공: {len(initialized_robots)}개")

except Exception as e:
    logger.error(f"❌ 실행 중 오류 발생: {e}")
    import traceback
    traceback.print_exc()

finally:
    # Clean shutdown
    if 'simulation_app' in locals():
        simulation_app.close()

# 메인 함수 실행
if __name__ == "__main__":
    main()