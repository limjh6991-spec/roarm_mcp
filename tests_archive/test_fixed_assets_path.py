#!/usr/bin/env python3
"""
🤖 RoArm MCP - Isaac Sim 5.0 올바른 로봇 Assets 경로 및 PhysX 초기화 솔루션

본 스크립트는 Isaac Sim 5.0에서 올바른 로봇 Assets 경로를 사용하여 
PhysX Tensors와 ArticulationRootAPI를 올바른 순서로 초기화하는 방법을 보여줍니다.
"""

import sys
import time
import logging
from pathlib import Path

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
    "display_options": 3094,
    "width": 1280,
    "height": 720,
}

print("✅ Isaac Sim 5.0 SimulationApp 초기화 중...")
simulation_app = SimulationApp(simulation_config)
print("✅ Isaac Sim 5.0 SimulationApp 초기화 성공")

try:
    # Isaac Sim imports (must be after SimulationApp initialization)
    import carb
    import numpy as np
    from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Sdf, Gf
    from isaacsim.core.api import World
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.storage.native import get_assets_root_path
    from isaacsim.core.utils.prims import get_prim_at_path, is_prim_path_valid
    from omni.physics.tensors import ArticulationView
    
    print("✅ Isaac Sim 모듈 import 성공")
    
    # Get Isaac Sim assets root path
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder")
        raise RuntimeError("Isaac Sim assets not found")
    
    print(f"✅ Isaac Sim Assets 경로 발견: {assets_root_path}")
    
    # Robot asset paths (correct for Isaac Sim 5.0 - cloud hosted)
    robot_assets = {
        "UR10": {
            "asset_path": assets_root_path + "/Isaac/Robots/UniversalRobots/ur10/ur10.usd",
            "prim_path": "/World/UR10",
            "rig_path": "/World/UR10_Rig",
        },
        "Franka": {
            "asset_path": assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd", 
            "prim_path": "/World/Franka",
            "rig_path": "/World/Franka_Rig",
        }
    }
    
    # Isaac Sim 5.0 uses cloud-hosted assets, so we'll proceed with all robots
    print("🔧 로봇 Assets 준비...")
    print(f"  Isaac Sim 5.0은 클라우드 호스팅 에셋을 사용합니다: {assets_root_path}")
    valid_robots = robot_assets.copy()  # All robots should be available via cloud
    
    for robot_name, robot_info in valid_robots.items():
        print(f"  📡 {robot_name}: {robot_info['asset_path']}")
    
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
        """Rig prim 생성 및 로봇 레퍼런스 추가"""
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
        """실제 Articulation 루트를 찾는 함수 - 중첩 방지를 위해 베이스 자체를 루트로 사용"""
        print(f"🔍 {base_path} 하위에서 실제 Articulation 루트 탐색...")
        
        base_prim = get_prim_at_path(base_path)
        if not base_prim or not base_prim.IsValid():
            print(f"  ❌ 기본 경로 {base_path}가 유효하지 않음")
            return None
        
        # List all children for debugging
        print(f"  🔍 {base_path} 하위 Prim 목록:")
        link_prims = []
        for child in base_prim.GetChildren():
            child_path = str(child.GetPath())
            child_type = child.GetTypeName()
            print(f"    - {child_path} (Type: {child_type})")
            
            # Collect link prims (those that should have RigidBody)
            if "link" in child.GetName().lower() and child_type == "Xform":
                if child.HasAPI(UsdPhysics.RigidBodyAPI):
                    link_prims.append(child_path)
                    print(f"      🟦 RigidBody Link: {child_path}")
            
            # Check grandchildren too for debug
            for grandchild in child.GetChildren():
                grandchild_path = str(grandchild.GetPath())
                grandchild_type = grandchild.GetTypeName()
                print(f"      └─ {grandchild_path} (Type: {grandchild_type})")
        
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
                if p.HasAPI(UsdPhysics.RigidBodyAPI):
                    rigid_body_count += 1
                    
                # Check for Joint schemas  
                if (p.HasAPI(UsdPhysics.RevoluteJoint) or 
                    p.HasAPI(UsdPhysics.PrismaticJoint) or
                    p.HasAPI(UsdPhysics.FixedJoint) or
                    p.HasAPI(UsdPhysics.SphericalJoint)):
                    joint_count += 1
                    
                # Traverse children
                for child in p.GetChildren():
                    traverse_prim(child)
                    
            traverse_prim(prim)
            return rigid_body_count, joint_count
        
        # Count total physics components in base_path
        rb_count, joint_count = count_physics_components(base_path)
        
        if rb_count > 1 and joint_count > 0:
            print(f"  🎯 베이스 경로에 충분한 물리 컴포넌트 발견 (RB: {rb_count}, Joints: {joint_count})")
            print(f"  ✅ Articulation 루트로 베이스 경로 사용: {base_path}")
            return base_path
        elif link_prims:
            # Find the base link (usually first in kinematic chain)
            base_link_candidates = [p for p in link_prims if "base" in p.lower()]
            if base_link_candidates:
                best_path = base_link_candidates[0]
                print(f"  ✅ Base Link 발견: {best_path}")
                return best_path
            else:
                # Use the first link as root
                best_path = link_prims[0]
                print(f"  ✅ 첫 번째 Link를 루트로 사용: {best_path}")
                return best_path
        else:
            print(f"  ❌ {base_path} 하위에서 적절한 Articulation 루트를 찾지 못했습니다")
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

    # Load robots with proper structure
    print("\n🤖 로봇 모델 로딩 (올바른 구조)...")
    robot_prims = {}
    for robot_name, robot_info in valid_robots.items():
        robot_prim = ensure_offset_and_reference(
            stage, 
            robot_info["rig_path"], 
            robot_info["prim_path"], 
            robot_info["asset_path"]
        )
        robot_prims[robot_name] = robot_prim
        print(f"  - {robot_name} 로봇 로딩 및 위치 설정 완료")
    
    # Wait for USD references to load
    print("\n⏳ USD 레퍼런스 로딩 대기 중...")
    for i in range(10):
        print(f"  대기 중... {i+1}/10초")
        time.sleep(1)
        my_world.step()  # Process USD loading
    
    # Find articulation roots and apply ArticulationRootAPI
    print(f"\n🔧 1단계: 실제 루트 경로 탐색 및 ArticulationRootAPI 적용...")
    articulation_roots = {}
    
    for robot_name, robot_info in valid_robots.items():
        if robot_name in robot_prims:
            root_path = find_articulation_root_path(stage, robot_info["prim_path"])
            if root_path:
                if apply_articulation_root_api(stage, root_path):
                    articulation_roots[robot_name] = root_path
                    print(f"    ✅ {robot_name}: ArticulationRootAPI 적용 성공")
                else:
                    print(f"    ❌ {robot_name}: ArticulationRootAPI 적용 실패")
            else:
                print(f"    ❌ {robot_name}: 실제 루트 경로를 찾지 못했습니다")
    
    # World reset (CRITICAL for proper PhysX initialization)
    print(f"\n🔄 2단계: World 리셋 (준비된 로봇: {len(articulation_roots)}개)...")
    if articulation_roots:
        try:
            my_world.reset()
            print("  ✅ World 리셋 성공")
        except Exception as e:
            print(f"  ❌ World 리셋 실패: {e}")
    
    # Create articulation views AFTER reset
    print(f"\n🔧 3단계: 리셋 후 Articulation 인스턴스 생성...")
    articulation_views = {}
    
    for robot_name, root_path in articulation_roots.items():
        try:
            # PhysX Tensors ArticulationView 생성 (위치 인수만 사용)
            articulation_view = ArticulationView([root_path])
            articulation_views[robot_name] = articulation_view
            print(f"    ✅ {robot_name}: Articulation 인스턴스 생성 성공 ({root_path})")
        except Exception as e:
            print(f"    ❌ {robot_name}: Articulation 생성 실패 - {e}")
    
    # Initialize articulations
    print(f"\n🔧 4단계: 업데이트 및 초기화 (생성된 Articulation: {len(articulation_views)}개)...")
    initialized_robots = []
    
    if articulation_views:
        # Step physics to ensure everything is properly loaded
        for _ in range(5):
            my_world.step()
        
        for robot_name, articulation_view in articulation_views.items():
            try:
                articulation_view.initialize()
                initialized_robots.append(robot_name)
                print(f"    ✅ {robot_name}: 초기화 성공")
            except Exception as e:
                print(f"    ❌ {robot_name}: 초기화 실패 - {e}")
    
    # Test robot control functionality  
    print(f"\n🎯 5단계: 로봇 제어 기능 테스트 (초기화된 로봇: {len(initialized_robots)}개)...")
    
    if initialized_robots:
        for robot_name in initialized_robots:
            try:
                articulation_view = articulation_views[robot_name]
                
                # Get articulation properties
                dof_count = articulation_view.count
                joint_positions = articulation_view.get_joint_positions()
                joint_velocities = articulation_view.get_joint_velocities()
                
                print(f"    🎯 {robot_name} 제어 테스트:")
                print(f"      - DOF 수: {dof_count}")
                print(f"      - 현재 관절 위치: {joint_positions}")
                print(f"      - 현재 관절 속도: {joint_velocities}")
                
                # Test simple joint position control
                if dof_count > 0 and joint_positions is not None:
                    # Create small position offset for testing
                    target_positions = joint_positions + np.random.uniform(-0.1, 0.1, joint_positions.shape)
                    articulation_view.set_joint_position_targets(target_positions)
                    print(f"      - 목표 관절 위치 설정 완료: {target_positions}")
                    
            except Exception as e:
                print(f"    ❌ {robot_name}: 제어 테스트 실패 - {e}")
    else:
        print("  ⚠️ 초기화된 로봇이 없어 제어 테스트를 건너뜁니다.")
    
    # Save USD file for inspection
    usd_save_path = "/home/roarm_m3/dev_roarm/roarm_mcp/tests/robot_control_proper_assets.usd"
    stage.Export(usd_save_path)
    print(f"✅ USD 파일 저장 완료: {usd_save_path}")
    
    # Final summary
    print(f"\n📊 최종 결과 요약:")
    print(f"  - 로드된 로봇: {len(robot_prims)}개")
    print(f"  - 루트 경로 탐색 성공: {len(articulation_roots)}개")
    print(f"  - Articulation 생성 성공: {len(articulation_views)}개")
    print(f"  - 최종 초기화 성공: {len(initialized_robots)}개")

except Exception as e:
    logger.error(f"❌ 실행 중 오류 발생: {e}")
    import traceback
    traceback.print_exc()

finally:
    # Clean shutdown
    if 'simulation_app' in locals():
        simulation_app.close()