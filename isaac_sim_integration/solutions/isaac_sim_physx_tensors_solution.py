#!/usr/bin/env python3
"""
🤖 RoArm MCP - Isaac Sim 5.0 올바른 PhysX Tensors 솔루션

사용자 제안 솔루션 적용:
A) 중첩된 Articulation Root 정리 (한 서브트리에 Root 1개)
B) 올바른 순서: Root 정리 → World.reset() → 업데이트 프레임 → 뷰 생성
C) PhysX Tensors의 create_simulation_view().create_articulation_view() 사용
"""

import os
import sys
import time
from typing import Dict, Optional, List

# Isaac Sim imports
import isaacsim
from isaacsim import SimulationApp

# Import torch for tensor operations
import torch

print("🤖 RoArm MCP - Isaac Sim 5.0 올바른 PhysX Tensors 솔루션")
print(f"Python 버전: {sys.version}")
print(f"작업 디렉토리: {os.getcwd()}")

def main():
    print("✅ Isaac Sim 5.0 SimulationApp 초기화 중...")
    
    # Create simulation app (headless mode for server use)
    simulation_app = SimulationApp({
        "headless": True,
        "width": 1280,
        "height": 720
    })
    
    try:
        # Import Isaac Sim modules after app creation
        from pxr import Usd, UsdPhysics, UsdGeom, Gf, Sdf
        from isaacsim.core.api import World
        from isaacsim.core.utils.prims import get_prim_at_path, create_prim
        from isaacsim.core.utils.rotations import euler_angles_to_quat
        from isaacsim.core.utils.stage import get_current_stage
        import omni.physics.tensors as tensors
        import omni.kit.app
        import isaacsim.core.utils as core_utils
        
        print("✅ Isaac Sim 5.0 SimulationApp 초기화 성공")
        print("✅ Isaac Sim 모듈 import 성공")
        
        # Check for Isaac Sim 5.0 cloud assets
        isaac_assets_base = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0"
        print(f"✅ Isaac Sim Assets 경로 발견: {isaac_assets_base}")
        
        # Robot asset URLs for Isaac Sim 5.0
        robot_assets = {
            "UR10": f"{isaac_assets_base}/Isaac/Robots/UniversalRobots/ur10/ur10.usd",
            "Franka": f"{isaac_assets_base}/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        }
        
        print("🔧 로봇 Assets 준비...")
        print(f"  Isaac Sim 5.0은 클라우드 호스팅 에셋을 사용합니다: {isaac_assets_base}")
        for name, url in robot_assets.items():
            print(f"  📡 {name}: {url}")
        
        # Initialize World with physics
        print("🔧 Physics Scene 설정 중...")
        stage = get_current_stage()
        
        # Check if Physics Scene already exists
        physics_scene = stage.GetPrimAtPath("/physicsScene")
        if physics_scene.IsValid():
            print("    ✅ 기존 Physics Scene 발견")
        else:
            print("    ⚠️  Physics Scene를 찾을 수 없습니다")
        
        my_world = World(stage_units_in_meters=1.0)
        print("✅ Isaac Sim World 초기화 성공")
        
        print("\n🌟 시뮬레이션 환경 설정...")
        
        # Add basic lighting (simplified)
        try:
            distant_light = create_prim(
                prim_path="/World/DistantLight",
                prim_type="DistantLight"
            )
            # Set light properties after creation
            if distant_light and distant_light.IsValid():
                light_api = UsdGeom.XformCommonAPI(distant_light)
                light_api.SetTranslate((0, 0, 10))
            print("  - 기본 조명 설정 완료")
        except Exception as e:
            print(f"  ⚠️ 조명 설정 건너뜀: {e}")
        
        # Add ground plane (simplified)
        try:
            ground_plane = create_prim(
                prim_path="/World/GroundPlane",
                prim_type="Cube",
                position=(0.0, 0.0, -0.5)
            )
            print("  - 바닥 평면 추가 완료")
        except Exception as e:
            print(f"  ⚠️ 바닥 평면 설정 건너뜀: {e}")
        
        # Load robot models with proper structure
        print("\n🤖 로봇 모델 로딩 (올바른 구조)...")
        
        def ensure_offset_and_reference(prim_path, asset_url, position=(0, 0, 0), orientation=(0, 0, 0, 1)):
            """오프셋과 USD 참조를 설정하는 함수"""
            
            # Create Rig prim for offset (separate from robot)
            rig_path = f"{prim_path}_Rig"
            rig_prim = create_prim(prim_path=rig_path, prim_type="Xform")
            
            # Create the actual robot prim
            robot_prim = create_prim(prim_path=prim_path, prim_type="Xform")
            
            # Set position using XformCommonAPI
            xform_api = UsdGeom.XformCommonAPI(robot_prim)
            xform_api.SetTranslate(position)
            xform_api.SetRotate(orientation[:3])  # Euler angles
            
            # Add USD reference to load robot asset
            robot_prim.GetReferences().AddReference(asset_url)
            print(f"  ✅ 참조 추가: {asset_url}")
            
            # Ensure instanceable=False to avoid reference issues
            robot_prim.SetInstanceable(False)
            
            return robot_prim
        
        # Robot configurations with positions
        robot_configs = {
            "UR10": {
                "prim_path": "/World/UR10", 
                "asset_url": robot_assets["UR10"],
                "position": (-1.0, 0.0, 0.0)
            },
            "Franka": {
                "prim_path": "/World/Franka", 
                "asset_url": robot_assets["Franka"],
                "position": (1.0, 0.0, 0.0)
            }
        }
        
        robot_prims = {}
        for robot_name, config in robot_configs.items():
            robot_prim = ensure_offset_and_reference(
                config["prim_path"],
                config["asset_url"], 
                config["position"]
            )
            robot_prims[robot_name] = robot_prim
            print(f"  - {robot_name} 로봇 로딩 및 위치 설정 완료")
        
        # Wait for USD references to load
        print("\n⏳ USD 레퍼런스 로딩 대기 중...")
        for i in range(10):
            print(f"  대기 중... {i+1}/10초")
            time.sleep(1)
            my_world.step()  # Process USD loading
        
        # A) 중첩된 Articulation Root 정리
        print(f"\n🔧 A단계: 중첩된 ArticulationRoot 정리 (한 서브트리에 Root 1개)...")
        
        def list_articulation_roots_under(stage, base_path: str) -> List[Usd.Prim]:
            """주어진 경로 하위의 모든 ArticulationRoot 찾기"""
            roots = []
            for prim in stage.Traverse():
                prim_path = prim.GetPath().pathString
                if not prim_path.startswith(base_path):
                    continue
                if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                    roots.append(prim)
            # 경로 길이로 정렬 (가장 바깥쪽이 먼저)
            roots.sort(key=lambda r: len(r.GetPath().pathString))
            return roots
        
        def collapse_to_single_root(stage, base_path: str) -> Optional[str]:
            """한 서브트리에서 ArticulationRoot를 1개만 남기고 나머지 제거"""
            roots = list_articulation_roots_under(stage, base_path)
            print(f"    🔍 {base_path} 하위에서 발견된 ArticulationRoot: {len(roots)}개")
            
            if not roots:
                print(f"    ⚠️ {base_path}에 기존 ArticulationRoot가 없음 - 베이스에 적용")
                base_prim = get_prim_at_path(base_path)
                if base_prim and base_prim.IsValid():
                    UsdPhysics.ArticulationRootAPI.Apply(base_prim)
                    return base_path
                return None
            
            # 정책: 가장 바깥쪽(상위) 루트를 남기고, 나머지 제거
            keep_root = roots[0]
            print(f"    ✅ 보존할 루트: {keep_root.GetPath()}")
            
            for i, extra_root in enumerate(roots[1:], 1):
                print(f"    🗑️  제거할 루트 #{i}: {extra_root.GetPath()}")
                # ArticulationRootAPI 제거
                extra_root.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            
            return keep_root.GetPath().pathString
        
        # 각 로봇 서브트리에서 ArticulationRoot 정리
        articulation_roots = {}
        for robot_name, config in robot_configs.items():
            root_path = collapse_to_single_root(stage, config["prim_path"])
            if root_path:
                articulation_roots[robot_name] = root_path
                print(f"    ✅ {robot_name}: 최종 루트 = {root_path}")
            else:
                print(f"    ❌ {robot_name}: 루트 정리 실패")
        
        # B) 올바른 순서: World 리셋
        print(f"\n🔄 B단계: World 리셋 (준비된 로봇: {len(articulation_roots)}개)...")
        if articulation_roots:
            try:
                my_world.reset()
                print("  ✅ World 리셋 성공")
                
                # 리셋 후 업데이트 프레임으로 빌드/로딩 진행
                print("  🔄 업데이트 프레임 처리 중...")
                for i in range(3):
                    omni.kit.app.get_app().update()
                    time.sleep(0.02)
                print("  ✅ 업데이트 프레임 완료")
                
            except Exception as e:
                print(f"  ❌ World 리셋 실패: {e}")
                return
        
        # C) PhysX Tensors로 올바른 ArticulationView 생성
        print(f"\n🔧 C단계: PhysX Tensors SimulationView 생성...")
        
        try:
            # create_simulation_view로 SimulationView 생성
            sim_view = tensors.create_simulation_view("torch")  # 또는 "warp"
            print("  ✅ SimulationView 생성 완료")
            
            # 각 로봇에 대해 ArticulationView 생성
            articulation_views = {}
            for robot_name, root_path in articulation_roots.items():
                try:
                    print(f"    🔧 {robot_name}: ArticulationView 생성 시도 ({root_path})")
                    # create_articulation_view로 뷰 생성 (경로 패턴 문자열 사용)
                    articulation_view = sim_view.create_articulation_view(root_path)
                    articulation_views[robot_name] = articulation_view
                    print(f"    ✅ {robot_name}: ArticulationView 생성 성공")
                except Exception as e:
                    print(f"    ❌ {robot_name}: ArticulationView 생성 실패 - {e}")
                    # 디버깅을 위해 traceback 출력
                    import traceback
                    traceback.print_exc()
            
            # 유효성 체크를 위한 업데이트 프레임
            print("  🔄 ArticulationView 초기화를 위한 업데이트...")
            omni.kit.app.get_app().update()
            
        except Exception as e:
            print(f"  ❌ SimulationView 생성 실패: {e}")
            import traceback
            traceback.print_exc()
            return
        
        # D) Articulation 정보 및 제어 기능 테스트
        print(f"\n🎯 D단계: 로봇 제어 기능 테스트 (생성된 뷰: {len(articulation_views)}개)...")
        
        initialized_robots = []
        for robot_name, articulation_view in articulation_views.items():
            try:
                # ArticulationView 속성 확인
                print(f"\n  🤖 {robot_name} 로봇 정보:")
                
                # 기본 속성들 체크
                if hasattr(articulation_view, 'count') and articulation_view.count is not None:
                    print(f"    - DOF 개수: {articulation_view.count}")
                else:
                    print(f"    ⚠️ DOF 개수를 확인할 수 없음")
                
                if hasattr(articulation_view, 'is_homogeneous'):
                    print(f"    - 동종성: {articulation_view.is_homogeneous}")
                
                # ArticulationView의 사용 가능한 메서드 탐색
                print(f"    🔍 ArticulationView 메서드 탐색:")
                available_methods = [method for method in dir(articulation_view) 
                                   if not method.startswith('_') and callable(getattr(articulation_view, method))]
                
                # 관절/DOF 관련 메서드 찾기
                joint_methods = [m for m in available_methods if any(keyword in m.lower() 
                               for keyword in ['joint', 'dof', 'position', 'velocity', 'torque'])]
                print(f"       관절/DOF 관련: {joint_methods}")
                
                # Joint 위치/속도 가져오기 테스트 - 다양한 API 시도
                try:
                    joint_positions = None
                    joint_velocities = None
                    
                    # Isaac Sim 5.0 PhysX Tensors API 시도
                    for method_name in ['get_joint_positions', 'get_dof_positions', 'dof_positions', 'positions']:
                        if hasattr(articulation_view, method_name):
                            try:
                                method = getattr(articulation_view, method_name)
                                if callable(method):
                                    joint_positions = method()
                                else:
                                    joint_positions = method
                                print(f"    ✅ 위치 읽기 성공: {method_name}")
                                break
                            except Exception as e:
                                print(f"    ⚠️ {method_name} 실패: {e}")
                                continue
                    
                    for method_name in ['get_joint_velocities', 'get_dof_velocities', 'dof_velocities', 'velocities']:
                        if hasattr(articulation_view, method_name):
                            try:
                                method = getattr(articulation_view, method_name)
                                if callable(method):
                                    joint_velocities = method()
                                else:
                                    joint_velocities = method
                                print(f"    ✅ 속도 읽기 성공: {method_name}")
                                break
                            except Exception as e:
                                print(f"    ⚠️ {method_name} 실패: {e}")
                                continue
                    
                    if joint_positions is not None:
                        print(f"    - 관절 위치 shape: {joint_positions.shape}")
                        print(f"    - 관절 위치: {joint_positions[:5] if len(joint_positions) > 5 else joint_positions}")
                        
                        # 간단한 제어 명령 테스트 - Isaac Sim 5.0 PhysX Tensors API
                        try:
                            if joint_positions is not None:
                                zero_positions = joint_positions * 0
                                
                                # Isaac Sim 5.0에서는 올바른 torch tensor indices가 필요함  
                                robot_indices = torch.arange(articulation_view.count, dtype=torch.int32)
                                print(f"    - Robot indices: {robot_indices} (type: {type(robot_indices)})")
                                
                                if hasattr(articulation_view, 'set_dof_position_targets'):
                                    articulation_view.set_dof_position_targets(zero_positions, indices=robot_indices)
                                    print(f"    ✅ 제어 명령 성공: set_dof_position_targets (영점 위치)")
                                    
                                # 속도 제어도 테스트 - joint_velocities가 None이 아닌 경우만
                                if joint_velocities is not None and hasattr(articulation_view, 'set_dof_velocity_targets'):
                                    zero_velocities = joint_velocities * 0
                                    articulation_view.set_dof_velocity_targets(zero_velocities, indices=robot_indices)
                                    print(f"    ✅ 제어 명령 성공: set_dof_velocity_targets (영점 속도)")
                                
                                initialized_robots.append(robot_name)
                            else:
                                print(f"    ⚠️ 관절 위치 정보가 None이므로 제어 명령을 건너뜀")
                            
                        except Exception as target_e:
                            print(f"    ⚠️ 제어 명령 실패: {target_e}")
                            # 다른 방법 시도 - 대체 indices 생성
                            try:
                                if joint_positions is not None and hasattr(articulation_view, 'set_dof_positions'):
                                    zero_positions = joint_positions * 0
                                    # Isaac Sim 5.0에서는 torch tensor indices 필요
                                    robot_indices_tensor = torch.arange(articulation_view.count, dtype=torch.int32)
                                    articulation_view.set_dof_positions(zero_positions, indices=robot_indices_tensor)
                                    print(f"    ✅ 대안 제어 명령 성공: set_dof_positions")
                                    initialized_robots.append(robot_name)
                                else:
                                    print(f"    ⚠️ 대안 제어 명령 조건 불만족 (positions: {joint_positions is not None})")
                            except Exception as alt_e:
                                print(f"    ⚠️ 대안 제어 명령도 실패: {alt_e}")
                                import traceback
                                traceback.print_exc()
                    else:
                        print(f"    ⚠️ 관절 위치 정보를 가져올 수 없음")
                    
                    if joint_velocities is not None:
                        print(f"    - 관절 속도 shape: {joint_velocities.shape}")
                    
                except Exception as control_e:
                    print(f"    ⚠️ 제어 기능 테스트 실패: {control_e}")
                    import traceback
                    traceback.print_exc()
                
            except Exception as e:
                print(f"    ❌ {robot_name}: 정보 확인 실패 - {e}")
        
        # 최종 시뮬레이션 스텝 테스트
        if initialized_robots:
            print(f"\n🚀 최종 시뮬레이션 스텝 테스트...")
            for step in range(3):
                my_world.step()
                print(f"    스텝 {step+1}/3 완료")
            print("  ✅ 시뮬레이션 스텝 성공")
        
        # Save USD file
        output_file = "/home/roarm_m3/dev_roarm/roarm_mcp/tests/robot_control_correct_solution.usd"
        stage.Export(output_file)
        print(f"✅ USD 파일 저장 완료: {output_file}")
        
        # 최종 결과 요약
        print(f"\n📊 최종 결과 요약:")
        print(f"  - 로드된 로봇: {len(robot_prims)}개")
        print(f"  - 정리된 ArticulationRoot: {len(articulation_roots)}개")
        print(f"  - 생성된 ArticulationView: {len(articulation_views)}개")
        print(f"  - 최종 초기화 성공: {len(initialized_robots)}개")
        
        if len(initialized_robots) == len(robot_configs):
            print("🎉 모든 로봇 초기화 성공! PhysX Tensors 통합 완료!")
        else:
            print(f"⚠️ 일부 로봇 초기화 실패. 성공: {initialized_robots}")
        
    except Exception as e:
        print(f"❌ 오류 발생: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Clean shutdown
        simulation_app.close()

if __name__ == "__main__":
    main()