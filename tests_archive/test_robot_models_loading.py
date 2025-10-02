#!/usr/bin/env python3
"""
RoArm MCP - Isaac Sim 로봇 모델 로딩 테스트
UR10, Franka 등 주요 로봇 모델들의 Isaac Sim 연동 테스트
"""

import sys
import time
import numpy as np
print("🤖 RoArm MCP - Isaac Sim 로봇 모델 로딩 테스트")
print(f"Python 버전: {sys.version}")
print(f"작업 디렉토리: {sys.path[0] if sys.path else 'N/A'}")

def test_robot_models_loading():
    """Isaac Sim에서 UR10, Franka 로봇 모델 로딩 테스트"""
    
    # Isaac Sim 5.0 앱 초기화
    from isaacsim import SimulationApp
    simulation_app = SimulationApp({"headless": True})
    
    try:
        print("=== Isaac Sim 로봇 모델 로딩 테스트 ===")
        print("✅ Isaac Sim 5.0 SimulationApp 초기화 성공")
        
        # 필요한 모듈들 import
        from pxr import Usd, UsdGeom, UsdLux, Sdf
        import omni.usd
        from omni.isaac.core import World
        from omni.isaac.core.utils.stage import add_reference_to_stage
        import omni.isaac.core.utils.prims as prim_utils
        
        # USD Context 및 Stage 설정
        ctx = omni.usd.get_context()
        created = ctx.new_stage()
        if not created:
            raise RuntimeError("USD Stage 생성 실패")
        
        stage = ctx.get_stage()
        if stage is None:
            raise RuntimeError("ctx.get_stage()가 None을 반환했습니다")
        
        print("✅ USD Stage 생성 성공")
        
        # World 초기화
        world = World()
        print("✅ Isaac Sim World 초기화 성공")
        
        # 기본 조명 설정
        light_path = Sdf.Path("/World/DistantLight")
        light_prim = UsdLux.DistantLight.Define(stage, light_path)
        distant_light = UsdLux.DistantLight(stage.GetPrimAtPath(light_path))
        distant_light.CreateIntensityAttr(3000.0)
        print("✅ 기본 조명 설정 완료")
        
        # 바닥 추가
        from omni.isaac.core.objects import GroundPlane
        ground_plane = GroundPlane(prim_path="/World/groundPlane")
        world.scene.add(ground_plane)
        print("✅ 바닥 평면 추가 완료")
        
        # UR10 로봇 모델 로딩 테스트
        print("\n🔍 UR10 로봇 모델 로딩 시도...")
        try:
            # Isaac Sim에 내장된 UR10 asset 경로
            ur10_asset_path = "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
            ur10_prim_path = "/World/UR10"
            
            # UR10 추가
            add_reference_to_stage(usd_path=ur10_asset_path, prim_path=ur10_prim_path)
            
            # UR10 프림 확인
            ur10_prim = stage.GetPrimAtPath(ur10_prim_path)
            if ur10_prim.IsValid():
                print("✅ UR10 로봇 모델 로딩 성공")
                
                # UR10 위치 설정 - USD API 타입 안전성 보장
                from pxr import Gf
                ur10_xform = UsdGeom.Xformable(ur10_prim)
                translation = Gf.Vec3d(0.0, 0.0, 0.0)  # 원점에 배치
                # GfMatrix4d 타입으로 명시적 변환
                transform_matrix = Gf.Matrix4d().SetTranslate(translation)
                ur10_xform.AddTransformOp().Set(transform_matrix)
                print("✅ UR10 위치 설정 완료")
                
            else:
                print("⚠️ UR10 Prim이 유효하지 않음")
                
        except Exception as ur10_error:
            print(f"❌ UR10 로딩 실패: {str(ur10_error)}")
        
        # Franka 로봇 모델 로딩 테스트  
        print("\n🔍 Franka 로봇 모델 로딩 시도...")
        try:
            # Isaac Sim에 내장된 Franka asset 경로
            franka_asset_path = "/Isaac/Robots/Franka/franka.usd"
            franka_prim_path = "/World/Franka"
            
            # Franka 추가
            add_reference_to_stage(usd_path=franka_asset_path, prim_path=franka_prim_path)
            
            # Franka 프림 확인
            franka_prim = stage.GetPrimAtPath(franka_prim_path)
            if franka_prim.IsValid():
                print("✅ Franka 로봇 모델 로딩 성공")
                
                # Franka 위치 설정 (UR10 옆에 배치) - USD API 타입 안전성 보장
                from pxr import Gf
                franka_xform = UsdGeom.Xformable(franka_prim)
                translation = Gf.Vec3d(1.5, 0.0, 0.0)  # UR10 옆에 배치
                # GfMatrix4d 타입으로 명시적 변환
                transform_matrix = Gf.Matrix4d().SetTranslate(translation)
                franka_xform.AddTransformOp().Set(transform_matrix)
                print("✅ Franka 위치 설정 완료")
                
            else:
                print("⚠️ Franka Prim이 유효하지 않음")
                
        except Exception as franka_error:
            print(f"❌ Franka 로딩 실패: {str(franka_error)}")
        
        # World 초기화 및 물리 시뮬레이션 준비 (Articulation 생성 전)
        print("\n⚙️ 시뮬레이션 환경 초기화...")
        world.reset()
        print("✅ World 초기 리셋 완료")
        
        # Isaac Sim Articulation 인터페이스 테스트
        print("\n🔧 로봇 제어 인터페이스 테스트...")
        
        try:
            from omni.isaac.core.articulations import Articulation
            
            # UR10 Articulation 생성 - World가 초기화된 후
            if stage.GetPrimAtPath("/World/UR10").IsValid():
                ur10_robot = Articulation(prim_path="/World/UR10", name="ur10_robot")
                world.scene.add(ur10_robot)
                print("✅ UR10 Articulation 인터페이스 생성 성공")
                
            # Franka Articulation 생성 - World가 초기화된 후
            if stage.GetPrimAtPath("/World/Franka").IsValid():
                franka_robot = Articulation(prim_path="/World/Franka", name="franka_robot")
                world.scene.add(franka_robot)
                print("✅ Franka Articulation 인터페이스 생성 성공")
                
            # Articulation이 추가된 후 다시 초기화
            world.reset()
            print("✅ Articulation 초기화 완료")
                
        except Exception as articulation_error:
            print(f"❌ Articulation 인터페이스 생성 실패: {str(articulation_error)}")
        
        # 로봇 정보 출력
        print("\n📊 로드된 로봇 모델 정보:")
        for name, articulation in world.scene.get_articulations().items():
            print(f"  - {name}: {articulation.prim_path}")
            
            # DOF 정보 출력
            try:
                dof_names = articulation.dof_names
                if dof_names:
                    print(f"    DOF 개수: {len(dof_names)}")
                    print(f"    DOF 이름: {dof_names}")
                else:
                    print("    DOF 정보 없음")
            except:
                print("    DOF 정보 조회 실패")
        
        # 간단한 시뮬레이션 실행
        print("\n🔄 시뮬레이션 테스트 실행...")
        simulation_steps = 100
        for step in range(simulation_steps):
            world.step(render=False)
            if step % 25 == 0:
                print(f"  📊 시뮬레이션 스텝: {step + 1}/{simulation_steps}")
        
        print("✅ 시뮬레이션 테스트 완료")
        
        # 결과 저장
        output_path = "/home/roarm_m3/dev_roarm/roarm_mcp/tests/robot_models_scene.usd"
        try:
            stage.GetRootLayer().Export(output_path)
            print(f"✅ 로봇 모델 씬 저장 완료: {output_path}")
        except Exception as save_error:
            print(f"⚠️ 씬 저장 실패: {save_error}")
        
        print("\n🎯 로봇 모델 로딩 테스트 완료!")
        print("📝 다음 단계: 로봇 제어 및 MCP 통신 테스트")
        
        return True
        
    except Exception as e:
        print(f"❌ 로봇 모델 로딩 테스트 오류: {str(e)}")
        import traceback
        traceback.print_exc()
        return False
    
    finally:
        # 정리 작업
        try:
            simulation_app.close()
            print("🔄 Isaac Sim 종료 완료")
        except Exception as cleanup_error:
            print(f"⚠️ 정리 작업 중 오류: {cleanup_error}")

if __name__ == "__main__":
    success = test_robot_models_loading()
    exit(0 if success else 1)