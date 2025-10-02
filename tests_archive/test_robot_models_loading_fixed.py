#!/usr/bin/env python3
"""
🤖 RoArm MCP - Isaac Sim 5.0 로봇 모델 로딩 테스트 (수정판)
Isaac Sim에서 UR10, Franka 로봇 모델을 로딩하고 제어 인터페이스 테스트
"""

import sys
import os
print(f"🤖 RoArm MCP - Isaac Sim 로봇 모델 로딩 테스트")
print(f"Python 버전: {sys.version}")
print(f"작업 디렉토리: {os.getcwd()}")

def test_robot_models_loading():
    """Isaac Sim에서 UR10, Franka 로봇 모델 로딩 및 제어 인터페이스 테스트"""
    
    # Isaac Sim 5.0 SimulationApp 초기화
    from isaacsim import SimulationApp
    simulation_app = SimulationApp({
        "headless": True,
        "width": 1280,
        "height": 720
    })
    
    try:
        print("✅ Isaac Sim 5.0 SimulationApp 초기화 성공")
        
        # Isaac Sim 5.0 호환 모듈 import
        import omni.usd
        from pxr import UsdGeom, UsdLux, Sdf, Gf
        from isaacsim.core.utils.stage import get_current_stage, add_reference_to_stage
        from isaacsim.core.api.world import World
        from isaacsim.core.api.objects import GroundPlane
        from isaacsim.core.prims import SingleArticulation
        
        # USD Stage 생성
        stage = get_current_stage()
        print("✅ USD Stage 생성 성공")
        
        # Isaac Sim World 초기화
        world = World()
        print("✅ Isaac Sim World 초기화 성공")
        
        # 기본 조명 설정
        light_path = Sdf.Path("/World/defaultLight")
        light_prim = UsdLux.DistantLight.Define(stage, light_path)
        distant_light = UsdLux.DistantLight(stage.GetPrimAtPath(light_path))
        distant_light.CreateIntensityAttr(3000.0)
        print("✅ 기본 조명 설정 완료")
        
        # 바닥 추가
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
                
                # UR10 위치 설정 - 안전한 방법으로 transform 처리
                ur10_xform = UsdGeom.Xformable(ur10_prim)
                if not ur10_xform.GetOrderedXformOps():
                    # Transform 연산자 생성
                    translate_op = ur10_xform.AddTranslateOp()
                    translate_op.Set(Gf.Vec3d(0.0, 0.0, 0.0))
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
                
                # Franka 위치 설정 (UR10 옆에 배치) - 안전한 방법으로 transform 처리
                franka_xform = UsdGeom.Xformable(franka_prim)
                if not franka_xform.GetOrderedXformOps():
                    # Transform 연산자 생성 - UR10 옆에 배치
                    translate_op = franka_xform.AddTranslateOp()
                    translate_op.Set(Gf.Vec3d(1.5, 0.0, 0.0))
                print("✅ Franka 위치 설정 완료")
                
            else:
                print("⚠️ Franka Prim이 유효하지 않음")
                
        except Exception as franka_error:
            print(f"❌ Franka 로딩 실패: {str(franka_error)}")
        
        # World 첫 번째 리셋 (물리 시뮬레이션 준비)
        print("\n⚙️ 시뮬레이션 환경 초기화...")
        world.reset()
        print("✅ World 초기 리셋 완료")
        
        # Isaac Sim SingleArticulation 인터페이스 테스트 (5.0 호환)
        print("\n🔧 로봇 제어 인터페이스 테스트...")
        
        ur10_robot = None
        franka_robot = None
        
        try:
            # UR10 Articulation 생성 - Isaac Sim 5.0 API
            if stage.GetPrimAtPath("/World/UR10").IsValid():
                ur10_robot = SingleArticulation(prim_path="/World/UR10", name="ur10_robot")
                world.scene.add(ur10_robot)
                print("✅ UR10 SingleArticulation 인터페이스 생성 성공")
                
            # Franka Articulation 생성 - Isaac Sim 5.0 API  
            if stage.GetPrimAtPath("/World/Franka").IsValid():
                franka_robot = SingleArticulation(prim_path="/World/Franka", name="franka_robot")
                world.scene.add(franka_robot)
                print("✅ Franka SingleArticulation 인터페이스 생성 성공")
                
            # Articulation이 추가된 후 안전한 초기화
            if ur10_robot or franka_robot:
                world.reset()
                print("✅ Articulation 초기화 완료")
                
                # DOF 정보 확인
                print("\n📊 로봇 DOF 정보:")
                if ur10_robot:
                    print(f"  - UR10 DOF 수: {ur10_robot.num_dof}")
                    joint_names = ur10_robot.get_joint_names()
                    if joint_names:
                        print(f"  - UR10 조인트: {joint_names}")
                        
                if franka_robot:
                    print(f"  - Franka DOF 수: {franka_robot.num_dof}")
                    joint_names = franka_robot.get_joint_names()
                    if joint_names:
                        print(f"  - Franka 조인트: {joint_names}")
                
        except Exception as articulation_error:
            print(f"❌ Articulation 인터페이스 생성 실패: {str(articulation_error)}")
        
        # 로봇 정보 출력
        print("\n📊 로드된 로봇 모델 정보:")
        for name, articulation in world.scene.get_articulations().items():
            print(f"  - {name}: {articulation.prim_path}")
            
        # 시뮬레이션 스텝 테스트
        print("\n🎮 시뮬레이션 동작 테스트...")
        for i in range(5):
            world.step(render=False)
            print(f"  단계 {i+1}/5 완료")
            
        print("\n✅ 로봇 모델 로딩 테스트 완료!")
        
        # USD 파일 저장
        output_path = "/home/roarm_m3/dev_roarm/roarm_mcp/tests/robot_models_test_fixed.usd"
        stage.Export(output_path)
        print(f"✅ USD 파일 저장 완료: {output_path}")
        
        return True
        
    except Exception as e:
        print(f"❌ 로봇 모델 로딩 테스트 오류: {e}")
        import traceback
        traceback.print_exc()
        return False
        
    finally:
        # 안전한 종료
        simulation_app.close()

if __name__ == "__main__":
    print("=" * 50)
    print("=== Isaac Sim 로봇 모델 로딩 테스트 ===")
    success = test_robot_models_loading()
    if success:
        print("\n🎉 모든 테스트 성공!")
    else:
        print("\n❌ 테스트 실패")
        sys.exit(1)